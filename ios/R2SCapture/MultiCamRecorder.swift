import AVFoundation
import Foundation

/// Mode C: AVCaptureMultiCamSession with as many back/front cameras as the device allows at once,
/// plus the LiDAR depth camera (AVCaptureDepthDataOutput, denser than ARKit's 256x192, with
/// calibration and lens distortion). ARKit cannot run at the same time, so there are no 6-DoF poses;
/// tools/recover_poses.py estimates them offline (SfM, metric scale from LiDAR) using the IMU log.
final class MultiCamRecorder: NSObject, AVCaptureVideoDataOutputSampleBufferDelegate, AVCaptureDepthDataOutputDelegate {
    let session = AVCaptureMultiCamSession()
    private let queue = DispatchQueue(label: "r2s.multicam", qos: .userInitiated)

    private final class Stream {
        let name: String
        let device: AVCaptureDevice
        let output: AVCaptureVideoDataOutput
        let port: AVCaptureInput.Port
        var writer: VideoWriter?
        var log: LineWriter?
        var count = 0
        init(name: String, device: AVCaptureDevice, output: AVCaptureVideoDataOutput, port: AVCaptureInput.Port) {
            self.name = name
            self.device = device
            self.output = output
            self.port = port
        }
    }

    private var streams: [Stream] = []
    private var depthOutput: AVCaptureDepthDataOutput?
    private var depthDevice: AVCaptureDevice?
    private var depthBlob: BlobWriter?
    private var depthLog: LineWriter?
    private var calibrationWritten = false
    private var camsDir: URL?
    private var recording = false
    private(set) var depthCount = 0
    private(set) var configured = false
    private(set) var summary = "not configured"
    var previewPort: AVCaptureInput.Port? { streams.first?.port }
    private weak var previewLayer: AVCaptureVideoPreviewLayer?
    var bitrate = 30_000_000

    static var isSupported: Bool { AVCaptureMultiCamSession.isMultiCamSupported }

    // MARK: configuration

    func configure(includeFront: Bool) {
        guard !configured, MultiCamRecorder.isSupported else {
            if !MultiCamRecorder.isSupported { summary = "multi-camera capture not supported on this device" }
            connectPreview()
            return
        }
        var types: [AVCaptureDevice.DeviceType] = [.builtInLiDARDepthCamera, .builtInWideAngleCamera,
                                                    .builtInUltraWideCamera, .builtInTelephotoCamera]
        if includeFront { types.append(.builtInTrueDepthCamera) }
        let discovery = AVCaptureDevice.DiscoverySession(deviceTypes: types, mediaType: .video, position: .unspecified)
        func score(_ set: Set<AVCaptureDevice>) -> Int {
            set.reduce(0) { acc, d in
                switch d.deviceType {
                case .builtInLiDARDepthCamera: return acc + 10
                case .builtInUltraWideCamera: return acc + 6
                case .builtInWideAngleCamera: return acc + (d.position == .front ? (includeFront ? 2 : -100) : 5)
                case .builtInTelephotoCamera: return acc + 4
                case .builtInTrueDepthCamera: return acc + (includeFront ? 2 : -100)
                default: return acc
                }
            }
        }
        let sets = discovery.supportedMultiCamDeviceSets
        guard let best = sets.max(by: { score($0) < score($1) }) else {
            summary = "no supported multi-camera device set"
            return
        }
        session.beginConfiguration()
        let ordered = best.sorted { score([$0]) > score([$1]) }
        for device in ordered {
            addCamera(device)
        }
        session.commitConfiguration()
        configured = true
        connectPreview()
        let names = streams.map { $0.name } + (depthOutput != nil ? ["lidar_depth"] : [])
        summary = "streams: \(names.joined(separator: ", ")) · hw cost \(String(format: "%.2f", session.hardwareCost))"
        if session.hardwareCost > 1.0 {
            summary += " (over budget: drop the front camera)"
        }
    }

    private func streamName(_ d: AVCaptureDevice) -> String {
        switch d.deviceType {
        case .builtInLiDARDepthCamera: return "lidar_rgb"
        case .builtInUltraWideCamera: return "ultrawide"
        case .builtInTelephotoCamera: return "tele"
        case .builtInTrueDepthCamera: return "front"
        default: return d.position == .front ? "front" : "wide"
        }
    }

    private func pickFormat(_ device: AVCaptureDevice, needsDepth: Bool) -> AVCaptureDevice.Format? {
        let candidates = device.formats.filter { f in
            guard f.isMultiCamSupported else { return false }
            if needsDepth && f.supportedDepthDataFormats.isEmpty { return false }
            let d = CMVideoFormatDescriptionGetDimensions(f.formatDescription)
            return d.width <= 1920 && CMFormatDescriptionGetMediaSubType(f.formatDescription) == kCVPixelFormatType_420YpCbCr8BiPlanarFullRange
        }
        func dims(_ f: AVCaptureDevice.Format) -> (Int32, Int32) {
            let d = CMVideoFormatDescriptionGetDimensions(f.formatDescription)
            return (d.width, d.height)
        }
        // prefer 4:3 (ARKitScenes-like), then the largest
        return candidates.max { a, b in
            let (aw, ah) = dims(a), (bw, bh) = dims(b)
            let a43 = aw * 3 == ah * 4, b43 = bw * 3 == bh * 4
            if a43 != b43 { return !a43 }
            return aw * ah < bw * bh
        }
    }

    private func addCamera(_ device: AVCaptureDevice) {
        let isLiDAR = device.deviceType == .builtInLiDARDepthCamera
        guard let input = try? AVCaptureDeviceInput(device: device), session.canAddInput(input) else { return }
        session.addInputWithNoConnections(input)
        if let fmt = pickFormat(device, needsDepth: isLiDAR) {
            do {
                try device.lockForConfiguration()
                device.activeFormat = fmt
                if isLiDAR {
                    let depthFormats = fmt.supportedDepthDataFormats.filter {
                        CMFormatDescriptionGetMediaSubType($0.formatDescription) == kCVPixelFormatType_DepthFloat32
                    }
                    if let df = depthFormats.max(by: {
                        CMVideoFormatDescriptionGetDimensions($0.formatDescription).width <
                            CMVideoFormatDescriptionGetDimensions($1.formatDescription).width
                    }) {
                        device.activeDepthDataFormat = df
                    }
                }
                device.activeVideoMinFrameDuration = CMTime(value: 1, timescale: 30)
                device.activeVideoMaxFrameDuration = CMTime(value: 1, timescale: 30)
                device.unlockForConfiguration()
            } catch {
                NSLog("format config failed: \(error)")
            }
        }
        guard let port = input.ports(for: .video, sourceDeviceType: device.deviceType, sourceDevicePosition: device.position).first else { return }
        let out = AVCaptureVideoDataOutput()
        out.videoSettings = [kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_420YpCbCr8BiPlanarFullRange]
        out.alwaysDiscardsLateVideoFrames = true
        out.setSampleBufferDelegate(self, queue: queue)
        guard session.canAddOutput(out) else { return }
        session.addOutputWithNoConnections(out)
        let conn = AVCaptureConnection(inputPorts: [port], output: out)
        guard session.canAddConnection(conn) else { session.removeOutput(out); return }
        session.addConnection(conn)
        if conn.isCameraIntrinsicMatrixDeliverySupported { conn.isCameraIntrinsicMatrixDeliveryEnabled = true }
        if conn.isVideoStabilizationSupported { conn.preferredVideoStabilizationMode = .off }
        var name = streamName(device)
        if streams.contains(where: { $0.name == name }) { name += "_\(streams.count)" }
        streams.append(Stream(name: name, device: device, output: out, port: port))

        if isLiDAR, let dport = input.ports(for: .depthData, sourceDeviceType: device.deviceType, sourceDevicePosition: device.position).first {
            let dout = AVCaptureDepthDataOutput()
            dout.isFilteringEnabled = false
            dout.alwaysDiscardsLateDepthData = true
            dout.setDelegate(self, callbackQueue: queue)
            if session.canAddOutput(dout) {
                session.addOutputWithNoConnections(dout)
                let dconn = AVCaptureConnection(inputPorts: [dport], output: dout)
                if session.canAddConnection(dconn) {
                    session.addConnection(dconn)
                    depthOutput = dout
                    depthDevice = device
                } else {
                    session.removeOutput(dout)
                }
            }
        }
    }

    /// The preview view can appear before or after `configure`, so whichever comes second makes the
    /// connection from the first (main) camera to the layer.
    func attachPreview(_ layer: AVCaptureVideoPreviewLayer) {
        previewLayer = layer
        connectPreview()
    }

    private func connectPreview() {
        guard let layer = previewLayer, layer.connection == nil, let port = previewPort else { return }
        if layer.session !== session { layer.setSessionWithNoConnection(session) }
        let conn = AVCaptureConnection(inputPort: port, videoPreviewLayer: layer)
        session.beginConfiguration()
        if session.canAddConnection(conn) {
            session.addConnection(conn)
            if conn.isVideoRotationAngleSupported(90) { conn.videoRotationAngle = 90 }  // app is portrait-only
        } else {
            summary += " (no preview: over the camera budget)"
        }
        session.commitConfiguration()
    }

    func startRunning() {
        queue.async { if !self.session.isRunning { self.session.startRunning() } }
    }

    func stopRunning() {
        queue.async { if self.session.isRunning { self.session.stopRunning() } }
    }

    // MARK: recording

    func startRecording(dir: URL) throws {
        let cams = dir.appendingPathComponent("cams", isDirectory: true)
        try FileManager.default.createDirectory(at: cams, withIntermediateDirectories: true)
        let logs = try streams.map { try LineWriter(url: cams.appendingPathComponent("\($0.name).jsonl")) }
        let dblob = try depthOutput != nil ? BlobWriter(url: cams.appendingPathComponent("lidar_depth.zlib.bin")) : nil
        let dlog = try depthOutput != nil ? LineWriter(url: cams.appendingPathComponent("lidar_depth.jsonl")) : nil
        queue.sync {
            camsDir = cams
            for (s, l) in zip(streams, logs) {
                s.log = l
                s.writer = nil
                s.count = 0
            }
            depthBlob = dblob
            depthLog = dlog
            depthCount = 0
            calibrationWritten = false
            recording = true
        }
        writeExtrinsics(to: cams.appendingPathComponent("extrinsics.json"))
    }

    func stopRecording(completion: @escaping () -> Void) {
        queue.async {
            self.recording = false
            let group = DispatchGroup()
            for s in self.streams {
                s.log?.close()
                s.log = nil
                if let w = s.writer {
                    group.enter()
                    w.finish { group.leave() }
                }
                s.writer = nil
            }
            self.depthBlob?.close()
            self.depthLog?.close()
            self.depthBlob = nil
            self.depthLog = nil
            group.notify(queue: .main) { completion() }
        }
    }

    func frameCounts() -> [String: Int] {
        queue.sync {
            var d = Dictionary(streams.map { ($0.name, $0.count) }, uniquingKeysWith: +)
            if depthOutput != nil { d["lidar_depth"] = depthCount }
            return d
        }
    }

    // MARK: delegates (on `queue`)

    func captureOutput(_ output: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
        guard recording, let cams = camsDir, let s = streams.first(where: { $0.output === output }),
              let pb = CMSampleBufferGetImageBuffer(sampleBuffer) else { return }
        let w = CVPixelBufferGetWidth(pb)
        let h = CVPixelBufferGetHeight(pb)
        if s.writer == nil {
            s.writer = try? VideoWriter(url: cams.appendingPathComponent("\(s.name).mov"), width: w, height: h, fps: 30,
                                        bitrate: bitrate, withAudio: false)
        }
        guard let writer = s.writer, writer.append(sampleBuffer: sampleBuffer) else { return }
        let t = hostSeconds(CMSampleBufferGetPresentationTimeStamp(sampleBuffer))
        var rec: [String: Any] = ["i": writer.framesWritten - 1, "t": t, "w": w, "h": h,
                                  "exp": s.device.exposureDuration.seconds, "iso": Double(s.device.iso),
                                  "lens_pos": Double(s.device.lensPosition)]
        if let K = MultiCamRecorder.intrinsics(sampleBuffer) {
            rec["K"] = [K[0][0], K[1][1], K[2][0], K[2][1]].map { Double($0) }
        } else {
            rec["K"] = NSNull()
        }
        s.log?.writeJSON(rec)
        s.count += 1
    }

    func depthDataOutput(_ output: AVCaptureDepthDataOutput, didOutput depthData: AVDepthData, timestamp: CMTime,
                         connection: AVCaptureConnection) {
        guard recording, let blob = depthBlob else { return }
        let depth = depthData.depthDataType == kCVPixelFormatType_DepthFloat32 ? depthData
            : depthData.converting(toDepthDataType: kCVPixelFormatType_DepthFloat32)
        let map = depth.depthDataMap
        let range = blob.append(raw: DepthConvert.millimetres(map))
        var rec: [String: Any] = ["i": depthCount, "t": hostSeconds(timestamp),
                                  "w": CVPixelBufferGetWidth(map), "h": CVPixelBufferGetHeight(map), "d": range,
                                  "quality": depth.depthDataQuality == .high ? "high" : "low",
                                  "accuracy": depth.depthDataAccuracy == .absolute ? "absolute" : "relative"]
        if let cal = depth.cameraCalibrationData {
            let K = cal.intrinsicMatrix
            rec["K"] = [K[0][0], K[1][1], K[2][0], K[2][1]].map { Double($0) }
            rec["ref_wh"] = [Double(cal.intrinsicMatrixReferenceDimensions.width), Double(cal.intrinsicMatrixReferenceDimensions.height)]
            if !calibrationWritten, let cams = camsDir {
                calibrationWritten = true
                writeCalibration(cal, to: cams.appendingPathComponent("calibration.json"))
            }
        }
        depthLog?.writeJSON(rec)
        depthCount += 1
    }

    // MARK: helpers

    private func hostSeconds(_ t: CMTime) -> Double {
        let clock = session.synchronizationClock ?? CMClockGetHostTimeClock()
        return CMSyncConvertTime(t, from: clock, to: CMClockGetHostTimeClock()).seconds
    }

    static func intrinsics(_ sb: CMSampleBuffer) -> matrix_float3x3? {
        guard let att = CMGetAttachment(sb, key: kCMSampleBufferAttachmentKey_CameraIntrinsicMatrix, attachmentModeOut: nil),
              let data = att as? Data, data.count >= MemoryLayout<matrix_float3x3>.size else { return nil }
        return data.withUnsafeBytes { $0.loadUnaligned(as: matrix_float3x3.self) }
    }

    private func writeCalibration(_ cal: AVCameraCalibrationData, to url: URL) {
        func floats(_ d: Data?) -> [Double] {
            guard let d = d else { return [] }
            return d.withUnsafeBytes { Array($0.bindMemory(to: Float.self)).map { Double($0) } }
        }
        let K = cal.intrinsicMatrix
        let E = cal.extrinsicMatrix
        var ext = [[Double]]()
        for r in 0..<3 { ext.append((0..<4).map { Double(E[$0][r]) }) }
        let obj: [String: Any] = [
            "camera": "lidar_rgb",
            "intrinsics": (0..<3).map { r in (0..<3).map { c in Double(K[c][r]) } },
            "reference_dims": [Double(cal.intrinsicMatrixReferenceDimensions.width), Double(cal.intrinsicMatrixReferenceDimensions.height)],
            "pixel_size_mm": Double(cal.pixelSize),
            "extrinsic_3x4": ext,
            "lens_distortion_center": [Double(cal.lensDistortionCenter.x), Double(cal.lensDistortionCenter.y)],
            "lens_distortion_lut": floats(cal.lensDistortionLookupTable),
            "inverse_lut": floats(cal.inverseLensDistortionLookupTable),
        ]
        SessionStorage.writeJSON(obj, to: url)
    }

    /// Factory extrinsics between every pair of recorded cameras (iOS 17 AVCaptureDevice.extrinsicMatrix).
    private func writeExtrinsics(to url: URL) {
        var out: [String: Any] = [:]
        for a in streams {
            var row: [String: Any] = [:]
            for b in streams where a !== b {
                guard let data = AVCaptureDevice.extrinsicMatrix(from: a.device, to: b.device),
                      data.count >= MemoryLayout<matrix_float4x3>.size else { continue }
                let m = data.withUnsafeBytes { $0.loadUnaligned(as: matrix_float4x3.self) }
                row[b.name] = (0..<3).map { r in (0..<4).map { c in Double(m[c][r]) } }
            }
            out[a.name] = ["extrinsics_to": row, "device_type": a.device.deviceType.rawValue,
                           "field_of_view_deg": Double(a.device.activeFormat.videoFieldOfView)]
        }
        SessionStorage.writeJSON(out, to: url)
    }
}
