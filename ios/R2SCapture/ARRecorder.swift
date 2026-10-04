import ARKit
import AVFoundation
import CoreImage
import ImageIO
import Foundation
import QuartzCore

/// Records ARKit frames (modes A and B): HEVC video of every ARFrame, per-frame intrinsics, pose,
/// exposure and light estimate, LiDAR depth + confidence (raw + smoothed), ARKit audio, and at the end
/// the scene mesh, plane anchors and world map.
///
/// Mode A: ARRecorder is the ARSession delegate (delegateQueue = its own background queue).
/// Mode B: RoomPlan drives the session, so frames are polled from `session.currentFrame` (see `poll`).
/// ARFrames are never retained beyond `process(_:)`.
final class ARRecorder: NSObject, ARSessionDelegate {
    let session = ARSession()
    let queue = DispatchQueue(label: "r2s.arframes", qos: .userInitiated)

    private(set) var isRecording = false
    private var settings = CaptureSettings()
    private var dir: URL?
    private var video: VideoWriter?
    private var framesLog: LineWriter?
    private var depthBlob: BlobWriter?
    private var confBlob: BlobWriter?
    private var sdepthBlob: BlobWriter?
    private var sconfBlob: BlobWriter?
    private var lastTimestamp: TimeInterval = -1
    private var lastHires: TimeInterval = 0
    private var hiresBusy = false
    private var polling = false
    private var pollBusy = false
    private let ciContext = CIContext()

    private(set) var frameCount = 0
    private(set) var depthCount = 0
    private(set) var droppedCount = 0
    private(set) var hiresCount = 0
    private(set) var firstTimestamp: TimeInterval = 0
    private(set) var lastFrameTimestamp: TimeInterval = 0
    private(set) var trackingText = "-"
    private(set) var videoSize = CGSize.zero
    private(set) var depthSize = CGSize.zero

    struct Stats {
        var frames = 0, depth = 0, dropped = 0, hires = 0
        var duration: TimeInterval = 0
        var tracking = "-"
        var videoSize = CGSize.zero
        var depthSize = CGSize.zero
    }

    /// Thread-safe snapshot for the UI.
    func stats() -> Stats {
        queue.sync {
            Stats(frames: frameCount, depth: depthCount, dropped: droppedCount, hires: hiresCount,
                  duration: frameCount > 0 ? lastFrameTimestamp - firstTimestamp : 0, tracking: trackingText,
                  videoSize: videoSize, depthSize: depthSize)
        }
    }

    override init() {
        super.init()
        session.delegateQueue = queue
    }

    // MARK: configuration (mode A)

    static func makeConfiguration(_ s: CaptureSettings) -> ARWorldTrackingConfiguration {
        let c = ARWorldTrackingConfiguration()
        c.worldAlignment = .gravity
        c.planeDetection = [.horizontal, .vertical]
        c.environmentTexturing = .none
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) { c.frameSemantics.insert(.sceneDepth) }
        if s.smoothedDepth && ARWorldTrackingConfiguration.supportsFrameSemantics(.smoothedSceneDepth) {
            c.frameSemantics.insert(.smoothedSceneDepth)
        }
        if s.sceneMesh && ARWorldTrackingConfiguration.supportsSceneReconstruction(.meshWithClassification) {
            c.sceneReconstruction = .meshWithClassification
        }
        switch s.videoPreset {
        case .uhd30:
            if let f = ARWorldTrackingConfiguration.recommendedVideoFormatFor4KResolution { c.videoFormat = f }
        case .hd60:
            let formats = ARWorldTrackingConfiguration.supportedVideoFormats.filter {
                $0.captureDevicePosition == .back && $0.captureDeviceType == .builtInWideAngleCamera
            }
            if let f = formats.first(where: { Int($0.imageResolution.width) == 1920 && Int($0.imageResolution.height) == 1440 && $0.framesPerSecond == 60 })
                ?? formats.first(where: { Int($0.imageResolution.width) == 1920 && Int($0.imageResolution.height) == 1440 }) {
                c.videoFormat = f
            }
        }
        c.isAutoFocusEnabled = !s.lockFocus
        c.providesAudioData = s.recordAudio
        return c
    }

    /// Locks exposure / white balance on the ARKit-owned camera (iOS 16+).
    static func applyCameraLocks(_ s: CaptureSettings) {
        guard s.lockExposureAndWhiteBalance,
              let dev = ARWorldTrackingConfiguration.configurableCaptureDeviceForPrimaryCamera else { return }
        do {
            try dev.lockForConfiguration()
            if dev.isExposureModeSupported(.locked) { dev.exposureMode = .locked }
            if dev.isWhiteBalanceModeSupported(.locked) { dev.whiteBalanceMode = .locked }
            if s.lockFocus && dev.isFocusModeSupported(.locked) { dev.focusMode = .locked }
            dev.unlockForConfiguration()
        } catch {
            NSLog("camera lock failed: \(error)")
        }
    }

    func runForModeA(_ s: CaptureSettings) {
        settings = s
        session.delegate = self
        session.run(ARRecorder.makeConfiguration(s), options: [.resetTracking, .removeExistingAnchors])
        DispatchQueue.main.asyncAfter(deadline: .now() + 1.0) { ARRecorder.applyCameraLocks(s) }
    }

    func pause() {
        session.pause()
    }

    // MARK: recording

    func startRecording(dir: URL, settings s: CaptureSettings) throws {
        settings = s
        self.dir = dir
        framesLog = try LineWriter(url: dir.appendingPathComponent("frames.jsonl"))
        depthBlob = try BlobWriter(url: dir.appendingPathComponent("depth.zlib.bin"))
        confBlob = try BlobWriter(url: dir.appendingPathComponent("conf.zlib.bin"))
        if s.smoothedDepth {
            sdepthBlob = try BlobWriter(url: dir.appendingPathComponent("depth_smooth.zlib.bin"))
            sconfBlob = try BlobWriter(url: dir.appendingPathComponent("conf_smooth.zlib.bin"))
        }
        if s.hiresStillIntervalSec > 0 {
            try FileManager.default.createDirectory(at: dir.appendingPathComponent("hires"), withIntermediateDirectories: true)
        }
        queue.sync {
            video = nil
            frameCount = 0
            depthCount = 0
            droppedCount = 0
            hiresCount = 0
            lastTimestamp = -1
            lastHires = 0
            isRecording = true
        }
    }

    /// Stops frame recording and writes mesh / planes / world map. Completion runs on the main queue.
    func stopRecording(saveWorldMap: Bool, completion: @escaping () -> Void) {
        queue.async {
            self.isRecording = false
            let anchors = self.session.currentFrame?.anchors ?? []
            let dir = self.dir
            self.framesLog?.close()
            [self.depthBlob, self.confBlob, self.sdepthBlob, self.sconfBlob].forEach { $0?.close() }
            self.framesLog = nil
            self.depthBlob = nil
            self.confBlob = nil
            self.sdepthBlob = nil
            self.sconfBlob = nil
            let group = DispatchGroup()
            if let dir = dir {
                MeshExporter.writeMesh(anchors: anchors, to: dir.appendingPathComponent("mesh.ply"))
                MeshExporter.writePlanes(anchors: anchors, to: dir.appendingPathComponent("planes.json"))
                if saveWorldMap {
                    group.enter()
                    self.session.getCurrentWorldMap { map, _ in
                        if let map = map,
                           let data = try? NSKeyedArchiver.archivedData(withRootObject: map, requiringSecureCoding: true) {
                            try? data.write(to: dir.appendingPathComponent("worldmap.arworldmap"))
                        }
                        group.leave()
                    }
                }
            }
            if let v = self.video {
                group.enter()
                v.finish { group.leave() }
            }
            group.notify(queue: .main) { completion() }
        }
    }

    // MARK: mode B polling (RoomPlan owns the session delegate)

    func startPolling() {
        polling = true
        let link = CADisplayLink(target: self, selector: #selector(poll))
        link.preferredFrameRateRange = CAFrameRateRange(minimum: 60, maximum: 120, preferred: 120)
        link.add(to: .main, forMode: .common)
        displayLink = link
    }

    func stopPolling() {
        polling = false
        displayLink?.invalidate()
        displayLink = nil
    }

    private var displayLink: CADisplayLink?

    @objc private func poll() {
        guard polling, !pollBusy, let frame = session.currentFrame else { return }
        if frame.timestamp == lastPolledTimestamp { return }
        lastPolledTimestamp = frame.timestamp
        pollBusy = true
        queue.async { [weak self] in
            self?.process(frame)
            DispatchQueue.main.async { self?.pollBusy = false }
        }
    }

    private var lastPolledTimestamp: TimeInterval = -1

    // MARK: ARSessionDelegate

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        process(frame)
    }

    func session(_ session: ARSession, didOutputAudioSampleBuffer audioSampleBuffer: CMSampleBuffer) {
        guard isRecording else { return }
        video?.appendAudio(audioSampleBuffer)
    }

    // MARK: per-frame work (always on `queue`)

    private func process(_ frame: ARFrame) {
        trackingText = ARRecorder.trackingString(frame.camera.trackingState)
        guard isRecording, let dir = dir else { return }
        let t = frame.timestamp
        guard t > lastTimestamp else { return }
        lastTimestamp = t
        let pb = frame.capturedImage
        let w = CVPixelBufferGetWidth(pb)
        let h = CVPixelBufferGetHeight(pb)
        if video == nil {
            let fps = settings.videoPreset == .uhd30 ? 30 : 60
            video = try? VideoWriter(url: dir.appendingPathComponent("video.mov"), width: w, height: h, fps: fps,
                                     bitrate: Int(settings.videoBitrateMbps * 1_000_000), withAudio: settings.recordAudio)
            firstTimestamp = t
            videoSize = CGSize(width: w, height: h)
        }
        guard let video = video, video.append(copying: pb, seconds: t) else {
            droppedCount += 1
            return
        }
        let index = video.framesWritten - 1
        let K = frame.camera.intrinsics
        let T = frame.camera.transform
        var rec: [String: Any] = [
            "i": index, "t": t, "w": w, "h": h,
            "K": [K[0][0], K[1][1], K[2][0], K[2][1]].map { Double($0) },
            "T": ARRecorder.rowMajor(T),
            "track": trackingText,
            "exp": frame.camera.exposureDuration,
            "eo": Double(frame.camera.exposureOffset),
            "wm": ARRecorder.mappingString(frame.worldMappingStatus),
        ]
        if let le = frame.lightEstimate {
            rec["amb"] = Double(le.ambientIntensity)
            rec["ct"] = Double(le.ambientColorTemperature)
        }
        if let exif = frame.exifData["{Exif}"] as? [String: Any],
           let iso = (exif["ISOSpeedRatings"] as? [NSNumber])?.first {
            rec["iso"] = iso.doubleValue
        }
        if let d = frame.sceneDepth {
            if let r = depthBlob?.append(raw: DepthConvert.millimetres(d.depthMap)) { rec["d"] = r }
            if let c = d.confidenceMap, let r = confBlob?.append(raw: DepthConvert.bytes(c)) { rec["c"] = r }
            rec["dw"] = CVPixelBufferGetWidth(d.depthMap)
            rec["dh"] = CVPixelBufferGetHeight(d.depthMap)
            depthSize = CGSize(width: CVPixelBufferGetWidth(d.depthMap), height: CVPixelBufferGetHeight(d.depthMap))
            depthCount += 1
        }
        if let sd = frame.smoothedSceneDepth, let sb = sdepthBlob {
            rec["sd"] = sb.append(raw: DepthConvert.millimetres(sd.depthMap))
            if let c = sd.confidenceMap, let cb = sconfBlob { rec["sc"] = cb.append(raw: DepthConvert.bytes(c)) }
        }
        framesLog?.writeJSON(rec)
        frameCount += 1
        lastFrameTimestamp = t
        if settings.hiresStillIntervalSec > 0, !hiresBusy, t - lastHires >= settings.hiresStillIntervalSec {
            lastHires = t
            captureHighRes(dir: dir.appendingPathComponent("hires"))
        }
    }

    private func captureHighRes(dir: URL) {
        hiresBusy = true
        session.captureHighResolutionFrame { [weak self] frame, _ in
            guard let self = self else { return }
            defer { self.queue.async { self.hiresBusy = false } }
            guard let frame = frame else { return }
            let t = frame.timestamp
            let name = String(format: "hires_%.6f", t)
            let image = CIImage(cvPixelBuffer: frame.capturedImage)
            if let cs = CGColorSpace(name: CGColorSpace.sRGB),
               let jpg = self.ciContext.jpegRepresentation(of: image, colorSpace: cs,
                                                            options: [CIImageRepresentationOption(rawValue: kCGImageDestinationLossyCompressionQuality as String): 0.95]) {
                try? jpg.write(to: dir.appendingPathComponent(name + ".jpg"))
            }
            let K = frame.camera.intrinsics
            let meta: [String: Any] = [
                "t": t, "w": CVPixelBufferGetWidth(frame.capturedImage), "h": CVPixelBufferGetHeight(frame.capturedImage),
                "K": [K[0][0], K[1][1], K[2][0], K[2][1]].map { Double($0) },
                "T": ARRecorder.rowMajor(frame.camera.transform),
                "exp": frame.camera.exposureDuration,
            ]
            SessionStorage.writeJSON(meta, to: dir.appendingPathComponent(name + ".json"))
            self.queue.async { self.hiresCount += 1 }
        }
    }

    // MARK: helpers

    static func rowMajor(_ m: simd_float4x4) -> [Double] {
        var out = [Double]()
        out.reserveCapacity(16)
        for r in 0..<4 { for c in 0..<4 { out.append(Double(m[c][r])) } }
        return out
    }

    static func trackingString(_ s: ARCamera.TrackingState) -> String {
        switch s {
        case .normal: return "normal"
        case .notAvailable: return "not_available"
        case .limited(let reason):
            switch reason {
            case .initializing: return "limited:initializing"
            case .excessiveMotion: return "limited:excessive_motion"
            case .insufficientFeatures: return "limited:insufficient_features"
            case .relocalizing: return "limited:relocalizing"
            @unknown default: return "limited:unknown"
            }
        }
    }

    static func mappingString(_ s: ARFrame.WorldMappingStatus) -> String {
        switch s {
        case .notAvailable: return "not_available"
        case .limited: return "limited"
        case .extending: return "extending"
        case .mapped: return "mapped"
        @unknown default: return "unknown"
        }
    }
}

enum DepthConvert {
    /// Float32 metres -> little-endian UInt16 millimetres (0 = invalid).
    static func millimetres(_ buf: CVPixelBuffer) -> Data {
        CVPixelBufferLockBaseAddress(buf, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(buf, .readOnly) }
        let w = CVPixelBufferGetWidth(buf)
        let h = CVPixelBufferGetHeight(buf)
        let stride = CVPixelBufferGetBytesPerRow(buf)
        guard let base = CVPixelBufferGetBaseAddress(buf) else { return Data() }
        var out = [UInt16](repeating: 0, count: w * h)
        let fmt = CVPixelBufferGetPixelFormatType(buf)
        for y in 0..<h {
            let row = base + y * stride
            if fmt == kCVPixelFormatType_DepthFloat16 || fmt == kCVPixelFormatType_DisparityFloat16 {
                let p = row.assumingMemoryBound(to: Float16.self)
                for x in 0..<w { out[y * w + x] = DepthConvert.mm(Float(p[x])) }
            } else {
                let p = row.assumingMemoryBound(to: Float32.self)
                for x in 0..<w { out[y * w + x] = DepthConvert.mm(p[x]) }
            }
        }
        return out.withUnsafeBufferPointer { Data(buffer: $0) }
    }

    @inline(__always) static func mm(_ m: Float) -> UInt16 {
        guard m.isFinite, m > 0 else { return 0 }
        let v = (m * 1000).rounded()
        return v >= 65535 ? 0 : UInt16(v)
    }

    /// One byte per pixel (ARConfidenceLevel 0..2), row padding removed.
    static func bytes(_ buf: CVPixelBuffer) -> Data {
        CVPixelBufferLockBaseAddress(buf, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(buf, .readOnly) }
        let w = CVPixelBufferGetWidth(buf)
        let h = CVPixelBufferGetHeight(buf)
        let stride = CVPixelBufferGetBytesPerRow(buf)
        guard let base = CVPixelBufferGetBaseAddress(buf) else { return Data() }
        var out = Data(count: w * h)
        out.withUnsafeMutableBytes { (dst: UnsafeMutableRawBufferPointer) in
            guard let d = dst.baseAddress else { return }
            for y in 0..<h { memcpy(d + y * w, base + y * stride, w) }
        }
        return out
    }
}
