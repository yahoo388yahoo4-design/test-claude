import AVFoundation
import CoreVideo

/// HEVC writer: one sample per appended frame, presentation time = the frame's uptime timestamp.
/// Not thread-safe: call every method from the same serial queue.
final class VideoWriter {
    private let writer: AVAssetWriter
    private let videoInput: AVAssetWriterInput
    private let adaptor: AVAssetWriterInputPixelBufferAdaptor
    private var audioInput: AVAssetWriterInput?
    private var pool: CVPixelBufferPool?
    private(set) var started = false
    private(set) var framesWritten = 0
    private(set) var framesDropped = 0
    let width: Int
    let height: Int

    init(url: URL, width: Int, height: Int, fps: Int, bitrate: Int, withAudio: Bool) throws {
        self.width = width
        self.height = height
        try? FileManager.default.removeItem(at: url)
        writer = try AVAssetWriter(outputURL: url, fileType: .mov)
        let compression: [String: Any] = [
            AVVideoAverageBitRateKey: bitrate,
            AVVideoExpectedSourceFrameRateKey: fps,
            AVVideoMaxKeyFrameIntervalKey: fps,
            AVVideoAllowFrameReorderingKey: false,
        ]
        videoInput = AVAssetWriterInput(mediaType: .video, outputSettings: [
            AVVideoCodecKey: AVVideoCodecType.hevc,
            AVVideoWidthKey: width,
            AVVideoHeightKey: height,
            AVVideoCompressionPropertiesKey: compression,
        ])
        videoInput.expectsMediaDataInRealTime = true
        let attrs: [String: Any] = [
            kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_420YpCbCr8BiPlanarFullRange,
            kCVPixelBufferWidthKey as String: width,
            kCVPixelBufferHeightKey as String: height,
            kCVPixelBufferIOSurfacePropertiesKey as String: [String: Any](),
        ]
        adaptor = AVAssetWriterInputPixelBufferAdaptor(assetWriterInput: videoInput, sourcePixelBufferAttributes: attrs)
        guard writer.canAdd(videoInput) else { throw NSError(domain: "VideoWriter", code: 1) }
        writer.add(videoInput)
        if withAudio {
            let a = AVAssetWriterInput(mediaType: .audio, outputSettings: [
                AVFormatIDKey: kAudioFormatMPEG4AAC,
                AVSampleRateKey: 48_000,
                AVNumberOfChannelsKey: 1,
                AVEncoderBitRateKey: 128_000,
            ])
            a.expectsMediaDataInRealTime = true
            if writer.canAdd(a) {
                writer.add(a)
                audioInput = a
            }
        }
        guard writer.startWriting() else { throw writer.error ?? NSError(domain: "VideoWriter", code: 2) }
    }

    /// Copies `source` into a buffer we own (so ARKit's capture pool is never starved), then appends it.
    @discardableResult
    func append(copying source: CVPixelBuffer, seconds: TimeInterval) -> Bool {
        // AVAssetWriter throws an uncatchable exception if used after it failed (e.g. encoder busy).
        guard writer.status == .writing else { framesDropped += 1; return false }
        let time = CMTime(seconds: seconds, preferredTimescale: 1_000_000)
        if !started {
            writer.startSession(atSourceTime: time)
            started = true
        }
        guard videoInput.isReadyForMoreMediaData else { framesDropped += 1; return false }
        if pool == nil { pool = adaptor.pixelBufferPool }
        guard let pool = pool, let copy = PixelBufferCopy.copy(source, pool: pool) else { framesDropped += 1; return false }
        let ok = adaptor.append(copy, withPresentationTime: time)
        if ok { framesWritten += 1 } else { framesDropped += 1 }
        return ok
    }

    /// Appends an already-owned video sample buffer (AVFoundation outputs).
    @discardableResult
    func append(sampleBuffer: CMSampleBuffer) -> Bool {
        // AVAssetWriter throws an uncatchable exception if used after it failed (e.g. encoder busy).
        guard writer.status == .writing else { framesDropped += 1; return false }
        let time = CMSampleBufferGetPresentationTimeStamp(sampleBuffer)
        if !started {
            writer.startSession(atSourceTime: time)
            started = true
        }
        guard videoInput.isReadyForMoreMediaData else { framesDropped += 1; return false }
        let ok = videoInput.append(sampleBuffer)
        if ok { framesWritten += 1 } else { framesDropped += 1 }
        return ok
    }

    func appendAudio(_ sampleBuffer: CMSampleBuffer) {
        guard started, writer.status == .writing, let a = audioInput, a.isReadyForMoreMediaData else { return }
        a.append(sampleBuffer)
    }

    func finish(_ completion: @escaping () -> Void) {
        guard started, writer.status == .writing else {
            if writer.status == .writing { writer.cancelWriting() }
            if let e = writer.error { NSLog("VideoWriter failed: \(e)") }
            completion()
            return
        }
        videoInput.markAsFinished()
        audioInput?.markAsFinished()
        writer.finishWriting { completion() }
    }
}

enum PixelBufferCopy {
    static func copy(_ src: CVPixelBuffer, pool: CVPixelBufferPool) -> CVPixelBuffer? {
        var out: CVPixelBuffer?
        guard CVPixelBufferPoolCreatePixelBuffer(nil, pool, &out) == kCVReturnSuccess, let dst = out else { return nil }
        CVPixelBufferLockBaseAddress(src, .readOnly)
        CVPixelBufferLockBaseAddress(dst, [])
        defer {
            CVPixelBufferUnlockBaseAddress(dst, [])
            CVPixelBufferUnlockBaseAddress(src, .readOnly)
        }
        let planes = CVPixelBufferGetPlaneCount(src)
        if planes == 0 {
            guard let s = CVPixelBufferGetBaseAddress(src), let d = CVPixelBufferGetBaseAddress(dst) else { return nil }
            copyRows(s, CVPixelBufferGetBytesPerRow(src), d, CVPixelBufferGetBytesPerRow(dst),
                     min(CVPixelBufferGetHeight(src), CVPixelBufferGetHeight(dst)))
        } else {
            guard CVPixelBufferGetPlaneCount(dst) == planes else { return nil }
            for p in 0..<planes {
                guard let s = CVPixelBufferGetBaseAddressOfPlane(src, p),
                      let d = CVPixelBufferGetBaseAddressOfPlane(dst, p) else { return nil }
                copyRows(s, CVPixelBufferGetBytesPerRowOfPlane(src, p), d, CVPixelBufferGetBytesPerRowOfPlane(dst, p),
                         min(CVPixelBufferGetHeightOfPlane(src, p), CVPixelBufferGetHeightOfPlane(dst, p)))
            }
        }
        return dst
    }

    private static func copyRows(_ s: UnsafeMutableRawPointer, _ sStride: Int, _ d: UnsafeMutableRawPointer,
                                 _ dStride: Int, _ rows: Int) {
        if sStride == dStride {
            memcpy(d, s, sStride * rows)
        } else {
            let n = min(sStride, dStride)
            for r in 0..<rows { memcpy(d + r * dStride, s + r * sStride, n) }
        }
    }
}
