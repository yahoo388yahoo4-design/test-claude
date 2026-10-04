import ARKit
import AVFoundation
import Foundation
import SwiftUI
import UIKit

/// App state: owns the recorders, starts / stops captures and writes session.json.
final class CaptureModel: ObservableObject {
    @Published var settings = CaptureSettings.load() {
        didSet { settings.save() }
    }
    @Published private(set) var isRecording = false
    @Published private(set) var isFinishing = false
    @Published var status = "Ready"
    @Published var statsLine = ""
    @Published var sessions: [URL] = []
    @Published var uploadStatus = ""
    @Published var depthPreview: CGImage?
    /// Bumped at each recording start so the coverage overlay starts empty.
    @Published private(set) var recordingIndex = 0

    let ar = ARRecorder()
    let room = RoomPlanRecorder()
    let multicam = MultiCamRecorder()
    let sensors = SensorRecorder()

    private var sessionDir: URL?
    private var startUnix: Double = 0
    private var startUptime: Double = 0
    private var statsTimer: Timer?
    private var previewMode: CaptureMode?

    init() {
        refreshSessions()
        multicam.onDepthPreview = { [weak self] img in self?.depthPreview = img }
    }

    var lidarAvailable: Bool { ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) }

    // MARK: preview lifecycle

    func activate() {
        sensors.requestPermissions(location: settings.recordLocation)
        if settings.recordAudio { AVCaptureDevice.requestAccess(for: .audio) { _ in } }
        startPreview()
        statsTimer?.invalidate()
        statsTimer = Timer.scheduledTimer(withTimeInterval: 0.25, repeats: true) { [weak self] _ in self?.refreshStats() }
    }

    func startPreview() {
        guard !isRecording else { return }
        let mode = settings.mode
        if previewMode == mode, mode != .arkitRGBD { return }
        stopPreview()
        previewMode = mode
        switch mode {
        case .arkitRGBD:
            ar.runForModeA(settings)
            status = lidarAvailable ? "Mode A ready" : "No LiDAR: RGB + poses only"
        case .arkitRoomPlan:
            // RoomCaptureView runs the shared ARSession once scanning starts.
            status = RoomPlanRecorder.isSupported ? "Mode B ready: press record to scan the room" : "RoomPlan not supported on this device"
        case .multiCam:
            multicam.configure(includeFront: settings.multicamIncludeFront)
            multicam.startRunning()
            status = multicam.summary
        }
    }

    func stopPreview() {
        switch previewMode {
        case .arkitRGBD?, .arkitRoomPlan?: ar.pause()
        case .multiCam?: multicam.stopRunning()
        case nil: break
        }
        previewMode = nil
    }

    func modeChanged() {
        guard !isRecording else { return }
        startPreview()
    }

    // MARK: recording

    func toggleRecording() {
        if isRecording { stop() } else { start() }
    }

    private func start() {
        if settings.mode == .arkitRoomPlan && room.view == nil {
            status = "RoomPlan view not ready yet"
            return
        }
        do {
            let dir = try SessionStorage.newSession(mode: settings.mode)
            sessionDir = dir
            startUnix = Date().timeIntervalSince1970
            startUptime = ProcessInfo.processInfo.systemUptime
            writeSessionJSON(final: false)
            sensors.start(in: dir, recordLocation: settings.recordLocation)
            switch settings.mode {
            case .arkitRGBD:
                try ar.startRecording(dir: dir, settings: settings)
            case .arkitRoomPlan:
                ar.session.delegate = nil
                try ar.startRecording(dir: dir, settings: settings)
                room.start()
                ar.startPolling()
            case .multiCam:
                multicam.bitrate = Int(settings.videoBitrateMbps * 1_000_000 * 0.6)
                try multicam.startRecording(dir: dir)
            }
            UIApplication.shared.isIdleTimerDisabled = true
            recordingIndex += 1
            isRecording = true
            status = "Recording \(dir.lastPathComponent)"
        } catch {
            status = "Could not start: \(error.localizedDescription)"
        }
    }

    private func stop() {
        guard let dir = sessionDir else { return }
        isRecording = false
        isFinishing = true
        status = "Finishing…"
        UIApplication.shared.isIdleTimerDisabled = false
        let done: () -> Void = { [weak self] in
            guard let self = self else { return }
            self.sensors.stop()
            self.writeSessionJSON(final: true)
            self.isFinishing = false
            self.status = "Saved \(dir.lastPathComponent) (\(ByteCountFormatter.string(fromByteCount: SessionStorage.size(of: dir), countStyle: .file)))"
            self.refreshSessions()
            if self.settings.mode == .arkitRoomPlan {
                self.previewMode = nil
            }
        }
        switch settings.mode {
        case .arkitRGBD:
            ar.stopRecording(saveWorldMap: settings.saveWorldMap, completion: done)
        case .arkitRoomPlan:
            ar.stopPolling()
            // Stop frames first, then let RoomPlan process and export the room.
            ar.stopRecording(saveWorldMap: settings.saveWorldMap) { [weak self] in
                guard let self = self else { return }
                self.status = "Processing room…"
                var finished = false
                let finishOnce = {
                    if finished { return }
                    finished = true
                    done()
                }
                self.room.stop(exportTo: dir) { message in
                    self.status = message
                    finishOnce()
                }
                // RoomPlan may never call back (e.g. scan too short): don't hang forever.
                DispatchQueue.main.asyncAfter(deadline: .now() + 60) { finishOnce() }
            }
        case .multiCam:
            multicam.stopRecording(completion: done)
        }
    }

    private func writeSessionJSON(final: Bool) {
        guard let dir = sessionDir else { return }
        var obj: [String: Any] = [
            "format": "r2s-capture", "version": 1,
            "mode": settings.mode.rawValue,
            "platform": "ios",
            "device": SessionStorage.deviceModel,
            "os": "\(UIDevice.current.systemName) \(UIDevice.current.systemVersion)",
            "app_version": SessionStorage.appVersion,
            "start_unix": startUnix, "start_uptime": startUptime,
            "settings": settings.asDictionary,
            "complete": final,
        ]
        if final {
            obj["end_unix"] = Date().timeIntervalSince1970
            obj["end_uptime"] = ProcessInfo.processInfo.systemUptime
            switch settings.mode {
            case .arkitRGBD, .arkitRoomPlan:
                let s = ar.stats()
                obj["video"] = ["file": "video.mov", "width": Int(s.videoSize.width), "height": Int(s.videoSize.height),
                                "fps": settings.videoPreset == .uhd30 ? 30 : 60, "codec": "hevc",
                                "bitrate": Int(settings.videoBitrateMbps * 1_000_000)]
                if s.depth > 0 {
                    obj["depth"] = ["width": Int(s.depthSize.width), "height": Int(s.depthSize.height), "unit": "mm",
                                    "dtype": "uint16", "compression": "raw-deflate"]
                }
                obj["counts"] = ["frames": s.frames, "depth": s.depth, "dropped": s.dropped, "hires": s.hires,
                                 "imu": sensors.imuCount, "location": sensors.locationCount]
                if settings.mode == .arkitRoomPlan { obj["roomplan_objects"] = room.objectCount }
            case .multiCam:
                obj["cams"] = multicam.frameCounts()
                obj["counts"] = ["imu": sensors.imuCount, "location": sensors.locationCount]
                obj["notes"] = "Mode C: no ARKit poses; run tools/recover_poses.py"
            }
        }
        SessionStorage.writeJSON(obj, to: dir.appendingPathComponent("session.json"))
    }

    private func refreshStats() {
        guard isRecording || previewMode != nil else { return }
        switch settings.mode {
        case .arkitRGBD, .arkitRoomPlan:
            let s = ar.stats()
            if isRecording {
                statsLine = String(format: "%.1f s · %d frames · %d depth · %d dropped · %@ · IMU %d · GPS %d",
                                   s.duration, s.frames, s.depth, s.dropped, s.tracking, sensors.imuCount, sensors.locationCount)
            } else {
                statsLine = "tracking: \(s.tracking)"
            }
        case .multiCam:
            if isRecording {
                statsLine = multicam.frameCounts().sorted { $0.key < $1.key }.map { "\($0.key) \($0.value)" }.joined(separator: " · ")
                    + " · IMU \(sensors.imuCount)"
            } else {
                statsLine = multicam.summary
            }
        }
    }

    // MARK: sessions

    func refreshSessions() {
        sessions = SessionStorage.listSessions()
    }

    func delete(_ url: URL) {
        try? FileManager.default.removeItem(at: url)
        refreshSessions()
    }

    func upload(_ url: URL) {
        let base = settings.uploadURL
        guard !base.isEmpty else {
            uploadStatus = "Set the receiver URL in Settings first"
            return
        }
        uploadStatus = "Starting upload…"
        Task {
            do {
                try await Uploader.upload(session: url, to: base) { msg in
                    DispatchQueue.main.async { self.uploadStatus = msg }
                }
            } catch {
                await MainActor.run { self.uploadStatus = "Upload failed: \(error.localizedDescription)" }
            }
        }
    }
}
