import ARKit
import Foundation
import RoomPlan
import UIKit

/// Mode B: RoomPlan scan sharing the ARRecorder's ARSession (iOS 17 `RoomCaptureView(frame:arSession:)`),
/// so the same pass yields RGB-D frames + poses (polled by ARRecorder) and a parametric room.
/// Exports roomplan/room.usdz (what LiteReality reads), room.json (Apple's Codable CapturedRoom) and
/// objects.json (the simplified form used by tools/convert.py).
final class RoomPlanRecorder: NSObject, RoomCaptureViewDelegate {
    private(set) var view: RoomCaptureView?
    private var exportDir: URL?
    private var onFinished: ((String) -> Void)?
    private(set) var objectCount = 0

    override init() { super.init() }

    required init?(coder: NSCoder) { super.init() }

    func encode(with coder: NSCoder) {}

    static var isSupported: Bool { RoomCaptureSession.isSupported }

    func makeView(arSession: ARSession) -> RoomCaptureView {
        if let v = view { return v }
        let v = RoomCaptureView(frame: .zero, arSession: arSession)
        v.delegate = self
        view = v
        return v
    }

    func start() {
        var config = RoomCaptureSession.Configuration()
        config.isCoachingEnabled = true
        view?.captureSession.run(configuration: config)
    }

    /// Stops scanning; `finished` is called on the main queue once the room has been exported.
    func stop(exportTo dir: URL, finished: @escaping (String) -> Void) {
        exportDir = dir
        onFinished = finished
        view?.captureSession.stop()
    }

    // MARK: RoomCaptureViewDelegate

    func captureView(shouldPresent roomDataForProcessing: CapturedRoomData, error: Error?) -> Bool {
        true
    }

    func captureView(didPresent processedResult: CapturedRoom, error: Error?) {
        let finish = onFinished
        onFinished = nil
        guard let base = exportDir else { return }
        let dir = base.appendingPathComponent("roomplan", isDirectory: true)
        try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        var message = "room exported"
        if let error = error { message = "RoomPlan error: \(error.localizedDescription)" }
        do {
            try processedResult.export(to: dir.appendingPathComponent("room.usdz"), exportOptions: .parametric)
        } catch {
            message = "usdz export failed: \(error.localizedDescription)"
        }
        if let data = try? JSONEncoder().encode(processedResult) {
            try? data.write(to: dir.appendingPathComponent("room.json"))
        }
        SessionStorage.writeJSON(RoomPlanRecorder.simplified(processedResult), to: dir.appendingPathComponent("objects.json"))
        objectCount = processedResult.objects.count
        DispatchQueue.main.async { finish?(message) }
    }

    // MARK: simplified JSON (FORMAT.md roomplan/objects.json)

    static func simplified(_ room: CapturedRoom) -> [String: Any] {
        func conf(_ c: CapturedRoom.Confidence) -> String {
            switch c {
            case .high: return "high"
            case .medium: return "medium"
            case .low: return "low"
            @unknown default: return "unknown"
            }
        }
        func vec(_ v: simd_float3) -> [Double] { [Double(v.x), Double(v.y), Double(v.z)] }
        func surface(_ s: CapturedRoom.Surface) -> [String: Any] {
            var d: [String: Any] = [
                "id": s.identifier.uuidString,
                "category": String(describing: s.category),
                "confidence": conf(s.confidence),
                "dims": vec(s.dimensions),
                "T": ARRecorder.rowMajor(s.transform),
            ]
            if let parent = s.parentIdentifier { d["parent"] = parent.uuidString }
            return d
        }
        let objects: [[String: Any]] = room.objects.map { o in
            var d: [String: Any] = [
                "id": o.identifier.uuidString,
                "category": String(describing: o.category),
                "confidence": conf(o.confidence),
                "dims": vec(o.dimensions),
                "T": ARRecorder.rowMajor(o.transform),
            ]
            if let parent = o.parentIdentifier { d["parent"] = parent.uuidString }
            return d
        }
        return [
            "objects": objects,
            "walls": room.walls.map(surface),
            "doors": room.doors.map(surface),
            "windows": room.windows.map(surface),
            "openings": room.openings.map(surface),
            "floors": room.floors.map(surface),
        ]
    }
}
