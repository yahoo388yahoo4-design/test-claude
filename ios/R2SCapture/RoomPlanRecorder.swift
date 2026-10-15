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
    /// Room name written into roommap/room_map.json (HOME.md); set before [stop].
    var roomName = ""
    /// Path of the last room map written (nil if none).
    private(set) var lastRoomMap: URL?

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
        // RoomPlan's mesh export (surfaces + objects as meshes) next to the parametric one
        try? processedResult.export(to: dir.appendingPathComponent("room_mesh.usdz"), exportOptions: .mesh)
        let mapDir = base.appendingPathComponent("roommap", isDirectory: true)
        try? FileManager.default.createDirectory(at: mapDir, withIntermediateDirectories: true)
        let mapURL = mapDir.appendingPathComponent("room_map.json")
        SessionStorage.writeJSON(RoomPlanRecorder.roomMap(processedResult, name: roomName.isEmpty ? "Room" : roomName,
                                                          session: base.lastPathComponent), to: mapURL)
        lastRoomMap = mapURL
        objectCount = processedResult.objects.count
        DispatchQueue.main.async { finish?(message) }
    }

    // MARK: room map (HOME.md roommap/room_map.json)

    static func roomMap(_ room: CapturedRoom, name: String, session: String) -> [String: Any] {
        func conf(_ c: CapturedRoom.Confidence) -> Double {
            switch c {
            case .high: return 0.9
            case .medium: return 0.6
            case .low: return 0.3
            @unknown default: return 0.5
            }
        }
        func element(id: UUID, category: String, confidence: CapturedRoom.Confidence, transform t: simd_float4x4,
                     dims: simd_float3, parent: UUID?) -> RoomMapCore.Element {
            var ax = Double(t.columns.0.x), az = Double(t.columns.0.z)
            let n = (ax * ax + az * az).squareRoot()
            if n > 1e-6 { ax /= n; az /= n } else { ax = 1; az = 0 }
            return RoomMapCore.Element(id: id.uuidString, category: category, confidence: conf(confidence),
                                       cx: Double(t.columns.3.x), cy: Double(t.columns.3.y), cz: Double(t.columns.3.z),
                                       axisX: ax, axisZ: az, width: Double(dims.x), height: Double(dims.y), depth: Double(dims.z),
                                       parent: parent?.uuidString)
        }
        func surface(_ s: CapturedRoom.Surface, _ cat: String) -> RoomMapCore.Element {
            element(id: s.identifier, category: cat, confidence: s.confidence, transform: s.transform, dims: s.dimensions, parent: s.parentIdentifier)
        }
        var corners: [[Double]] = []
        var floorY: Double?
        if let f = room.floors.first {
            for c in f.polygonCorners {
                let w = f.transform * simd_float4(c.x, c.y, c.z, 1)
                corners.append([Double(w.x), Double(w.y), Double(w.z)])
            }
            floorY = Double(f.transform.columns.3.y)
        }
        let objects = room.objects.map { o in
            element(id: o.identifier, category: RoomMapCore.category(String(describing: o.category)), confidence: o.confidence,
                    transform: o.transform, dims: o.dimensions, parent: o.parentIdentifier)
        }
        return RoomMapCore.build(name: name, session: session, device: SessionStorage.deviceModel,
                                 walls: room.walls.map { surface($0, "wall") }, doors: room.doors.map { surface($0, "door") },
                                 windows: room.windows.map { surface($0, "window") }, openings: room.openings.map { surface($0, "opening") },
                                 objects: objects, floorCorners: corners, floorY: floorY, nowUnix: Date().timeIntervalSince1970)
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
