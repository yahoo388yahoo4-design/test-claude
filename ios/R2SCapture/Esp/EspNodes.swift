import Foundation
import simd

/// Positions of the ESP32 sensing nodes in a capture session (sensing/esp32/README.md).
///
/// File: <session>/esp_nodes.json, written live while recording (tag tracking / tap on the ESP) and from
/// the session viewer (tap the ESP in a video frame). Same schema as sensing/esp32/esp_common.py and the
/// Android app: {"version":1,"format":"r2s-esp-nodes","frame":"session_world","units":"m","up":"+y",
/// "nodes":[{"node_id":1,"label":"esp-1","position":[x,y,z],"normal":[..]|null,"method":"...",...}]}
/// session_world = the ARKit world of the recording (y up, metres), the frame of frames.jsonl poses.
struct EspNode {
    let id: Int
    var position: SIMD3<Double>
    var normal: SIMD3<Double>?
    var method: String
    var frameIndex: Int?
    var t: Double?
    var confidence: Double = 1
    var observations: Int = 1
    var updated: String = EspMath.nowISO()
}

enum EspMath {
    static let nodeCount = 6
    static let tagSizeM: Double = 0.070

    /// A person's tap beats automatic detection.
    static func rank(_ method: String?) -> Int {
        switch method {
        case "tap_frame", "tap_live", "triangulated": return 4
        case "auto_tag", "auto_tag_live": return 2
        case "planned": return 0
        default: return 1
        }
    }

    static func nowISO() -> String {
        let f = ISO8601DateFormatter()
        f.formatOptions = [.withInternetDateTime]
        return f.string(from: Date())
    }

    /// K = [fx, fy, cx, cy] at the native image size; T = 16 row-major camera-to-world (camera looks down -z).
    static func unproject(K: [Double], T: [Double], u: Double, v: Double, depth: Double) -> SIMD3<Double> {
        let x = (u - K[2]) / K[0] * depth, y = -(v - K[3]) / K[1] * depth, z = -depth
        return SIMD3<Double>((0..<3).map { r in T[r * 4] * x + T[r * 4 + 1] * y + T[r * 4 + 2] * z + T[r * 4 + 3] })
    }

    static func ray(K: [Double], T: [Double], u: Double, v: Double) -> (SIMD3<Double>, SIMD3<Double>) {
        let dc = SIMD3((u - K[2]) / K[0], -(v - K[3]) / K[1], -1.0)
        let d = SIMD3<Double>((0..<3).map { r in T[r * 4] * dc.x + T[r * 4 + 1] * dc.y + T[r * 4 + 2] * dc.z })
        return (SIMD3(T[3], T[7], T[11]), simd_normalize(d))
    }

    /// World point -> (u, v, depth) at the native image size, nil when behind the camera.
    static func project(K: [Double], T: [Double], p: SIMD3<Double>) -> SIMD3<Double>? {
        let q = p - SIMD3(T[3], T[7], T[11])
        let cx = T[0] * q.x + T[4] * q.y + T[8] * q.z
        let cy = T[1] * q.x + T[5] * q.y + T[9] * q.z
        let cz = T[2] * q.x + T[6] * q.y + T[10] * q.z
        let depth = -cz
        guard depth > 1e-6 else { return nil }
        return SIMD3(K[0] * cx / depth + K[2], K[1] * -cy / depth + K[3], depth)
    }

    /// Closest point to >= 2 rays: (point, rms distance to the rays, largest angle between rays in degrees).
    static func triangulate(_ rays: [(SIMD3<Double>, SIMD3<Double>)]) -> (SIMD3<Double>, Double, Double)? {
        guard rays.count >= 2 else { return nil }
        var A = simd_double3x3()
        var b = SIMD3<Double>(repeating: 0)
        for (o, d) in rays {
            let P = matrix_identity_double3x3 - simd_double3x3(rows: [d * d.x, d * d.y, d * d.z])
            A += P
            b += P * o
        }
        guard abs(A.determinant) > 1e-12 else { return nil }
        let p = A.inverse * b
        var ss = 0.0
        for (o, d) in rays {
            let w = p - o
            let r = w - simd_dot(w, d) * d
            ss += simd_dot(r, r)
        }
        var ang = 0.0
        for i in 0..<rays.count {
            for j in (i + 1)..<rays.count {
                ang = max(ang, acos(min(1, abs(simd_dot(rays[i].1, rays[j].1)))) * 180 / .pi)
            }
        }
        return (p, sqrt(ss / Double(rays.count)), ang)
    }

    /// Median valid depth (m) around RGB pixel (u, v) of a (w, h) image from a (dw, dh) uint16-mm map.
    static func depthAt(mm: [UInt16], dw: Int, dh: Int, w: Int, h: Int, u: Double, v: Double, r: Int = 1) -> Double? {
        let du = Int((u * Double(dw) / Double(w)).rounded()), dv = Int((v * Double(dh) / Double(h)).rounded())
        guard du >= 0, du < dw, dv >= 0, dv < dh else { return nil }
        var vals: [UInt16] = []
        for y in max(0, dv - r)...min(dh - 1, dv + r) {
            for x in max(0, du - r)...min(dw - 1, du + r) where mm[y * dw + x] > 0 { vals.append(mm[y * dw + x]) }
        }
        guard !vals.isEmpty else { return nil }
        vals.sort()
        return Double(vals[vals.count / 2]) / 1000
    }

    static func median(_ pts: [SIMD3<Double>]) -> SIMD3<Double> {
        func m(_ k: Int) -> Double {
            let s = pts.map { $0[k] }.sorted()
            return s.count % 2 == 1 ? s[s.count / 2] : (s[s.count / 2 - 1] + s[s.count / 2]) / 2
        }
        return SIMD3(m(0), m(1), m(2))
    }
}

/// esp_nodes.json of one session. Thread-safe.
final class EspNodeStore {
    static let fileName = "esp_nodes.json"
    private let url: URL
    private let lock = NSLock()
    private var nodes: [Int: EspNode] = [:]
    private var extra: [String: Any] = [:]
    private(set) var dirty = false

    init(sessionDir: URL) {
        url = sessionDir.appendingPathComponent(EspNodeStore.fileName)
        load()
    }

    var all: [EspNode] { lock.lock(); defer { lock.unlock() }; return nodes.values.sorted { $0.id < $1.id } }

    func node(_ id: Int) -> EspNode? { lock.lock(); defer { lock.unlock() }; return nodes[id] }

    /// Insert or replace; a lower-ranked method never replaces a higher-ranked one unless `force`.
    @discardableResult func put(_ n: EspNode, force: Bool = false) -> Bool {
        lock.lock(); defer { lock.unlock() }
        if let old = nodes[n.id], !force, EspMath.rank(old.method) > EspMath.rank(n.method) { return false }
        nodes[n.id] = n
        dirty = true
        return true
    }

    func remove(_ id: Int) { lock.lock(); if nodes.removeValue(forKey: id) != nil { dirty = true }; lock.unlock() }

    func save() {
        lock.lock()
        var o = extra
        o["version"] = 1; o["format"] = "r2s-esp-nodes"; o["frame"] = "session_world"; o["units"] = "m"; o["up"] = "+y"
        func r4(_ v: Double) -> Double { (v * 10000).rounded() / 10000 }
        o["nodes"] = nodes.values.sorted { $0.id < $1.id }.map { n -> [String: Any] in
            [
                "node_id": n.id, "label": "esp-\(n.id)",
                "position": [r4(n.position.x), r4(n.position.y), r4(n.position.z)],
                "normal": n.normal.map { [r4($0.x), r4($0.y), r4($0.z)] as Any } ?? NSNull(),
                "method": n.method, "frame_index": n.frameIndex.map { $0 as Any } ?? NSNull(), "t": n.t.map { $0 as Any } ?? NSNull(),
                "confidence": (n.confidence * 1000).rounded() / 1000, "observations": n.observations, "updated": n.updated,
            ]
        }
        dirty = false
        lock.unlock()
        if let d = try? JSONSerialization.data(withJSONObject: o, options: [.prettyPrinted, .sortedKeys]) {
            try? d.write(to: url, options: .atomic)
        }
    }

    private func load() {
        guard let d = try? Data(contentsOf: url),
              let o = try? JSONSerialization.jsonObject(with: d) as? [String: Any] else { return }
        extra = o
        extra.removeValue(forKey: "nodes")
        for n in (o["nodes"] as? [[String: Any]]) ?? [] {
            guard let id = n["node_id"] as? Int, let p = n["position"] as? [Double], p.count == 3 else { continue }
            let nv = (n["normal"] as? [Double]).flatMap { $0.count == 3 ? SIMD3($0[0], $0[1], $0[2]) : nil }
            nodes[id] = EspNode(id: id, position: SIMD3(p[0], p[1], p[2]), normal: nv,
                                method: n["method"] as? String ?? "unknown", frameIndex: n["frame_index"] as? Int,
                                t: n["t"] as? Double, confidence: n["confidence"] as? Double ?? 1,
                                observations: n["observations"] as? Int ?? 1, updated: n["updated"] as? String ?? "")
        }
    }
}
