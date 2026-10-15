import Foundation

/// Room map (HOME.md "room_map.json") from RoomPlan geometry, platform independent so it is unit tested on Linux
/// (ios/NavTests/roommap). Input is in the ARKit world (y up, metres); the output is a floor plan: x right,
/// y up the map, z up from the floor, origin at the room's min corner, the dominant wall direction along x.
/// The same frame and fields as the Android RoomMap.kt.
enum RoomMapCore {
    /// A box-like element: centre, x axis (horizontal unit vector along its width, world x/z), dimensions
    /// (width, height, depth) as RoomPlan reports them.
    struct Element {
        var id: String
        var category: String
        var confidence: Double
        var cx: Double, cy: Double, cz: Double
        var axisX: Double, axisZ: Double
        var width: Double, height: Double, depth: Double
        var parent: String?
    }

    /// RoomPlan category names -> the shared vocabulary (Android uses the same names).
    static func category(_ roomPlanName: String) -> String {
        switch roomPlanName {
        case "television": return "tv"
        case "washerDryer": return "washer_dryer"
        default:
            // camelCase -> snake_case
            var out = ""
            for ch in roomPlanName {
                if ch.isUppercase { out += "_" + ch.lowercased() } else { out.append(ch) }
            }
            return out
        }
    }

    static func slug(_ name: String) -> String {
        let s = name.lowercased().map { ($0.isLetter || $0.isNumber) && $0.isASCII ? $0 : "_" }
        let joined = String(s).split(separator: "_").joined(separator: "_")
        return joined.isEmpty ? "room" : joined
    }

    struct Plan {
        var theta: Double, ox: Double, oy: Double, floorY: Double
        func toPlan(_ x: Double, _ y: Double, _ z: Double) -> [Double] {
            let c = cos(theta), s = sin(theta)
            let rx = c * x + s * z, rz = -s * x + c * z
            return [rx - ox, -rz - oy, y - floorY]
        }
        func yaw(_ dx: Double, _ dz: Double) -> Double {
            let c = cos(theta), s = sin(theta)
            return atan2(-(-s * dx + c * dz), c * dx + s * dz)
        }
        var worldToPlan: [Double] {
            let c = cos(theta), s = sin(theta)
            return [c, 0, s, -ox, s, 0, -c, -oy, 0, 1, 0, -floorY, 0, 0, 0, 1]
        }
    }

    /// [floorCorners] world points of the floor outline (may be empty: the wall ends are used then).
    static func build(name: String, session: String, device: String, walls: [Element], doors: [Element],
                      windows: [Element], openings: [Element], objects: [Element], floorCorners: [[Double]],
                      floorY floorY0: Double?, nowUnix: Double) -> [String: Any] {
        let wallBottoms: [Double] = walls.map { (e: Element) -> Double in e.cy - e.height / 2 }
        let objectBottoms: [Double] = objects.map { (e: Element) -> Double in e.cy - e.height / 2 }
        let floorY: Double = floorY0 ?? wallBottoms.min() ?? objectBottoms.min() ?? 0
        // dominant direction mod 90 deg, length weighted
        var sc = 0.0, ss = 0.0
        for w in walls {
            let a = atan2(w.axisZ, w.axisX) * 4
            sc += cos(a) * w.width; ss += sin(a) * w.width
        }
        let theta = walls.isEmpty ? 0 : atan2(ss, sc) / 4
        var plan = Plan(theta: theta, ox: 0, oy: 0, floorY: floorY)
        func ends(_ e: Element) -> ([Double], [Double]) {
            let h = e.width / 2
            return (plan.toPlan(e.cx - e.axisX * h, e.cy, e.cz - e.axisZ * h), plan.toPlan(e.cx + e.axisX * h, e.cy, e.cz + e.axisZ * h))
        }
        var pts: [[Double]] = floorCorners.map { plan.toPlan($0[0], $0[1], $0[2]) }
        for w in walls { let (a, b) = ends(w); pts.append(a); pts.append(b) }
        if pts.isEmpty { for o in objects { pts.append(plan.toPlan(o.cx, o.cy, o.cz)) } }
        if pts.isEmpty { pts = [[0, 0, 0]] }
        let minX = pts.map { $0[0] }.min()!, minY = pts.map { $0[1] }.min()!
        let maxX = pts.map { $0[0] }.max()!, maxY = pts.map { $0[1] }.max()!
        plan.ox = minX; plan.oy = minY
        func r(_ v: Double) -> Double { (v * 1000).rounded() / 1000 }
        func p2(_ p: [Double]) -> [Double] { [r(p[0]), r(p[1])] }

        var floor: [[Double]]
        if floorCorners.count >= 3 {
            floor = floorCorners.map { p2(plan.toPlan($0[0], $0[1], $0[2])) }
        } else {
            let hull = convexHull(pts.map { [$0[0] - minX, $0[1] - minY] })
            floor = hull.count >= 3 ? hull.map(p2) : [[0, 0], [r(maxX - minX), 0], [r(maxX - minX), r(maxY - minY)], [0, r(maxY - minY)]]
        }
        func seg(_ e: Element) -> [String: Any] {
            let (a, b) = ends(e)
            var d: [String: Any] = ["id": e.id, "a": p2(a), "b": p2(b), "height": r(e.height),
                                    "bottom": r(e.cy - e.height / 2 - floorY), "confidence": e.confidence]
            if let p = e.parent { d["parent"] = p }
            return d
        }
        var counts: [String: Int] = [:]
        let objs: [[String: Any]] = objects.map { o in
            let n = (counts[o.category] ?? 0) + 1
            counts[o.category] = n
            let c = plan.toPlan(o.cx, o.cy, o.cz)
            let label = o.category.replacingOccurrences(of: "_", with: " ").capitalized + (n > 1 ? " \(n)" : "")
            return ["id": "\(o.category)_\(n)", "uuid": o.id, "category": o.category, "label": label,
                    "confidence": o.confidence, "center": [r(c[0]), r(c[1]), r(c[2])],
                    "size": [r(o.width), r(o.depth), r(o.height)], "yaw": r(plan.yaw(o.axisX, o.axisZ))]
        }
        let height = walls.map { $0.height }.max() ?? 2.5
        return [
            "format": "r2s-roommap", "version": 1,
            "id": slug(name), "name": name,
            "source": ["platform": "ios", "session": session, "device": device, "method": "RoomPlan CapturedRoom"],
            "created_unix": nowUnix, "units": "m",
            "frame": "floor plan: x right, y up the map, z up from the floor; origin = room min corner",
            "world_to_room": plan.worldToPlan.map { ($0 * 1e4).rounded() / 1e4 },
            "bounds": [0, 0, r(maxX - minX), r(maxY - minY)],
            "height": r(height),
            "floor": floor,
            "walls": walls.map(seg), "doors": doors.map(seg), "windows": windows.map(seg), "openings": openings.map(seg),
            "objects": objs,
        ]
    }

    static func convexHull(_ points: [[Double]]) -> [[Double]] {
        let p = points.sorted { $0[0] != $1[0] ? $0[0] < $1[0] : $0[1] < $1[1] }
        if p.count < 3 { return p }
        func cross(_ o: [Double], _ a: [Double], _ b: [Double]) -> Double { (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]) }
        var lower: [[Double]] = [], upper: [[Double]] = []
        for q in p { while lower.count >= 2 && cross(lower[lower.count - 2], lower[lower.count - 1], q) <= 0 { lower.removeLast() }; lower.append(q) }
        for q in p.reversed() { while upper.count >= 2 && cross(upper[upper.count - 2], upper[upper.count - 1], q) <= 0 { upper.removeLast() }; upper.append(q) }
        lower.removeLast(); upper.removeLast()
        return lower + upper
    }

    /// `sensors` layer from the session's esp_nodes.json (RuView CSI nodes placed by the ESP32 placement tools, in the
    /// session world frame), moved into the room frame with `worldToRoom` (row-major 4x4 of room_map.json).
    static func sensors(espNodes: [String: Any], worldToRoom m: [Double]) -> [[String: Any]] {
        guard m.count == 16, let nodes = espNodes["nodes"] as? [[String: Any]] else { return [] }
        var out: [[String: Any]] = []
        for n in nodes {
            guard let id = (n["node_id"] as? NSNumber)?.intValue, let p = n["position"] as? [Any], p.count >= 3 else { continue }
            let w = p.prefix(3).compactMap { ($0 as? NSNumber)?.doubleValue }
            guard w.count == 3 else { continue }
            let pos = (0..<3).map { r -> Double in
                let v = m[r * 4] * w[0] + m[r * 4 + 1] * w[1] + m[r * 4 + 2] * w[2] + m[r * 4 + 3]
                return (v * 1000).rounded() / 1000
            }
            var e: [String: Any] = ["id": "ruview_\(id)", "kind": "ruview", "node_id": id,
                                    "label": n["label"] as? String ?? "esp-\(id)", "position": pos]
            if let meth = n["method"] as? String { e["method"] = meth }
            if let c = (n["confidence"] as? NSNumber)?.doubleValue { e["confidence"] = c }
            out.append(e)
        }
        return out
    }

    /// Adds / refreshes `sensors` in a room map from esp_nodes.json in the session folder, when there is one.
    static func withEspNodes(_ map: [String: Any], session dir: URL) -> [String: Any]? {
        guard let d = try? Data(contentsOf: dir.appendingPathComponent("esp_nodes.json")),
              let esp = (try? JSONSerialization.jsonObject(with: d)) as? [String: Any],
              let m = map["world_to_room"] as? [Any] else { return nil }
        let s = sensors(espNodes: esp, worldToRoom: m.compactMap { ($0 as? NSNumber)?.doubleValue })
        if s.isEmpty { return nil }
        var out = map
        out["sensors"] = s
        return out
    }
}
