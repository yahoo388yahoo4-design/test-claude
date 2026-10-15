import Foundation

// Tests for ios/R2SCapture/Home/RoomMapCore.swift (RoomPlan -> room_map.json floor plan).
var failures = 0
func check(_ cond: Bool, _ msg: String, line: Int = #line) {
    if cond { print("  ok  \(msg)") } else { failures += 1; print("FAIL  \(msg)  (line \(line))") }
}
func near(_ a: Double, _ b: Double, _ tol: Double = 1e-3) -> Bool { abs(a - b) <= tol }

typealias E = RoomMapCore.Element
for rot in [0.0, 0.4, -1.2] {
    print("4 x 3 m room turned \(rot) rad")
    let c = cos(rot), s = sin(rot)
    func w(_ x: Double, _ z: Double) -> (Double, Double) { (c * x - s * z, s * x + c * z) }
    func wall(_ x: Double, _ z: Double, _ ax: Double, _ az: Double, _ len: Double) -> E {
        let p = w(x, z), a = w(ax, az)
        return E(id: "w", category: "wall", confidence: 0.9, cx: p.0, cy: 0.05, cz: p.1, axisX: a.0, axisZ: a.1,
                 width: len, height: 2.5, depth: 0, parent: nil)
    }
    let walls = [wall(0, -1.5, 1, 0, 4), wall(0, 1.5, -1, 0, 4), wall(-2, 0, 0, 1, 3), wall(2, 0, 0, -1, 3)]
    let tvp = w(0, -1.4), tva = w(1, 0)
    let tv = E(id: "u1", category: RoomMapCore.category("television"), confidence: 0.9, cx: tvp.0, cy: 0.0, cz: tvp.1,
               axisX: tva.0, axisZ: tva.1, width: 1.2, height: 0.7, depth: 0.1, parent: nil)
    let m = RoomMapCore.build(name: "Living Room", session: "s", device: "d", walls: walls, doors: [], windows: [], openings: [],
                              objects: [tv], floorCorners: [], floorY: nil, nowUnix: 0)
    let b = m["bounds"] as! [Double]
    let dims = [b[2], b[3]].sorted()
    check(near(dims[0], 3, 0.01) && near(dims[1], 4, 0.01), "bounds \(b)")
    check(m["id"] as? String == "living_room", "slug")
    let o = (m["objects"] as! [[String: Any]])[0]
    check(o["category"] as? String == "tv" && o["id"] as? String == "tv_1", "tv category")
    let ctr = o["center"] as! [Double]
    check(near(ctr[2], 1.2, 0.01), "tv centre 1.2 m above the floor (floor from the wall bottoms)")
    let yaw = o["yaw"] as! Double
    check(near(abs(yaw).truncatingRemainder(dividingBy: Double.pi / 2), 0, 0.01) || near(abs(yaw).truncatingRemainder(dividingBy: Double.pi / 2), Double.pi / 2, 0.01), "tv aligned with the plan axes")
    for wl in m["walls"] as! [[String: Any]] {
        let a = wl["a"] as! [Double], bb = wl["b"] as! [Double]
        check(near(a[0], bb[0], 0.01) || near(a[1], bb[1], 0.01), "wall axis aligned \(a) \(bb)")
    }
    check((m["floor"] as! [[Double]]).count == 4, "floor outline is the 4 corners")
}
do {
    let m: [Double] = [1, 0, 0, 1, 0, 0, -1, 0, 0, 1, 0, 1.2, 0, 0, 0, 1]
    let esp: [String: Any] = ["version": 1, "nodes": [
        ["node_id": 3, "label": "esp-3", "position": [0.5, 0.3, -2.0], "method": "tap_live", "confidence": 0.8],
        ["node_id": 4, "label": "no position"]]]
    let s = RoomMapCore.sensors(espNodes: esp, worldToRoom: m)
    check(s.count == 1 && s[0]["id"] as? String == "ruview_3" && s[0]["node_id"] as? Int == 3, "esp node -> sensor")
    let p = s.first?["position"] as? [Double] ?? []
    check(p.count == 3 && near(p[0], 1.5, 1e-6) && near(p[1], 2.0, 1e-6) && near(p[2], 1.5, 1e-6), "esp node in room frame \(p)")
}
check(RoomMapCore.category("washerDryer") == "washer_dryer" && RoomMapCore.category("sofa") == "sofa", "categories")
print(failures == 0 ? "ALL PASSED" : "\(failures) FAILED")
exit(failures == 0 ? 0 : 1)
