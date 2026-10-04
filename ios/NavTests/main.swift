import Foundation

// Unit + closed-loop simulation tests for ios/R2SCapture/Nav/NavCore.swift.
// Run: ios/NavTests/run_tests.sh   (needs a Swift toolchain; works on Linux and macOS)

var failures = 0
func check(_ cond: Bool, _ msg: String, file: String = #file, line: Int = #line) {
    if cond { print("  ok  \(msg)") } else { failures += 1; print("FAIL  \(msg)  (line \(line))") }
}
func near(_ a: Double, _ b: Double, _ tol: Double) -> Bool { abs(a - b) <= tol }

print("conventions")
do {
    let pose = Pose2(p: P2(0, 0), theta: 0)            // facing +x
    check(near(pose.forward.x, 1, 1e-9) && near(pose.forward.z, 0, 1e-9), "theta 0 faces +x")
    check(near(deg(pose.bearing(to: P2(0, -1))), 90, 1e-6), "-z is on the left when facing +x")
    check(near(deg(pose.bearing(to: P2(0, 1))), -90, 1e-6), "+z is on the right")
    let north = Pose2(p: P2(0, 0), theta: rad(90))     // facing -z (the default ARKit camera forward)
    check(near(north.forward.z, -1, 1e-9), "theta 90 faces -z")
    check(near(deg(north.bearing(to: P2(-1, 0))), 90, 1e-6), "facing -z, -x is on the left")
    check(near(deg(Pose2.heading(from: P2(0, 0), to: P2(0, -2))), 90, 1e-9), "heading to -z is 90")
    check(near(deg(wrapAngle(rad(350))), -10, 1e-9), "wrapAngle")
}

print("occupancy grid")
let grid = OccupancyGrid(cell: 0.05, sizeMeters: 10)
do {
    let sensor = P2(0, 0)
    let wall = stride(from: -1.0, through: 1.0, by: 0.02).map { P2(2.0, $0) }
    let floor = stride(from: 0.2, through: 1.8, by: 0.1).map { P2($0, 0.3) }
    grid.integrate(sensor: sensor, hits: wall, floor: floor)
    check(!grid.isOccupied(P2(2.0, 0)), "one sweep is not enough to mark occupied (noise filter)")
    grid.integrate(sensor: sensor, hits: wall, floor: floor)
    check(grid.isOccupied(P2(2.0, 0)), "two sweeps mark the wall occupied")
    check(!grid.isOccupied(P2(1.0, 0)), "cells before the wall stay free")
    let (i, j) = grid.cellOf(P2(1.0, 0))!
    check(grid.logOdds[j * grid.size + i] < OccupancyGrid.freeBelow, "ray-traced cells become free")
    let r = grid.raycast(from: sensor, angle: 0, maxRange: 4)
    check(near(r, 2.0, 0.08), "virtual lidar hits the wall at 2 m (got \(r))")
    let scan = grid.scan(pose: Pose2(p: sensor, theta: 0), count: 72, maxRange: 4)
    check(near(scan[0], 2.0, 0.08) && scan[36] == 4, "scan: wall ahead, nothing behind")
    let d = grid.distanceField(cap: 1)
    let (ci, cj) = grid.cellOf(P2(1.5, 0))!
    check(near(Double(d[cj * grid.size + ci]), 0.5, 0.06), "distance field ~0.5 m from wall")
}

print("ROS map export / import")
do {
    let pgm = grid.pgm()
    let other = OccupancyGrid(cell: 0.05, sizeMeters: 10)
    check(other.loadStatic(pgm: pgm), "PGM loads back")
    check(other.isOccupied(P2(2.0, 0)), "occupied cell survives the round trip (static layer)")
    check(grid.rosYAML(imageName: "map.pgm").contains("resolution: 0.05"), "map.yaml")
}

print("planner")
do {
    let g = OccupancyGrid(cell: 0.05, sizeMeters: 10)
    // A 2 m wall across x = 1.5, from z = -1 to z = 1 (static, as if loaded from RoomPlan).
    for z in stride(from: -1.0, through: 1.0, by: 0.025) { g.markStatic(P2(1.5, z)) }
    let cfg = PlannerConfig(robotRadius: 0.2, margin: 0.2)
    switch Planner.plan(grid: g, start: P2(0, 0), goal: P2(3, 0), cfg: cfg) {
    case .ok(let path):
        check(path.first == P2(0, 0) && path.last == P2(3, 0), "path runs start -> goal")
        let field = g.distanceField(cap: 1)
        var minClear = 9.0
        for k in 0..<(path.count - 1) {
            let a = path[k], b = path[k + 1]
            for s in 0...50 {
                let q = a + (b - a) * (Double(s) / 50)
                if let (i, j) = g.cellOf(q) { minClear = min(minClear, Double(field[j * g.size + i])) }
            }
        }
        check(minClear >= 0.19, "path keeps the robot radius from the wall (min \(String(format: "%.2f", minClear)) m)")
        check(path.count <= 6, "path is smoothed (\(path.count) vertices)")
    default:
        check(false, "planner found a path around the wall")
    }
    // Goal inside the wall: planner picks the nearest free cell.
    if case .ok(let p) = Planner.plan(grid: g, start: P2(0, 0), goal: P2(1.5, 0), cfg: cfg) {
        check(p.last!.distance(to: P2(1.5, 0)) < 0.35, "blocked goal snaps to a nearby free cell")
    } else {
        check(false, "blocked goal handled")
    }
    // Fully enclosed goal.
    let box = OccupancyGrid(cell: 0.05, sizeMeters: 10)
    for a in stride(from: 0.0, to: 2 * Double.pi, by: 0.005) { box.markStatic(P2(3 + 1.5 * cos(a), 1.5 * sin(a))) }
    if case .noPath = Planner.plan(grid: box, start: P2(0, 0), goal: P2(3, 0), cfg: cfg) {
        check(true, "enclosed goal reports no path")
    } else {
        check(false, "enclosed goal reports no path")
    }
}

print("follower")
do {
    let cfg = FollowerConfig()
    let path = [P2(0, 0), P2(2, 0)]
    let out = Follower.step(path: path, pose: Pose2(p: P2(0, 0), theta: 0), frontGap: 5, cfg: cfg)
    check(out.state == .driving && out.cmd.v > 0.2 && abs(out.cmd.w) < 0.01, "straight path drives forward")
    let back = Follower.step(path: path, pose: Pose2(p: P2(0, 0), theta: Double.pi), frontGap: 5, cfg: cfg)
    check(back.state == .rotating && back.cmd.v == 0, "target behind: rotate in place")
    let left = Follower.step(path: [P2(0, 0), P2(1, -1)], pose: Pose2(p: P2(0, 0), theta: rad(20)), frontGap: 5, cfg: cfg)
    check(left.cmd.w > 0, "target to the left turns left (w > 0)")
    let blocked = Follower.step(path: path, pose: Pose2(p: P2(0, 0), theta: 0), frontGap: 0.2, cfg: cfg)
    check(blocked.state == .blocked && blocked.cmd.v == 0, "obstacle within stop distance blocks")
    let slow = Follower.step(path: path, pose: Pose2(p: P2(0, 0), theta: 0), frontGap: 0.45, cfg: cfg)
    check(slow.cmd.v < out.cmd.v && slow.cmd.v > 0, "obstacle within slow distance slows down")
    let done = Follower.step(path: path, pose: Pose2(p: P2(1.95, 0), theta: 0), frontGap: 5, cfg: cfg)
    check(done.state == .arrived, "within tolerance: arrived")
    check(Follower.safety(DriveCommand(v: 0.3, w: 0), frontGap: 0.1, cfg: cfg).v == 0, "safety stops forward motion")
    check(Follower.safety(DriveCommand(v: -0.2, w: 0), frontGap: 0.1, cfg: cfg).v == -0.2, "safety allows reversing")
}

print("guidance")
do {
    let cfg = FollowerConfig()
    let path = [P2(0, 0), P2(2, 0), P2(2, -2)]
    let a = Guidance.instruction(path: path, pose: Pose2(p: P2(0, 0), theta: 0), frontGap: 5, cfg: cfg)
    check(a.text == "Go straight 2 meters, then turn left", "straight then left: \(a.text)")
    let b = Guidance.instruction(path: path, pose: Pose2(p: P2(0, 0), theta: rad(-90)), frontGap: 5, cfg: cfg)
    check(b.text == "Turn left 90 degrees", "turn: \(b.text)")
    let c = Guidance.instruction(path: path, pose: Pose2(p: P2(0, 0), theta: 0), frontGap: 0.25, cfg: cfg)
    check(c.text == "Obstacle ahead, 25 centimeters", "obstacle: \(c.text)")
}

print("protocol")
do {
    let s = RobotProtocol.encode(RobotProtocol.vel(DriveCommand(v: 0.12345, w: -0.5), seq: 7))!
    check(s == #"{"seq":7,"type":"vel","v":0.123,"w":-0.5}"#, "vel JSON: \(s)")
    let m = RobotProtocol.decode(RobotProtocol.encode(RobotProtocol.move(cm: 50, speedCmS: 20, seq: 8))!)!
    check(m["type"] as? String == "move" && (m["dist_cm"] as? Double) == 50, "move round trip")
    check(RobotProtocol.encode(["type": "vel", "v": Double.nan]) == nil, "NaN is rejected, not crashed on")
}

print("map prior (RoomPlan objects.json + mesh.ply)")
do {
    // RoomPlan: floor at y = -1.2, a 4 m wall along z = -2 with a door in the middle, a table at (1, 0).
    func T(_ x: Double, _ y: Double, _ z: Double, yaw: Double = 0) -> [Double] {
        [cos(yaw), 0, sin(yaw), x, 0, 1, 0, y, -sin(yaw), 0, cos(yaw), z, 0, 0, 0, 1]
    }
    let room: [String: Any] = [
        "floors": [["T": T(0, -1.2, 0), "dims": [6, 0, 6]]],
        "walls": [["id": "w1", "T": T(0, 0, -2), "dims": [4, 2.4, 0]]],
        "doors": [["id": "d1", "T": T(0, -0.2, -2), "dims": [0.9, 2.0, 0], "parent": "w1"]],
        "objects": [["id": "t1", "category": "table", "T": T(1, -0.8, 0, yaw: 0.3), "dims": [1.2, 0.75, 0.8]],
                    ["id": "c1", "category": "storage", "T": T(-1, 0.6, 0), "dims": [1, 0.5, 0.4]]],
    ]
    let g = OccupancyGrid(cell: 0.05, sizeMeters: 12)
    let r = MapPrior.loadRoomPlan(json: try! JSONSerialization.data(withJSONObject: room), into: g, robotHeight: 0.6)
    check(r.floorY == -1.2, "floor height from RoomPlan floors")
    check(g.isOccupied(P2(1.5, -2)) && g.isOccupied(P2(-1.5, -2)), "wall rasterised")
    check(g.isOccupied(P2(1, 0)), "table footprint rasterised")
    check(!g.isOccupied(P2(-1, 0)), "wall cabinet above robot height ignored")
    check(r.items.contains { $0.label == "table" }, "table listed for the go-to menu")
    for item in r.items where item.kind == "door" { MapPrior.fill(item.footprint, grid: g, inflate: 0.1) { g.clearStatic($0) } }
    check(!g.isOccupied(P2(0, -2)), "door opening cleared in the wall")
    let table = r.items.first { $0.label == "table" }!
    let ap = table.approachPoint(from: P2(1, 2), clearance: 0.3)
    check(ap.z > 0.9 && abs(ap.x - 1) < 0.01, "approach point is on the robot's side of the table")
    if case .ok(let path) = Planner.plan(grid: g, start: P2(0, 1.5), goal: P2(0, -3), cfg: PlannerConfig(robotRadius: 0.2, margin: 0.15)) {
        var crossX = 99.0
        for k in 0..<(path.count - 1) where (path[k].z + 2) * (path[k + 1].z + 2) <= 0 {
            let a = path[k], b = path[k + 1]
            crossX = a.x + (b.x - a.x) * ((-2 - a.z) / (b.z - a.z))
        }
        check(abs(crossX) < 0.45, "plan goes through the door (crosses the wall at x = \(String(format: "%.2f", crossX)))")
    } else {
        check(false, "plan through the door")
    }

    // mesh.ply: floor triangle (cls 2) at y = 0 and a box face (cls 4) 0.3 m high at x = 2.
    var ply = Data("ply\nformat binary_little_endian 1.0\nelement vertex 6\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\nelement face 2\nproperty list uchar int vertex_indices\nproperty uchar classification\nend_header\n".utf8)
    let verts: [[Float]] = [[0, 0, 0], [1, 0, 0], [0, 0, 1], [2, 0.3, 0], [2, 0.3, 0.5], [2, 0.1, 0.25]]
    for v in verts { for f in v + [0, 1, 0] { var x = f; ply.append(Data(bytes: &x, count: 4)) } }
    func face(_ a: Int32, _ b: Int32, _ c: Int32, _ cls: UInt8) {
        var n: UInt8 = 3; ply.append(Data(bytes: &n, count: 1))
        for i in [a, b, c] { var x = i; ply.append(Data(bytes: &x, count: 4)) }
        var k = cls; ply.append(Data(bytes: &k, count: 1))
    }
    face(0, 1, 2, 2)
    face(3, 4, 5, 4)
    let g2 = OccupancyGrid(cell: 0.05, sizeMeters: 12)
    let m = MapPrior.loadMesh(ply: ply, into: g2, floorY: 0, robotHeight: 0.6)
    check(m.vertices == 6 && m.cells == 3, "mesh: 3 obstacle vertices -> \(m.cells) cells")
    check(g2.isOccupied(P2(2, 0.5)), "mesh obstacle marked")
    check(!g2.isOccupied(P2(1, 0)), "mesh floor not an obstacle")
}

print("closed-loop simulation (differential drive, simulated depth sensor, replanning)")
do {
    // Ground-truth world: room walls 6 x 4 m, a table-sized box in the middle, unknown to the robot.
    func truthOccupied(_ p: P2) -> Bool {
        if p.x < -0.5 || p.x > 5.5 || p.z < -2 || p.z > 2 { return true }
        if p.x > 2.0 && p.x < 2.8 && p.z > -0.9 && p.z < 0.6 { return true }
        return false
    }
    let g = OccupancyGrid(cell: 0.05, sizeMeters: 16)
    var pose = Pose2(p: P2(0, 0), theta: 0)
    let goal = P2(4.8, 0.2)
    let fcfg = FollowerConfig()
    let pcfg = PlannerConfig(robotRadius: 0.2, margin: 0.2)
    var path: [P2] = []
    var arrived = false
    var minTruthClear = 9.0
    let dt = 0.1
    var t = 0.0
    while t < 90 {
        // Simulated forward depth sensor: 60 degree FOV, 4 m range.
        var hits: [P2] = [], floor: [P2] = []
        var frontGap = 9.0
        for k in 0..<61 {
            let a = pose.theta + rad(Double(k - 30))
            var r = 0.05
            var hit = false
            while r < 4 {
                let q = pose.p + P2(cos(a), -sin(a)) * r
                if truthOccupied(q) { hits.append(q); hit = true; break }
                if Int(r * 100) % 20 == 0 { floor.append(q) }
                r += 0.02
            }
            if hit {
                let l = pose.local(hits[hits.count - 1])
                if l.fwd > 0 && abs(l.left) < 0.25 { frontGap = min(frontGap, l.fwd - 0.2) }
            }
        }
        g.integrate(sensor: pose.p, hits: hits, floor: floor)
        if Int((t / dt).rounded()) % 5 == 0 {   // replan at 2 Hz
            if case .ok(let p) = Planner.plan(grid: g, start: pose.p, goal: goal, cfg: pcfg) { path = p }
        }
        let out = Follower.step(path: path, pose: pose, frontGap: frontGap, cfg: fcfg)
        if out.state == .arrived { arrived = true; break }
        pose.theta = wrapAngle(pose.theta + out.cmd.w * dt)
        pose.p = pose.p + pose.forward * (out.cmd.v * dt)
        // Ground-truth clearance from the robot centre.
        var c = 9.0
        for k in 0..<36 {
            let a = Double(k) * Double.pi / 18
            var r = 0.0
            while r < 1 { if truthOccupied(pose.p + P2(cos(a), -sin(a)) * r) { break }; r += 0.01 }
            c = min(c, r)
        }
        minTruthClear = min(minTruthClear, c)
        t += dt
    }
    check(arrived, "robot reaches the goal around an unseen obstacle (t = \(String(format: "%.1f", t)) s)")
    check(minTruthClear > 0.2, "robot body never touched an obstacle (min centre clearance \(String(format: "%.2f", minTruthClear)) m)")
}

print(failures == 0 ? "\nALL TESTS PASSED" : "\n\(failures) FAILURE(S)")
exit(failures == 0 ? 0 : 1)
