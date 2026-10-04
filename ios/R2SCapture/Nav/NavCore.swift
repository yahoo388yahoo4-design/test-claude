import Foundation

// Platform-independent navigation core: 2D occupancy grid, A* planner, path follower, spoken-guidance
// text and the robot wire protocol. No ARKit / UIKit here, so it also compiles and is unit-tested on
// Linux (ios/NavTests/run_tests.sh).
//
// Conventions (see NAVIGATION.md):
// * The floor plane is ARKit's world x / z (y is up, gravity-aligned). A floor point is P2(x, z).
// * Heading `theta` is measured counter-clockwise seen from above, from +x towards -z, so the robot's
//   forward unit vector is (cos theta, -sin theta) and its left unit vector is (-sin theta, -cos theta).
// * Angular velocity w > 0 turns left (ROS convention). Linear velocity v > 0 drives forward.

struct P2: Equatable {
    var x: Double
    var z: Double
    init(_ x: Double, _ z: Double) { self.x = x; self.z = z }
    static func + (a: P2, b: P2) -> P2 { P2(a.x + b.x, a.z + b.z) }
    static func - (a: P2, b: P2) -> P2 { P2(a.x - b.x, a.z - b.z) }
    static func * (a: P2, s: Double) -> P2 { P2(a.x * s, a.z * s) }
    var length: Double { (x * x + z * z).squareRoot() }
    func distance(to o: P2) -> Double { (self - o).length }
}

struct Pose2: Equatable {
    var p: P2
    var theta: Double
    var forward: P2 { P2(cos(theta), -sin(theta)) }
    /// Expresses a world point in the robot frame: (forward, left) metres.
    func local(_ q: P2) -> (fwd: Double, left: Double) {
        let d = q - p
        return (d.x * cos(theta) - d.z * sin(theta), -d.x * sin(theta) - d.z * cos(theta))
    }
    /// Bearing of a world point relative to the robot's forward axis, radians, positive = left.
    func bearing(to q: P2) -> Double {
        let l = local(q)
        return atan2(l.left, l.fwd)
    }
    /// World heading of the direction from p to q.
    static func heading(from a: P2, to b: P2) -> Double { atan2(-(b.z - a.z), b.x - a.x) }
}

@inline(__always) func wrapAngle(_ a: Double) -> Double {
    var x = a.truncatingRemainder(dividingBy: 2 * Double.pi)
    if x > Double.pi { x -= 2 * Double.pi }
    if x < -Double.pi { x += 2 * Double.pi }
    return x
}

@inline(__always) func deg(_ r: Double) -> Double { r * 180 / Double.pi }
@inline(__always) func rad(_ d: Double) -> Double { d * Double.pi / 180 }

// MARK: - occupancy grid

/// Log-odds occupancy grid over a fixed square of floor, plus a "static" layer loaded from a saved map
/// (mesh.ply, RoomPlan objects.json or a previous map.pgm). Fresh free-space observations override the
/// static layer, so furniture that has moved since the map was made does not block the robot forever.
final class OccupancyGrid {
    let cell: Double
    let size: Int
    let origin: P2              // world position of the corner of cell (0, 0)
    private(set) var logOdds: [Float]
    private(set) var staticOcc: [UInt8]
    private(set) var version = 0

    static let hit: Float = 0.85
    static let miss: Float = -0.4
    static let clampHi: Float = 4
    static let clampLo: Float = -4
    static let occupiedAbove: Float = 1.2   // two consistent hits
    static let freeBelow: Float = -0.6
    static let staticOverride: Float = -2.0 // this free evidence clears a static cell

    init(cell: Double = 0.05, sizeMeters: Double = 30, center: P2 = P2(0, 0)) {
        self.cell = cell
        size = max(16, Int((sizeMeters / cell).rounded()))
        origin = P2(center.x - Double(size) * cell / 2, center.z - Double(size) * cell / 2)
        logOdds = [Float](repeating: 0, count: size * size)
        staticOcc = [UInt8](repeating: 0, count: size * size)
    }

    @inline(__always) func cellOf(_ p: P2) -> (Int, Int)? {
        let i = Int(floor((p.x - origin.x) / cell))
        let j = Int(floor((p.z - origin.z) / cell))
        guard i >= 0, j >= 0, i < size, j < size else { return nil }
        return (i, j)
    }

    @inline(__always) func centerOf(_ i: Int, _ j: Int) -> P2 {
        P2(origin.x + (Double(i) + 0.5) * cell, origin.z + (Double(j) + 0.5) * cell)
    }

    @inline(__always) func occupied(_ k: Int) -> Bool {
        let l = logOdds[k]
        return l > OccupancyGrid.occupiedAbove || (staticOcc[k] != 0 && l > OccupancyGrid.staticOverride)
    }

    @inline(__always) func known(_ k: Int) -> Bool { logOdds[k] != 0 || staticOcc[k] != 0 }

    func isOccupied(_ p: P2) -> Bool {
        guard let (i, j) = cellOf(p) else { return false }
        return occupied(j * size + i)
    }

    func reset() {
        for k in 0..<logOdds.count { logOdds[k] = 0 }
        version += 1
    }

    func markStatic(_ p: P2) {
        guard let (i, j) = cellOf(p) else { return }
        staticOcc[j * size + i] = 1
    }

    func clearStatic(_ p: P2) {
        guard let (i, j) = cellOf(p) else { return }
        staticOcc[j * size + i] = 0
    }

    func clearAllStatic() {
        for k in 0..<staticOcc.count { staticOcc[k] = 0 }
        version += 1
    }

    /// Marks free evidence at a point without ray tracing (used for floor seen in a saved mesh).
    func markFree(_ p: P2, amount: Float = OccupancyGrid.miss) {
        guard let (i, j) = cellOf(p) else { return }
        let k = j * size + i
        logOdds[k] = max(OccupancyGrid.clampLo, logOdds[k] + amount)
    }

    /// Integrates one depth sweep. `hits` are obstacle points (in the robot's height band), `floor` are
    /// points seen on the floor. Cells between the sensor and every point become freer; hit cells more
    /// occupied. Each cell is updated at most once per call.
    func integrate(sensor: P2, hits: [P2], floor: [P2], maxRange: Double = 5) {
        guard let (si, sj) = cellOf(sensor) else { return }
        var delta = [Int: Float]()
        delta.reserveCapacity(4096)
        var hitCells = Set<Int>()
        var freeEnds = Set<Int>()
        for p in hits {
            guard p.distance(to: sensor) <= maxRange, let (i, j) = cellOf(p) else { continue }
            hitCells.insert(j * size + i)
        }
        for p in floor {
            guard p.distance(to: sensor) <= maxRange, let (i, j) = cellOf(p) else { continue }
            let k = j * size + i
            if !hitCells.contains(k) { freeEnds.insert(k) }
        }
        let trace: (Int, Bool) -> Void = { k, includeEnd in
            let ei = k % self.size, ej = k / self.size
            self.bresenham(si, sj, ei, ej) { ci, cj, isEnd in
                if isEnd && !includeEnd { return }
                let c = cj * self.size + ci
                if hitCells.contains(c) { return }
                delta[c] = OccupancyGrid.miss
            }
        }
        for k in hitCells { trace(k, false) }
        for k in freeEnds { trace(k, true) }
        for k in hitCells { delta[k] = OccupancyGrid.hit }
        for (k, d) in delta {
            logOdds[k] = min(OccupancyGrid.clampHi, max(OccupancyGrid.clampLo, logOdds[k] + d))
        }
        version += 1
    }

    private func bresenham(_ x0: Int, _ y0: Int, _ x1: Int, _ y1: Int, _ visit: (Int, Int, Bool) -> Void) {
        var x = x0, y = y0
        let dx = abs(x1 - x0), dy = -abs(y1 - y0)
        let sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1
        var err = dx + dy
        while true {
            let end = x == x1 && y == y1
            visit(x, y, end)
            if end { break }
            let e2 = 2 * err
            if e2 >= dy { err += dy; x += sx }
            if e2 <= dx { err += dx; y += sy }
        }
    }

    /// Virtual 2D lidar: distance from `from` along world heading `angle` to the first occupied cell,
    /// or `maxRange` when none is hit.
    func raycast(from: P2, angle: Double, maxRange: Double) -> Double {
        let step = cell * 0.5
        let dir = P2(cos(angle), -sin(angle))
        var r = step
        while r < maxRange {
            let q = from + dir * r
            guard let (i, j) = cellOf(q) else { return maxRange }
            if occupied(j * size + i) { return r }
            r += step
        }
        return maxRange
    }

    /// 360 degree virtual scan around a pose, `count` rays starting at the robot's forward axis and going
    /// counter-clockwise (left). Index k is at bearing k * 360 / count degrees.
    func scan(pose: Pose2, count: Int = 72, maxRange: Double = 4) -> [Double] {
        (0..<count).map { k in raycast(from: pose.p, angle: pose.theta + 2 * Double.pi * Double(k) / Double(count), maxRange: maxRange) }
    }

    /// Distance (metres) from each cell to the nearest occupied cell, capped at `cap`. Two-pass chamfer
    /// transform, O(cells).
    func distanceField(cap: Double) -> [Float] {
        let n = size
        let big = Float(cap)
        var d = [Float](repeating: big, count: n * n)
        for k in 0..<(n * n) where occupied(k) { d[k] = 0 }
        let a = Float(cell), b = Float(cell * 2.0.squareRoot())
        for j in 0..<n {
            for i in 0..<n {
                let k = j * n + i
                var v = d[k]
                if v == 0 { continue }
                if i > 0 { v = min(v, d[k - 1] + a) }
                if j > 0 {
                    v = min(v, d[k - n] + a)
                    if i > 0 { v = min(v, d[k - n - 1] + b) }
                    if i < n - 1 { v = min(v, d[k - n + 1] + b) }
                }
                d[k] = v
            }
        }
        for j in stride(from: n - 1, through: 0, by: -1) {
            for i in stride(from: n - 1, through: 0, by: -1) {
                let k = j * n + i
                var v = d[k]
                if v == 0 { continue }
                if i < n - 1 { v = min(v, d[k + 1] + a) }
                if j < n - 1 {
                    v = min(v, d[k + n] + a)
                    if i < n - 1 { v = min(v, d[k + n + 1] + b) }
                    if i > 0 { v = min(v, d[k + n - 1] + b) }
                }
                d[k] = min(v, big)
            }
        }
        return d
    }

    // MARK: ROS map_server (PGM) export / import

    /// 8-bit PGM in ROS map_server convention: 254 free, 0 occupied, 205 unknown. Row 0 is the top of
    /// the image, which is the most negative z (the "far" side when +x is right and -z is up).
    func pgm() -> Data {
        var out = Data("P5\n\(size) \(size)\n255\n".utf8)
        out.reserveCapacity(out.count + size * size)
        for j in 0..<size {
            for i in 0..<size {
                let k = j * size + i
                let v: UInt8 = occupied(k) ? 0 : (logOdds[k] < OccupancyGrid.freeBelow ? 254 : 205)
                out.append(v)
            }
        }
        return out
    }

    /// map.yaml for ROS map_server. ROS maps are x right / y up; here image rows run along +z, so ROS
    /// y = -z. The origin is the world position of the image's bottom-left pixel.
    func rosYAML(imageName: String) -> String {
        """
        image: \(imageName)
        resolution: \(cell)
        origin: [\(origin.x), \(-(origin.z + Double(size) * cell)), 0.0]
        negate: 0
        occupied_thresh: 0.65
        free_thresh: 0.196
        # r2s: ROS x = ARKit x, ROS y = -ARKit z; ARKit world frame of the saved worldmap.arworldmap
        """
    }

    /// Loads a PGM written by `pgm()` (same size, cell and origin) into the static layer.
    @discardableResult
    func loadStatic(pgm data: Data) -> Bool {
        let bytes = [UInt8](data)
        var fields: [String] = []
        var idx = 0
        while fields.count < 4 && idx < bytes.count {
            while idx < bytes.count, bytes[idx] == 0x20 || bytes[idx] == 0x0A || bytes[idx] == 0x0D || bytes[idx] == 0x09 { idx += 1 }
            if idx < bytes.count, bytes[idx] == 0x23 { // comment
                while idx < bytes.count, bytes[idx] != 0x0A { idx += 1 }
                continue
            }
            var s = ""
            while idx < bytes.count, !(bytes[idx] == 0x20 || bytes[idx] == 0x0A || bytes[idx] == 0x0D || bytes[idx] == 0x09) {
                s.append(Character(UnicodeScalar(bytes[idx])))
                idx += 1
            }
            fields.append(s)
        }
        idx += 1
        guard fields.count == 4, fields[0] == "P5", let w = Int(fields[1]), let h = Int(fields[2]),
              w == size, h == size, bytes.count - idx >= w * h else { return false }
        for k in 0..<(w * h) {
            let v = bytes[idx + k]
            if v < 100 { staticOcc[k] = 1 } else if v > 250 { logOdds[k] = min(logOdds[k], -1) }
        }
        version += 1
        return true
    }
}

// MARK: - A* planner

struct PlannerConfig {
    var robotRadius = 0.20
    var margin = 0.25            // soft cost band outside the robot radius
    var unknownCost = 0.3        // extra cost per metre through unobserved cells
    var maxExpansions = 400_000
}

enum PlanResult {
    case ok([P2])
    case startBlocked
    case goalBlocked
    case noPath
}

enum Planner {
    /// Plans on the grid from `start` to `goal`. Cells closer than robotRadius to an obstacle are
    /// forbidden (except right around the start, so a robot that ended up near a wall can back out);
    /// cells inside the margin band cost more. If the goal itself is blocked, the nearest reachable free
    /// cell within 1 m is used. Returns a smoothed polyline that starts at `start`.
    static func plan(grid: OccupancyGrid, start: P2, goal: P2, cfg: PlannerConfig, field: [Float]? = nil) -> PlanResult {
        let n = grid.size
        let dist = field ?? grid.distanceField(cap: cfg.robotRadius + cfg.margin + grid.cell)
        guard let (si, sj) = grid.cellOf(start) else { return .startBlocked }
        guard var (gi, gj) = grid.cellOf(goal) else { return .goalBlocked }
        let r = Float(cfg.robotRadius)
        let s = sj * n + si
        func lethal(_ k: Int) -> Bool { dist[k] < r }
        var g = gj * n + gi
        let substituted = lethal(g)
        if substituted {
            guard let alt = nearestFree(grid: grid, dist: dist, from: g, minDist: r, maxCells: Int(1.0 / grid.cell)) else { return .goalBlocked }
            g = alt
            gi = g % n
            gj = g / n
        }
        let escape = Double(cfg.robotRadius + 0.1)
        var gScore = [Float](repeating: .infinity, count: n * n)
        var came = [Int32](repeating: -1, count: n * n)
        var closed = [Bool](repeating: false, count: n * n)
        var heap = MinHeap()
        gScore[s] = 0
        heap.push(Float(heuristic(si, sj, gi, gj) * grid.cell), s)
        let nbr: [(Int, Int, Float)] = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1),
                                        (1, 1, 1.41421356), (1, -1, 1.41421356), (-1, 1, 1.41421356), (-1, -1, 1.41421356)]
        let cellF = Float(grid.cell)
        let margin = Float(cfg.margin)
        var expansions = 0
        var found = false
        while let (_, k) = heap.pop() {
            if closed[k] { continue }
            if k == g { found = true; break }
            closed[k] = true
            expansions += 1
            if expansions > cfg.maxExpansions { break }
            let ci = k % n, cj = k / n
            for (di, dj, w) in nbr {
                let ni = ci + di, nj = cj + dj
                guard ni >= 0, nj >= 0, ni < n, nj < n else { continue }
                let m = nj * n + ni
                if closed[m] { continue }
                var cost = w * cellF
                if lethal(m) {
                    // Only allowed right next to the start (escape from a tight spot), and expensive.
                    if grid.centerOf(ni, nj).distance(to: start) > escape { continue }
                    cost *= 20
                } else if dist[m] < r + margin {
                    let t = (r + margin - dist[m]) / margin
                    cost *= 1 + 4 * t * t
                }
                if !grid.known(m) { cost *= Float(1 + cfg.unknownCost) }
                let tentative = gScore[k] + cost
                if tentative < gScore[m] {
                    gScore[m] = tentative
                    came[m] = Int32(k)
                    heap.push(tentative + Float(heuristic(ni, nj, gi, gj) * grid.cell), m)
                }
            }
        }
        guard found else { return .noPath }
        var cells: [Int] = [g]
        var cur = g
        while cur != s {
            let p = Int(came[cur])
            if p < 0 { return .noPath }
            cells.append(p)
            cur = p
        }
        cells.reverse()
        let pts = cells.map { grid.centerOf($0 % n, $0 / n) }
        // Shortcuts keep half the soft margin, so the smoothed path does not hug corners.
        var out = smooth(pts, grid: grid, dist: dist, clearance: r + margin * 0.5)
        out[0] = start
        let end = substituted ? grid.centerOf(gi, gj) : goal
        if out.count == 1 { out.append(end) } else { out[out.count - 1] = end }
        return .ok(out)
    }

    private static func heuristic(_ i: Int, _ j: Int, _ gi: Int, _ gj: Int) -> Double {
        let dx = Double(abs(i - gi)), dy = Double(abs(j - gj))
        return (dx + dy) + (2.0.squareRoot() - 2) * min(dx, dy)
    }

    private static func nearestFree(grid: OccupancyGrid, dist: [Float], from: Int, minDist: Float, maxCells: Int) -> Int? {
        let n = grid.size
        let fi = from % n, fj = from / n
        var best: Int?
        var bestD = Int.max
        for dj in -maxCells...maxCells {
            for di in -maxCells...maxCells {
                let i = fi + di, j = fj + dj
                guard i >= 0, j >= 0, i < n, j < n else { continue }
                let k = j * n + i
                let d2 = di * di + dj * dj
                if d2 < bestD && dist[k] >= minDist && d2 <= maxCells * maxCells {
                    best = k
                    bestD = d2
                }
            }
        }
        return best
    }

    /// Greedy line-of-sight shortcutting: keeps a vertex only when the straight segment to the next kept
    /// vertex would pass closer than `clearance` to an obstacle.
    static func smooth(_ pts: [P2], grid: OccupancyGrid, dist: [Float], clearance: Float) -> [P2] {
        guard pts.count > 2 else { return pts }
        var out = [pts[0]]
        var i = 0
        while i < pts.count - 1 {
            var j = pts.count - 1
            while j > i + 1 && !lineClear(pts[i], pts[j], grid: grid, dist: dist, clearance: clearance) { j -= 1 }
            out.append(pts[j])
            i = j
        }
        return out
    }

    static func lineClear(_ a: P2, _ b: P2, grid: OccupancyGrid, dist: [Float], clearance: Float) -> Bool {
        let len = a.distance(to: b)
        let steps = max(1, Int(len / (grid.cell * 0.5)))
        for s in 0...steps {
            let q = a + (b - a) * (Double(s) / Double(steps))
            guard let (i, j) = grid.cellOf(q) else { return false }
            if dist[j * grid.size + i] < clearance { return false }
        }
        return true
    }
}

/// Binary min-heap of (priority, cell index).
struct MinHeap {
    private var a: [(Float, Int)] = []
    var isEmpty: Bool { a.isEmpty }
    mutating func push(_ p: Float, _ v: Int) {
        a.append((p, v))
        var c = a.count - 1
        while c > 0 {
            let par = (c - 1) / 2
            if a[par].0 <= a[c].0 { break }
            a.swapAt(par, c)
            c = par
        }
    }
    mutating func pop() -> (Float, Int)? {
        guard !a.isEmpty else { return nil }
        let top = a[0]
        let last = a.removeLast()
        if !a.isEmpty {
            a[0] = last
            var c = 0
            while true {
                let l = 2 * c + 1, r = l + 1
                var m = c
                if l < a.count && a[l].0 < a[m].0 { m = l }
                if r < a.count && a[r].0 < a[m].0 { m = r }
                if m == c { break }
                a.swapAt(m, c)
                c = m
            }
        }
        return top
    }
}

// MARK: - path follower (pure pursuit + rotate in place + obstacle slow-down)

struct DriveCommand: Equatable {
    var v: Double   // m/s, + forward
    var w: Double   // rad/s, + left
    static let zero = DriveCommand(v: 0, w: 0)
}

struct FollowerConfig {
    var maxSpeed = 0.30
    var maxTurnRate = rad(60)
    var lookahead = 0.40
    var goalTolerance = 0.15
    var rotateInPlaceAbove = rad(35)
    var slowDistance = 0.60      // forward gap (from robot edge) where speed starts to drop
    var stopDistance = 0.30      // forward gap where the robot stops
    var turnGain = 1.8
}

enum FollowState: String {
    case driving = "Driving"
    case rotating = "Rotating"
    case blocked = "Blocked"
    case arrived = "Arrived"
}

struct FollowOutput {
    var cmd: DriveCommand
    var state: FollowState
    var target: P2
    var headingError: Double     // radians, + = target is to the left
    var remaining: Double        // metres along the path
}

enum Follower {
    /// Remaining path length from the closest point on the path to the end.
    static func remaining(path: [P2], from p: P2) -> (length: Double, segment: Int) {
        guard path.count >= 2 else { return (path.first.map { $0.distance(to: p) } ?? 0, 0) }
        var bestSeg = 0
        var bestD = Double.infinity
        var bestT = 0.0
        for s in 0..<(path.count - 1) {
            let a = path[s], b = path[s + 1]
            let ab = b - a
            let L2 = ab.x * ab.x + ab.z * ab.z
            let t = L2 > 0 ? max(0, min(1, ((p - a).x * ab.x + (p - a).z * ab.z) / L2)) : 0
            let d = (a + ab * t).distance(to: p)
            if d < bestD { bestD = d; bestSeg = s; bestT = t }
        }
        var len = path[bestSeg].distance(to: path[bestSeg + 1]) * (1 - bestT)
        for s in (bestSeg + 1)..<(path.count - 1) { len += path[s].distance(to: path[s + 1]) }
        return (len + bestD, bestSeg)
    }

    /// The point `lookahead` metres ahead along the path from the closest point to `p`.
    static func lookaheadPoint(path: [P2], from p: P2, lookahead: Double) -> P2 {
        guard path.count >= 2 else { return path.first ?? p }
        let seg = remaining(path: path, from: p).segment
        var acc = 0.0
        var cur = p
        for s in seg..<(path.count - 1) {
            let b = path[s + 1]
            let d = cur.distance(to: b)
            if acc + d >= lookahead {
                let t = (lookahead - acc) / max(d, 1e-6)
                return cur + (b - cur) * t
            }
            acc += d
            cur = b
        }
        return path[path.count - 1]
    }

    static func step(path: [P2], pose: Pose2, frontGap: Double, cfg: FollowerConfig) -> FollowOutput {
        guard let goal = path.last else {
            return FollowOutput(cmd: .zero, state: .arrived, target: pose.p, headingError: 0, remaining: 0)
        }
        let rem = remaining(path: path, from: pose.p).length
        if pose.p.distance(to: goal) <= cfg.goalTolerance {
            return FollowOutput(cmd: .zero, state: .arrived, target: goal, headingError: 0, remaining: 0)
        }
        let target = lookaheadPoint(path: path, from: pose.p, lookahead: min(cfg.lookahead, max(rem, 0.05)))
        let err = pose.bearing(to: target)
        var w = max(-cfg.maxTurnRate, min(cfg.maxTurnRate, cfg.turnGain * err))
        if abs(err) > cfg.rotateInPlaceAbove {
            if abs(w) < 0.35 * cfg.maxTurnRate { w = (w < 0 ? -1 : 1) * 0.35 * cfg.maxTurnRate }
            return FollowOutput(cmd: DriveCommand(v: 0, w: w), state: .rotating, target: target, headingError: err, remaining: rem)
        }
        // Speed: cap by heading error, by distance to goal (decelerate), by forward obstacle gap.
        var v = cfg.maxSpeed * max(0.25, cos(err))
        v = min(v, max(0.08, 0.8 * rem))
        if frontGap <= cfg.stopDistance {
            // Rotating in place is still safe for a round base: turning towards the path usually swings
            // the obstacle out of the forward corridor.
            let wb = abs(err) > rad(8) ? (err > 0 ? 1.0 : -1.0) * 0.35 * cfg.maxTurnRate : 0
            return FollowOutput(cmd: DriveCommand(v: 0, w: wb), state: .blocked, target: target, headingError: err, remaining: rem)
        }
        if frontGap < cfg.slowDistance {
            let t = (frontGap - cfg.stopDistance) / max(0.01, cfg.slowDistance - cfg.stopDistance)
            v *= max(0.2, t)
        }
        return FollowOutput(cmd: DriveCommand(v: v, w: w), state: .driving, target: target, headingError: err, remaining: rem)
    }

    /// Clamps any command (manual joystick, move X cm) against the forward obstacle gap.
    static func safety(_ c: DriveCommand, frontGap: Double, cfg: FollowerConfig) -> DriveCommand {
        guard c.v > 0 else { return c }
        if frontGap <= cfg.stopDistance { return DriveCommand(v: 0, w: c.w) }
        if frontGap < cfg.slowDistance {
            let t = (frontGap - cfg.stopDistance) / max(0.01, cfg.slowDistance - cfg.stopDistance)
            return DriveCommand(v: c.v * max(0.2, t), w: c.w)
        }
        return c
    }
}

// MARK: - spoken / visual guidance

enum Guidance {
    /// One short instruction for a person (or the HUD) following the path.
    static func instruction(path: [P2], pose: Pose2, frontGap: Double, cfg: FollowerConfig) -> (text: String, key: String) {
        guard let goal = path.last else { return ("Pick a goal", "idle") }
        let d = pose.p.distance(to: goal)
        if d <= cfg.goalTolerance { return ("You have arrived", "arrived") }
        if frontGap <= cfg.stopDistance { return ("Obstacle ahead, \(cm(frontGap))", "blocked") }
        let target = Follower.lookaheadPoint(path: path, from: pose.p, lookahead: 0.6)
        let err = deg(pose.bearing(to: target))
        if abs(err) >= 20 {
            let a = Int((abs(err) / 15).rounded() * 15)
            let side = err > 0 ? "left" : "right"
            if a >= 150 { return ("Turn around", "around") }
            return ("Turn \(side) \(a) degrees", "turn-\(side)-\(a)")
        }
        // Distance to the next corner of the path.
        let seg = Follower.remaining(path: path, from: pose.p).segment
        let next = min(seg + 1, path.count - 1)
        let toCorner = pose.p.distance(to: path[next])
        let stepM = toCorner < 1 ? 0.25 : 0.5
        let rounded = max(stepM, (toCorner / stepM).rounded() * stepM)
        if next == path.count - 1 { return ("Go straight \(meters(rounded)) to the goal", "straight-goal-\(rounded)") }
        let after = deg(wrapAngle(Pose2.heading(from: path[next], to: path[min(next + 1, path.count - 1)]) -
                                  Pose2.heading(from: path[seg], to: path[next])))
        let side = after > 0 ? "left" : "right"
        return ("Go straight \(meters(rounded)), then turn \(side)", "straight-\(rounded)-\(side)")
    }

    static func cm(_ m: Double) -> String { "\(Int((m * 100).rounded())) centimeters" }
    static func meters(_ m: Double) -> String {
        if m < 1 { return "\(Int((m * 100).rounded())) centimeters" }
        let s = m == m.rounded() ? String(Int(m)) : String(format: "%.1f", m)
        return "\(s) meters"
    }
}

// MARK: - robot wire protocol (robot/PROTOCOL.md)

enum RobotProtocol {
    static let version = 1

    static func vel(_ c: DriveCommand, seq: Int) -> [String: Any] {
        ["type": "vel", "v": round3(c.v), "w": round3(c.w), "seq": seq]
    }
    static func move(cm: Double, speedCmS: Double, seq: Int) -> [String: Any] {
        ["type": "move", "dist_cm": round3(cm), "speed_cms": round3(speedCmS), "seq": seq]
    }
    static func turn(deg: Double, speedDegS: Double, seq: Int) -> [String: Any] {
        ["type": "turn", "deg": round3(deg), "speed_dps": round3(speedDegS), "seq": seq]
    }
    static func stop(seq: Int) -> [String: Any] { ["type": "stop", "seq": seq] }
    static func ping(seq: Int, t: Double) -> [String: Any] { ["type": "ping", "seq": seq, "t": t] }
    static func hello(name: String) -> [String: Any] { ["type": "hello", "proto": version, "client": name] }

    /// One JSON object per line (BLE) or per WebSocket text message.
    static func encode(_ obj: [String: Any]) -> String? {
        guard JSONSerialization.isValidJSONObject(obj),
              let d = try? JSONSerialization.data(withJSONObject: obj, options: [.sortedKeys]) else { return nil }
        return String(data: d, encoding: .utf8)
    }

    static func decode(_ s: String) -> [String: Any]? {
        guard let d = s.data(using: .utf8) else { return nil }
        return (try? JSONSerialization.jsonObject(with: d)) as? [String: Any]
    }

    private static func round3(_ x: Double) -> Double { x.isFinite ? (x * 1000).rounded() / 1000 : 0 }
}
