import Foundation

// LiDAR point classification for navigation: which depth points are obstacles, floor or drops, the
// per-sector and forward gaps, and the floor-height estimate. Foundation only, so it is unit-tested on
// Linux together with NavCore (ios/NavTests).

/// One back-projected LiDAR depth sample in ARKit world coordinates (metres, y up).
struct DepthPoint {
    var x: Double
    var y: Double
    var z: Double
    var conf: UInt8          // ARConfidenceLevel: 0 low, 1 medium, 2 high
}

enum FloorSource: String {
    case plane = "floor plane"
    case estimate = "estimated from camera height"
}

/// Floor height: the detected floor plane once ARKit has one, else camera height minus the configured
/// mount height. Switching to the plane snaps straight to it (blending from a wrong guess made the real
/// floor look like a drop for a second or two).
struct FloorEstimator {
    private(set) var y: Double?
    private(set) var source: FloorSource = .estimate
    private var planeSince: Double = 0

    mutating func update(planeY: Double?, cameraY: Double, mountHeight: Double, now: Double) {
        if let p = planeY {
            if source != .plane || y == nil {
                y = p
                source = .plane
                planeSince = now
            } else if let cur = y {
                // A lower plane appearing later is the real floor (the first one may be a low step).
                y = p < cur - 0.08 ? p : 0.9 * cur + 0.1 * p
            }
        } else if source != .plane {
            y = cameraY - mountHeight
        }
    }

    /// True once a floor plane has been tracked for a second; only then are drops trusted.
    func stable(now: Double) -> Bool { source == .plane && now - planeSince > 1.0 }

    mutating func reset() {
        y = nil
        source = .estimate
        planeSince = 0
    }
}

struct PerceptionConfig {
    var robotRadius = 0.20
    var robotHeight = 0.60
    var minObstacleHeight = 0.05     // above the floor
    var floorBand = 0.08             // |height| below this (and above -this) is floor
    var cliffDrop = 0.15             // this far below the floor is a drop (stairs down, a hole)
    var minConfidence: UInt8 = 1     // medium
    /// A sector or the forward corridor needs this many obstacle points before it counts as blocked, so
    /// single noisy LiDAR pixels cannot stop the robot.
    var minSupport = 6
    var maxRange = 5.0
}

struct PerceptionResult {
    var hits: [P2] = []              // obstacle points for the map (drops are not mapped)
    var floor: [P2] = []
    var sectors: [Double] = []       // gap from the robot's edge per sector, .infinity = clear
    var markers: [(x: Double, y: Double, z: Double)?] = []
    var frontGap = Double.infinity   // forward clearance in the robot's corridor
    var obstaclePoints = 0
    var cliffPoints = 0
    var debugObstacles: [(x: Double, y: Double, z: Double)] = []
    var debugCliffs: [(x: Double, y: Double, z: Double)] = []
}

enum Perception {
    /// Sector centres, degrees, + = left; 10 degrees wide.
    static let sectorBearings: [Double] = stride(from: 40.0, through: -40.0, by: -10.0).map { $0 }

    /// Classifies depth points around a robot at `pose` (robot centre on the floor plane).
    /// `cliffsAllowed` should be true only when the floor height comes from a stable floor plane.
    static func classify(_ pts: [DepthPoint], pose: Pose2, floorY: Double, cliffsAllowed: Bool,
                         cfg: PerceptionConfig, debugLimit: Int = 600) -> PerceptionResult {
        var res = PerceptionResult()
        let n = sectorBearings.count
        var sectorGaps = [[(Double, Int)]](repeating: [], count: n)
        var corridor: [Double] = []
        var obstacleIdx: [Int] = []
        var cliffIdx: [Int] = []
        let r = cfg.robotRadius
        let cosT = cos(pose.theta), sinT = sin(pose.theta)
        for (i, p) in pts.enumerated() {
            guard p.conf >= cfg.minConfidence else { continue }
            let q = P2(p.x, p.z)
            let dx = q.x - pose.p.x, dz = q.z - pose.p.z
            let lf = dx * cosT - dz * sinT
            let ll = -dx * sinT - dz * cosT
            let dist = (lf * lf + ll * ll).squareRoot()
            if dist < r + 0.03 || dist > cfg.maxRange { continue }   // the robot's own body / too far
            let h = p.y - floorY
            var isCliff = false
            if abs(h) < cfg.floorBand {
                res.floor.append(q)
                continue
            } else if h < 0 {
                guard cliffsAllowed, h < -cfg.cliffDrop else { continue }
                isCliff = true
            } else if h < cfg.minObstacleHeight || h > cfg.robotHeight {
                continue
            }
            if isCliff { cliffIdx.append(i) } else { res.hits.append(q); obstacleIdx.append(i) }
            let gap = dist - r
            let b = deg(atan2(ll, lf))
            let s = Int(((40 - b) / 10).rounded())
            if s >= 0, s < n, abs(b) <= 45 { sectorGaps[s].append((gap, i)) }
            // Forward clearance of a round robot to a point at lateral offset ll inside its width.
            if lf > 0, abs(ll) < r {
                corridor.append(max(0, lf - (r * r - ll * ll).squareRoot()))
            }
        }
        let k = max(1, cfg.minSupport)
        res.sectors = [Double](repeating: .infinity, count: n)
        res.markers = Array(repeating: nil, count: n)
        for s in 0..<n where sectorGaps[s].count >= k {
            let sorted = sectorGaps[s].sorted { $0.0 < $1.0 }
            let (g, i) = sorted[k - 1]
            res.sectors[s] = max(0, g)
            res.markers[s] = (pts[i].x, pts[i].y, pts[i].z)
        }
        if corridor.count >= k {
            corridor.sort()
            res.frontGap = corridor[k - 1]
        }
        res.obstaclePoints = obstacleIdx.count
        res.cliffPoints = cliffIdx.count
        let stepO = max(1, obstacleIdx.count / max(1, debugLimit))
        for j in stride(from: 0, to: obstacleIdx.count, by: stepO) {
            let p = pts[obstacleIdx[j]]
            res.debugObstacles.append((p.x, p.y, p.z))
        }
        let stepC = max(1, cliffIdx.count / max(1, debugLimit / 4))
        for j in stride(from: 0, to: cliffIdx.count, by: stepC) {
            let p = pts[cliffIdx[j]]
            res.debugCliffs.append((p.x, p.y, p.z))
        }
        return res
    }

    /// Forward clearance from the map: the first occupied cell inside the robot's width ahead, up to
    /// `range` metres, measured like `classify` (round robot).
    static func corridorGap(grid: OccupancyGrid, pose: Pose2, radius r: Double, range: Double = 2) -> Double {
        let fwd = pose.forward
        let left = P2(-sin(pose.theta), -cos(pose.theta))
        var best = Double.infinity
        var d = 0.0
        while d < r + range {
            for lat in stride(from: -r + grid.cell / 2, through: r - grid.cell / 2, by: grid.cell) {
                let q = pose.p + fwd * d + left * lat
                if (q - pose.p).length < r + grid.cell { continue }   // the robot's own footprint
                if grid.isOccupied(q) {
                    best = min(best, max(0, d - (r * r - lat * lat).squareRoot()))
                }
            }
            if best.isFinite { return best }
            d += grid.cell
        }
        return best
    }
}
