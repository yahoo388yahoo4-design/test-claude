import Foundation

// Turns a saved capture session (modes A / B, or a saved navigation map) into the navigation map's static
// layer: mesh.ply (ARKit scene mesh), roomplan/objects.json (RoomPlan walls / doors / objects) and
// map.pgm (a map saved from the navigation screen). Foundation only, so it is unit-tested on Linux.

/// A RoomPlan object or wall, for the minimap and the "go to" menu.
struct RoomItem: Identifiable, Equatable {
    let id: String
    let label: String
    let kind: String            // "object", "wall", "door", "opening", "window"
    let footprint: [P2]         // 4 corners (2 for a wall seen edge-on), world floor plane
    let center: P2
    let size: (w: Double, d: Double)

    static func == (a: RoomItem, b: RoomItem) -> Bool { a.id == b.id }

    /// A reachable spot next to the item: on the side facing `from`, `clearance` beyond its footprint.
    func approachPoint(from: P2, clearance: Double) -> P2 {
        let dir = from - center
        let len = max(dir.length, 1e-6)
        let half = 0.5 * (size.w * size.w + size.d * size.d).squareRoot()
        return center + dir * ((half + clearance) / len)
    }
}

struct MapPriorReport {
    var meshVertices = 0
    var meshObstacleCells = 0
    var roomItems: [RoomItem] = []
    var floorY: Double?
    var pgmLoaded = false
    var notes: [String] = []

    var summary: String {
        var parts: [String] = []
        if meshVertices > 0 { parts.append("mesh \(meshVertices / 1000)k pts") }
        let objs = roomItems.filter { $0.kind == "object" }.count
        let walls = roomItems.filter { $0.kind == "wall" }.count
        if walls + objs > 0 { parts.append("RoomPlan \(walls) walls \(objs) objects") }
        if pgmLoaded { parts.append("saved grid") }
        return parts.isEmpty ? "no map layers" : parts.joined(separator: " · ")
    }
}

enum MapPrior {
    /// Loads every map layer found in `dir` into `grid`'s static layer.
    static func load(dir: URL, into grid: OccupancyGrid, robotHeight: Double) -> MapPriorReport {
        var rep = MapPriorReport()
        let fm = FileManager.default
        let objURL = dir.appendingPathComponent("roomplan/objects.json")
        if fm.fileExists(atPath: objURL.path), let data = try? Data(contentsOf: objURL) {
            let r = loadRoomPlan(json: data, into: grid, robotHeight: robotHeight)
            rep.roomItems = r.items
            rep.floorY = r.floorY
        }
        let pgmURL = dir.appendingPathComponent("map.pgm")
        if fm.fileExists(atPath: pgmURL.path), let data = try? Data(contentsOf: pgmURL) {
            rep.pgmLoaded = grid.loadStatic(pgm: data)
            if !rep.pgmLoaded { rep.notes.append("map.pgm has a different size; skipped") }
        }
        let plyURL = dir.appendingPathComponent("mesh.ply")
        if fm.fileExists(atPath: plyURL.path), let data = try? Data(contentsOf: plyURL, options: .mappedIfSafe) {
            let r = loadMesh(ply: data, into: grid, floorY: rep.floorY, robotHeight: robotHeight)
            rep.meshVertices = r.vertices
            rep.meshObstacleCells = r.cells
            if rep.floorY == nil { rep.floorY = r.floorY }
        }
        // Doors and openings must stay passable even where the mesh saw a closed door or a wall edge.
        for item in rep.roomItems where item.kind == "door" || item.kind == "opening" {
            fill(item.footprint, grid: grid, inflate: 0.1) { grid.clearStatic($0) }
        }
        return rep
    }

    // MARK: RoomPlan objects.json (FORMAT.md)

    static func loadRoomPlan(json: Data, into grid: OccupancyGrid, robotHeight: Double) -> (items: [RoomItem], floorY: Double?) {
        guard let root = (try? JSONSerialization.jsonObject(with: json)) as? [String: Any] else { return ([], nil) }
        var items: [RoomItem] = []
        var floorY: Double?
        for f in root["floors"] as? [[String: Any]] ?? [] {
            if let T = f["T"] as? [Double], T.count == 16 { floorY = min(floorY ?? T[7], T[7]) }
        }
        for (key, kind) in [("walls", "wall"), ("doors", "door"), ("openings", "opening"), ("windows", "window"), ("objects", "object")] {
            for (n, o) in (root[key] as? [[String: Any]] ?? []).enumerated() {
                guard let T = o["T"] as? [Double], T.count == 16, let dims = o["dims"] as? [Double], dims.count == 3 else { continue }
                let w = dims[0], h = dims[1], d = dims[2]
                func world(_ a: Double, _ c: Double) -> P2 {
                    P2(T[0] * a + T[2] * c + T[3], T[8] * a + T[10] * c + T[11])
                }
                let corners = [world(-w / 2, -d / 2), world(w / 2, -d / 2), world(w / 2, d / 2), world(-w / 2, d / 2)]
                let category = o["category"] as? String ?? kind
                let label = kind == "object" ? prettify(category) : kind
                let item = RoomItem(id: o["id"] as? String ?? "\(key)\(n)", label: label, kind: kind, footprint: corners,
                                    center: P2(T[3], T[11]), size: (w, d))
                items.append(item)
                switch kind {
                case "wall":
                    fill(corners, grid: grid, inflate: 0.03) { grid.markStatic($0) }
                case "object":
                    // Skip things entirely above the robot (wall cabinets, a TV on the wall).
                    let bottom = T[7] - h / 2
                    if let fy = floorY, bottom - fy > robotHeight { continue }
                    fill(corners, grid: grid, inflate: 0.0) { grid.markStatic($0) }
                default:
                    break
                }
            }
        }
        return (items, floorY)
    }

    static func prettify(_ category: String) -> String {
        // String(describing: CapturedRoom.Object.Category) gives e.g. "table" or "storage"; some carry
        // associated values in newer SDKs ("chair(...)"), keep the base name.
        let base = category.split(separator: "(").first.map(String.init) ?? category
        return base.replacingOccurrences(of: "_", with: " ")
    }

    /// Calls `mark` for points covering the convex quad (or segment) `poly`, grown by `inflate` metres.
    static func fill(_ poly: [P2], grid: OccupancyGrid, inflate: Double, mark: (P2) -> Void) {
        guard poly.count >= 2 else { return }
        let step = grid.cell * 0.5
        if poly.count == 4 {
            let a = poly[0], b = poly[1], d = poly[3]
            let u = b - a, v = d - a
            let lu = max(u.length, 1e-9), lv = max(v.length, 1e-9)
            let uh = u * (1 / lu), vh = v * (1 / lv)
            var s = -inflate
            while s <= lu + inflate {
                var t = -inflate
                while t <= lv + inflate {
                    mark(a + uh * s + vh * t)
                    t += step
                }
                s += step
            }
        } else {
            for k in 0..<(poly.count - 1) {
                let a = poly[k], b = poly[k + 1]
                let n = max(1, Int(a.distance(to: b) / step))
                for i in 0...n { mark(a + (b - a) * (Double(i) / Double(n))) }
            }
        }
    }

    // MARK: mesh.ply (FORMAT.md: binary LE, vertex x y z nx ny nz float, face uchar n, int v0 v1 v2, uchar cls)

    static func loadMesh(ply data: Data, into grid: OccupancyGrid, floorY: Double?, robotHeight: Double) -> (vertices: Int, cells: Int, floorY: Double?) {
        guard let headerEnd = data.range(of: Data("end_header\n".utf8)) else { return (0, 0, nil) }
        let header = String(decoding: data[data.startIndex..<headerEnd.lowerBound], as: UTF8.self)
        var nv = 0, nf = 0
        for line in header.split(separator: "\n") {
            let p = line.split(separator: " ")
            if p.count == 3, p[0] == "element" {
                if p[1] == "vertex" { nv = Int(p[2]) ?? 0 }
                if p[1] == "face" { nf = Int(p[2]) ?? 0 }
            }
        }
        let vStart = headerEnd.upperBound - data.startIndex
        let fStart = vStart + nv * 24
        guard nv > 0, data.count >= fStart + nf * 14 else { return (0, 0, nil) }
        return data.withUnsafeBytes { (raw: UnsafeRawBufferPointer) -> (Int, Int, Double?) in
            func vy(_ i: Int) -> Float { raw.loadUnaligned(fromByteOffset: vStart + i * 24 + 4, as: Float.self) }
            func vxz(_ i: Int) -> P2 {
                P2(Double(raw.loadUnaligned(fromByteOffset: vStart + i * 24, as: Float.self)),
                   Double(raw.loadUnaligned(fromByteOffset: vStart + i * 24 + 8, as: Float.self)))
            }
            // Floor height: the RoomPlan floor if known, else the median height of floor-class faces, else
            // the 2nd percentile of all vertex heights.
            var fy = floorY
            var floorVerts: [Int] = []
            var obstacleFaces: [(Int32, Int32, Int32)] = []
            for f in 0..<nf {
                let off = fStart + f * 14
                let i0 = raw.loadUnaligned(fromByteOffset: off + 1, as: Int32.self)
                let i1 = raw.loadUnaligned(fromByteOffset: off + 5, as: Int32.self)
                let i2 = raw.loadUnaligned(fromByteOffset: off + 9, as: Int32.self)
                let cls = raw[off + 13]
                guard i0 >= 0, i1 >= 0, i2 >= 0, Int(i0) < nv, Int(i1) < nv, Int(i2) < nv else { continue }
                switch cls {
                case 2: floorVerts.append(Int(i0))      // floor
                case 3: break                           // ceiling
                default: obstacleFaces.append((i0, i1, i2))
                }
            }
            if fy == nil {
                if floorVerts.count > 50 {
                    let ys = floorVerts.map { vy($0) }.sorted()
                    fy = Double(ys[ys.count / 2])
                } else {
                    var ys: [Float] = []
                    ys.reserveCapacity(nv / 7 + 1)
                    for i in stride(from: 0, to: nv, by: 7) { ys.append(vy(i)) }
                    ys.sort()
                    if !ys.isEmpty { fy = Double(ys[ys.count / 50]) }
                }
            }
            guard let floor = fy else { return (nv, 0, nil) }
            var cells = Set<Int>()
            func consider(_ i: Int32) {
                let h = Double(vy(Int(i))) - floor
                guard h > 0.06, h < robotHeight else { return }
                let p = vxz(Int(i))
                if let (ci, cj) = grid.cellOf(p) { cells.insert(cj * grid.size + ci) }
            }
            for (a, b, c) in obstacleFaces { consider(a); consider(b); consider(c) }
            for k in cells { grid.markStatic(grid.centerOf(k % grid.size, k / grid.size)) }
            // Seen floor counts as observed free space, once per cell and weaker than staticOverride, so it
            // never erases a wall cell.
            var floorCells = Set<Int>()
            for i in floorVerts {
                if let (ci, cj) = grid.cellOf(vxz(i)) { floorCells.insert(cj * grid.size + ci) }
            }
            for k in floorCells.subtracting(cells) { grid.markFree(grid.centerOf(k % grid.size, k / grid.size), amount: -0.7) }
            return (nv, cells.count, floor)
        }
    }
}
