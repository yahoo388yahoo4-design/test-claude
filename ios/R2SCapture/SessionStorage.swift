import Foundation
import Compression
import UIKit

/// Folder layout and low-level writers for the r2s-capture raw format (see capture/FORMAT.md).
enum SessionStorage {
    static var root: URL {
        let docs = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
        let dir = docs.appendingPathComponent("sessions", isDirectory: true)
        try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        return dir
    }

    static func newSession(mode: CaptureMode) throws -> URL {
        let f = DateFormatter()
        f.locale = Locale(identifier: "en_US_POSIX")
        f.dateFormat = "yyyyMMdd_HHmmss"
        let url = root.appendingPathComponent("\(f.string(from: Date()))_\(mode.rawValue)", isDirectory: true)
        try FileManager.default.createDirectory(at: url, withIntermediateDirectories: true)
        return url
    }

    static func listSessions() -> [URL] {
        let urls = (try? FileManager.default.contentsOfDirectory(at: root, includingPropertiesForKeys: [.isDirectoryKey])) ?? []
        return urls.filter { $0.hasDirectoryPath }.sorted { $0.lastPathComponent > $1.lastPathComponent }
    }

    static func size(of url: URL) -> Int64 {
        guard let e = FileManager.default.enumerator(at: url, includingPropertiesForKeys: [.fileSizeKey]) else { return 0 }
        var total: Int64 = 0
        for case let f as URL in e {
            total += Int64((try? f.resourceValues(forKeys: [.fileSizeKey]).fileSize) ?? 0)
        }
        return total
    }

    static var deviceModel: String {
        var sys = utsname()
        uname(&sys)
        let mirror = Mirror(reflecting: sys.machine)
        return mirror.children.reduce("") { acc, el in
            guard let v = el.value as? Int8, v != 0 else { return acc }
            return acc + String(UnicodeScalar(UInt8(bitPattern: v)))
        }
    }

    static var appVersion: String {
        let v = Bundle.main.infoDictionary?["CFBundleShortVersionString"] as? String ?? "?"
        let b = Bundle.main.infoDictionary?["CFBundleVersion"] as? String ?? "?"
        return "\(v) (\(b))"
    }

    static func writeJSON(_ obj: Any, to url: URL) {
        if let data = try? JSONSerialization.data(withJSONObject: obj, options: [.prettyPrinted, .sortedKeys]) {
            try? data.write(to: url)
        }
    }
}

/// Buffered, thread-safe text writer for .csv / .jsonl files.
final class LineWriter {
    private let handle: FileHandle
    private var buffer = Data()
    private let lock = NSLock()
    private(set) var lines = 0
    private var closed = false

    init(url: URL, header: String? = nil) throws {
        FileManager.default.createFile(atPath: url.path, contents: nil)
        handle = try FileHandle(forWritingTo: url)
        if let header = header { write(header) }
    }

    func write(_ line: String) {
        lock.lock()
        defer { lock.unlock() }
        guard !closed else { return }
        buffer.append(contentsOf: Array(line.utf8))
        buffer.append(0x0A)
        lines += 1
        if buffer.count > 1 << 18 {
            handle.write(buffer)
            buffer.removeAll(keepingCapacity: true)
        }
    }

    func writeJSON(_ obj: [String: Any]) {
        guard let data = try? JSONSerialization.data(withJSONObject: obj, options: []),
              let s = String(data: data, encoding: .utf8) else { return }
        write(s)
    }

    func close() {
        lock.lock()
        defer { lock.unlock() }
        guard !closed else { return }
        closed = true
        handle.write(buffer)
        buffer.removeAll()
        try? handle.close()
    }
}

/// Appends raw-deflate compressed chunks to one file and returns their byte ranges.
final class BlobWriter {
    private let handle: FileHandle
    private var offset: UInt64 = 0
    private let lock = NSLock()

    init(url: URL) throws {
        FileManager.default.createFile(atPath: url.path, contents: nil)
        handle = try FileHandle(forWritingTo: url)
    }

    /// Compresses `raw` with COMPRESSION_ZLIB (raw deflate, RFC 1951) and appends it.
    func append(raw: Data) -> [UInt64] {
        let packed = Deflate.compress(raw)
        lock.lock()
        defer { lock.unlock() }
        let start = offset
        handle.write(packed)
        offset += UInt64(packed.count)
        return [start, UInt64(packed.count)]
    }

    func close() {
        lock.lock()
        defer { lock.unlock() }
        try? handle.close()
    }
}

enum Deflate {
    /// Raw deflate (python: zlib.decompress(b, -15)).
    static func compress(_ data: Data) -> Data {
        if data.isEmpty { return Data() }
        let capacity = data.count + data.count / 8 + 1024
        let dst = UnsafeMutablePointer<UInt8>.allocate(capacity: capacity)
        defer { dst.deallocate() }
        let n: Int = data.withUnsafeBytes { (src: UnsafeRawBufferPointer) -> Int in
            guard let base = src.bindMemory(to: UInt8.self).baseAddress else { return 0 }
            return compression_encode_buffer(dst, capacity, base, data.count, nil, COMPRESSION_ZLIB)
        }
        return Data(bytes: dst, count: n)
    }
}

extension Double {
    /// Fixed-precision string for CSV output.
    func f(_ digits: Int = 6) -> String { String(format: "%.\(digits)f", self) }
}
