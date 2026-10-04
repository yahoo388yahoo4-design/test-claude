import Foundation

/// Uploads a session folder file-by-file with HTTP PUT to tools/receiver.py:
///   PUT <base>/upload/<session>/<relative path>   (header X-R2S-Token if a token is set in the URL fragment)
/// Files that are already complete on the receiver (same size, per HEAD) are skipped, so an
/// interrupted upload can simply be restarted.
enum Uploader {
    static func upload(session dir: URL, to baseString: String, progress: @escaping (String) -> Void) async throws {
        var base = baseString.trimmingCharacters(in: .whitespaces)
        var token: String?
        if let hash = base.firstIndex(of: "#") {
            token = String(base[base.index(after: hash)...])
            base = String(base[..<hash])
        }
        while base.hasSuffix("/") { base.removeLast() }
        guard URL(string: base) != nil else { throw URLError(.badURL) }
        let files = enumerate(dir)
        let config = URLSessionConfiguration.default
        config.timeoutIntervalForRequest = 600
        config.timeoutIntervalForResource = 24 * 3600
        let session = URLSession(configuration: config)
        var done = 0
        for (rel, url, size) in files {
            let enc = rel.addingPercentEncoding(withAllowedCharacters: .urlPathAllowed) ?? rel
            guard let target = URL(string: "\(base)/upload/\(dir.lastPathComponent)/\(enc)") else { continue }
            var head = URLRequest(url: target)
            head.httpMethod = "HEAD"
            if let token = token { head.setValue(token, forHTTPHeaderField: "X-R2S-Token") }
            if let r = try? await session.data(for: head), let http = r.1 as? HTTPURLResponse,
               http.statusCode == 200, http.expectedContentLength == size {
                done += 1
                progress("skip \(rel) (\(done)/\(files.count))")
                continue
            }
            var req = URLRequest(url: target)
            req.httpMethod = "PUT"
            req.setValue("application/octet-stream", forHTTPHeaderField: "Content-Type")
            if let token = token { req.setValue(token, forHTTPHeaderField: "X-R2S-Token") }
            progress("upload \(rel) \(ByteCountFormatter.string(fromByteCount: size, countStyle: .file)) (\(done + 1)/\(files.count))")
            let (_, resp) = try await session.upload(for: req, fromFile: url)
            guard let http = resp as? HTTPURLResponse, (200..<300).contains(http.statusCode) else {
                throw URLError(.badServerResponse)
            }
            done += 1
        }
        // marker so the receiver knows the session is complete
        if let target = URL(string: "\(base)/upload/\(dir.lastPathComponent)/.complete") {
            var req = URLRequest(url: target)
            req.httpMethod = "PUT"
            if let token = token { req.setValue(token, forHTTPHeaderField: "X-R2S-Token") }
            _ = try? await session.upload(for: req, from: Data("\(files.count)\n".utf8))
        }
        progress("uploaded \(files.count) files")
    }

    static func enumerate(_ dir: URL) -> [(String, URL, Int64)] {
        guard let e = FileManager.default.enumerator(at: dir, includingPropertiesForKeys: [.isRegularFileKey, .fileSizeKey]) else { return [] }
        var out: [(String, URL, Int64)] = []
        let prefix = dir.standardizedFileURL.path + "/"
        for case let url as URL in e {
            let v = try? url.resourceValues(forKeys: [.isRegularFileKey, .fileSizeKey])
            guard v?.isRegularFile == true else { continue }
            let rel = url.standardizedFileURL.path.replacingOccurrences(of: prefix, with: "")
            out.append((rel, url, Int64(v?.fileSize ?? 0)))
        }
        // small metadata first, big binaries last
        return out.sorted { $0.2 < $1.2 }
    }
}
