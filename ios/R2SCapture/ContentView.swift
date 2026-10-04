import ARKit
import AVFoundation
import RoomPlan
import SwiftUI
import UIKit

struct ContentView: View {
    @EnvironmentObject var model: CaptureModel
    @State private var showSettings = false
    @State private var showSessions = false

    var body: some View {
        ZStack {
            preview.ignoresSafeArea()
            VStack(spacing: 10) {
                VStack(alignment: .leading, spacing: 4) {
                    Text(model.status).font(.footnote.weight(.semibold))
                    if !model.statsLine.isEmpty {
                        Text(model.statsLine).font(.caption.monospacedDigit())
                    }
                }
                .padding(10)
                .frame(maxWidth: .infinity, alignment: .leading)
                .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 12))
                .padding(.horizontal)
                Spacer()
                Picker("Mode", selection: $model.settings.mode) {
                    ForEach(CaptureMode.allCases) { Text($0.shortTitle).tag($0) }
                }
                .pickerStyle(.segmented)
                .disabled(model.isRecording || model.isFinishing)
                .padding(.horizontal)
                .onChange(of: model.settings.mode) { _, _ in model.modeChanged() }
                Text(model.settings.mode.title).font(.caption).foregroundStyle(.secondary)
                HStack(spacing: 40) {
                    Button { showSessions = true } label: {
                        Image(systemName: "folder").font(.title2)
                    }
                    .disabled(model.isRecording)
                    Button(action: model.toggleRecording) {
                        ZStack {
                            Circle().stroke(Color.white, lineWidth: 4).frame(width: 76, height: 76)
                            RoundedRectangle(cornerRadius: model.isRecording ? 8 : 32)
                                .fill(Color.red)
                                .frame(width: model.isRecording ? 34 : 62, height: model.isRecording ? 34 : 62)
                        }
                    }
                    .disabled(model.isFinishing)
                    Button { showSettings = true } label: {
                        Image(systemName: "gearshape").font(.title2)
                    }
                    .disabled(model.isRecording)
                }
                .foregroundStyle(.white)
                .padding(.bottom, 20)
            }
        }
        .onAppear { model.activate() }
        .sheet(isPresented: $showSettings, onDismiss: { model.modeChanged() }) {
            SettingsView().environmentObject(model)
        }
        .sheet(isPresented: $showSessions) {
            SessionsView().environmentObject(model)
        }
    }

    @ViewBuilder private var preview: some View {
        switch model.settings.mode {
        case .arkitRGBD:
            ZStack {
                ARPreview(session: model.ar.session)
                CoverageOverlay(session: model.ar.session, resetToken: model.recordingIndex)
            }
        case .arkitRoomPlan:
            ZStack {
                RoomPreview(recorder: model.room, session: model.ar.session)
                CoverageOverlay(session: model.ar.session, resetToken: model.recordingIndex)
            }
        case .multiCam:
            ZStack(alignment: .topTrailing) {
                MultiCamPreview(recorder: model.multicam)
                if let img = model.depthPreview {
                    // AVFoundation depth is in sensor (landscape) orientation; the app is portrait.
                    Image(uiImage: UIImage(cgImage: img, scale: 1, orientation: .right))
                        .resizable()
                        .scaledToFit()
                        .frame(width: 120)
                        .clipShape(RoundedRectangle(cornerRadius: 8))
                        .overlay(alignment: .bottom) {
                            Text("LiDAR depth").font(.caption2).padding(2)
                        }
                        .padding(.top, 140)
                        .padding(.trailing, 12)
                        .allowsHitTesting(false)
                }
            }
        }
    }
}

// MARK: previews

struct ARPreview: UIViewRepresentable {
    let session: ARSession
    func makeUIView(context: Context) -> ARSCNView {
        let v = ARSCNView(frame: .zero)
        v.session = session
        v.automaticallyUpdatesLighting = false
        v.rendersCameraGrain = false
        return v
    }
    func updateUIView(_ uiView: ARSCNView, context: Context) {
        if uiView.session !== session { uiView.session = session }
    }
}

struct RoomPreview: UIViewRepresentable {
    let recorder: RoomPlanRecorder
    let session: ARSession
    func makeUIView(context: Context) -> RoomCaptureView {
        recorder.makeView(arSession: session)
    }
    func updateUIView(_ uiView: RoomCaptureView, context: Context) {}
}

final class PreviewHostView: UIView {
    override class var layerClass: AnyClass { AVCaptureVideoPreviewLayer.self }
    var previewLayer: AVCaptureVideoPreviewLayer { layer as! AVCaptureVideoPreviewLayer }
}

struct MultiCamPreview: UIViewRepresentable {
    let recorder: MultiCamRecorder
    func makeUIView(context: Context) -> PreviewHostView {
        let v = PreviewHostView()
        v.previewLayer.videoGravity = .resizeAspectFill
        recorder.attachPreview(v.previewLayer)
        return v
    }
    func updateUIView(_ uiView: PreviewHostView, context: Context) {
        recorder.attachPreview(uiView.previewLayer)
    }
}

// MARK: settings

struct SettingsView: View {
    @EnvironmentObject var model: CaptureModel
    @Environment(\.dismiss) private var dismiss

    var body: some View {
        NavigationStack {
            Form {
                Section("Mode") {
                    Picker("Capture mode", selection: $model.settings.mode) {
                        ForEach(CaptureMode.allCases) { Text($0.title).tag($0) }
                    }
                    .pickerStyle(.inline)
                    .labelsHidden()
                }
                Section("Video (modes A/B)") {
                    Picker("Format", selection: $model.settings.videoPreset) {
                        ForEach(VideoPreset.allCases) { Text($0.rawValue).tag($0) }
                    }
                    Stepper("Bitrate \(Int(model.settings.videoBitrateMbps)) Mbps", value: $model.settings.videoBitrateMbps, in: 10...150, step: 10)
                    Toggle("Lock focus", isOn: $model.settings.lockFocus)
                    Toggle("Lock exposure + white balance", isOn: $model.settings.lockExposureAndWhiteBalance)
                    Stepper(model.settings.hiresStillIntervalSec == 0 ? "Full-res stills: off" :
                                String(format: "Full-res still every %.0f s", model.settings.hiresStillIntervalSec),
                            value: $model.settings.hiresStillIntervalSec, in: 0...10, step: 1)
                    Toggle("Audio track", isOn: $model.settings.recordAudio)
                }
                Section("Depth and geometry (modes A/B)") {
                    Toggle("Smoothed depth too", isOn: $model.settings.smoothedDepth)
                    Toggle("Scene mesh with classes", isOn: $model.settings.sceneMesh)
                    Toggle("Save ARWorldMap", isOn: $model.settings.saveWorldMap)
                }
                Section("Mode C") {
                    Toggle("Include front camera", isOn: $model.settings.multicamIncludeFront)
                }
                Section("Sensors") {
                    Toggle("GPS + compass", isOn: $model.settings.recordLocation)
                    Text("IMU (200 Hz requested), magnetometer, barometer, thermal and battery are always recorded.")
                        .font(.caption).foregroundStyle(.secondary)
                }
                Section("Upload receiver (tools/receiver.py)") {
                    TextField("http://192.168.1.20:8765#token", text: $model.settings.uploadURL)
                        .keyboardType(.URL)
                        .textInputAutocapitalization(.never)
                        .autocorrectionDisabled()
                    Text("Or copy sessions with the Files app / Finder (On My iPhone › R2S Capture › sessions).")
                        .font(.caption).foregroundStyle(.secondary)
                }
                Section {
                    Text("LiDAR: \(model.lidarAvailable ? "yes" : "no") · RoomPlan: \(RoomPlanRecorder.isSupported ? "yes" : "no") · MultiCam: \(MultiCamRecorder.isSupported ? "yes" : "no")")
                        .font(.caption)
                }
            }
            .navigationTitle("Settings")
            .toolbar { Button("Done") { dismiss() } }
        }
    }
}

// MARK: sessions

struct SessionsView: View {
    @EnvironmentObject var model: CaptureModel
    @Environment(\.dismiss) private var dismiss

    var body: some View {
        NavigationStack {
            List {
                if !model.uploadStatus.isEmpty {
                    Text(model.uploadStatus).font(.caption)
                }
                ForEach(model.sessions, id: \.self) { url in
                    VStack(alignment: .leading) {
                        Text(url.lastPathComponent).font(.body.monospaced())
                        Text(ByteCountFormatter.string(fromByteCount: SessionStorage.size(of: url), countStyle: .file))
                            .font(.caption).foregroundStyle(.secondary)
                    }
                    .swipeActions {
                        Button(role: .destructive) { model.delete(url) } label: { Label("Delete", systemImage: "trash") }
                        Button { model.upload(url) } label: { Label("Upload", systemImage: "arrow.up.circle") }
                            .tint(.blue)
                    }
                    .contextMenu {
                        ShareLink(item: url) { Label("Share folder", systemImage: "square.and.arrow.up") }
                        Button { model.upload(url) } label: { Label("Upload to receiver", systemImage: "arrow.up.circle") }
                    }
                }
            }
            .navigationTitle("Sessions")
            .toolbar { Button("Done") { dismiss() } }
            .onAppear { model.refreshSessions() }
        }
    }
}
