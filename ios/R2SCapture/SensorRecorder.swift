import CoreLocation
import CoreMotion
import Foundation
import UIKit

/// Records every non-camera sensor on the device's uptime clock (same timebase as ARFrame.timestamp):
/// fused device motion, raw accelerometer / gyroscope / magnetometer, barometer, GPS, compass heading,
/// thermal state and battery.
final class SensorRecorder: NSObject, CLLocationManagerDelegate {
    private let motion = CMMotionManager()
    private let altimeter = CMAltimeter()
    private let location = CLLocationManager()
    private let queue: OperationQueue = {
        let q = OperationQueue()
        q.name = "r2s.sensors"
        q.maxConcurrentOperationCount = 1
        q.qualityOfService = .userInitiated
        return q
    }()
    private var writers: [String: LineWriter] = [:]
    private var statusTimer: Timer?
    private(set) var imuCount = 0
    private(set) var locationCount = 0
    private(set) var lastLocation: CLLocation?

    func requestPermissions(location wantLocation: Bool) {
        if wantLocation && location.authorizationStatus == .notDetermined {
            location.requestWhenInUseAuthorization()
        }
    }

    func start(in dir: URL, recordLocation: Bool, rate: Double = 200) {
        stop()
        imuCount = 0
        locationCount = 0
        func w(_ name: String, _ header: String) -> LineWriter? {
            let lw = try? LineWriter(url: dir.appendingPathComponent(name), header: header)
            writers[name] = lw
            return lw
        }
        let dt = 1.0 / rate
        if motion.isDeviceMotionAvailable, let out = w("imu.csv", "t,ax,ay,az,gx,gy,gz,grx,gry,grz,qx,qy,qz,qw,mx,my,mz,mag_acc,heading") {
            motion.deviceMotionUpdateInterval = dt
            let frames = CMMotionManager.availableAttitudeReferenceFrames()
            let ref: CMAttitudeReferenceFrame = frames.contains(.xMagneticNorthZVertical) ? .xMagneticNorthZVertical : .xArbitraryCorrectedZVertical
            motion.startDeviceMotionUpdates(using: ref, to: queue) { [weak self] m, _ in
                guard let m = m else { return }
                let q = m.attitude.quaternion
                let mf = m.magneticField
                out.write([m.timestamp.f(6),
                           m.userAcceleration.x.f(), m.userAcceleration.y.f(), m.userAcceleration.z.f(),
                           m.rotationRate.x.f(), m.rotationRate.y.f(), m.rotationRate.z.f(),
                           m.gravity.x.f(), m.gravity.y.f(), m.gravity.z.f(),
                           q.x.f(7), q.y.f(7), q.z.f(7), q.w.f(7),
                           mf.field.x.f(3), mf.field.y.f(3), mf.field.z.f(3), String(mf.accuracy.rawValue),
                           m.heading.f(2)].joined(separator: ","))
                self?.imuCount += 1
            }
        }
        if motion.isAccelerometerAvailable, let out = w("accel.csv", "t,x,y,z") {
            motion.accelerometerUpdateInterval = dt
            motion.startAccelerometerUpdates(to: queue) { d, _ in
                guard let d = d else { return }
                out.write("\(d.timestamp.f(6)),\(d.acceleration.x.f()),\(d.acceleration.y.f()),\(d.acceleration.z.f())")
            }
        }
        if motion.isGyroAvailable, let out = w("gyro.csv", "t,x,y,z") {
            motion.gyroUpdateInterval = dt
            motion.startGyroUpdates(to: queue) { d, _ in
                guard let d = d else { return }
                out.write("\(d.timestamp.f(6)),\(d.rotationRate.x.f()),\(d.rotationRate.y.f()),\(d.rotationRate.z.f())")
            }
        }
        if motion.isMagnetometerAvailable, let out = w("mag.csv", "t,x,y,z") {
            motion.magnetometerUpdateInterval = 1.0 / 100.0
            motion.startMagnetometerUpdates(to: queue) { d, _ in
                guard let d = d else { return }
                out.write("\(d.timestamp.f(6)),\(d.magneticField.x.f(3)),\(d.magneticField.y.f(3)),\(d.magneticField.z.f(3))")
            }
        }
        if CMAltimeter.isRelativeAltitudeAvailable(), let out = w("altimeter.csv", "t,rel_alt_m,pressure_kpa") {
            altimeter.startRelativeAltitudeUpdates(to: queue) { d, _ in
                guard let d = d else { return }
                out.write("\(d.timestamp.f(6)),\(d.relativeAltitude.doubleValue.f(4)),\(d.pressure.doubleValue.f(5))")
            }
        }
        if CMAltimeter.isAbsoluteAltitudeAvailable(), let out = w("altimeter_abs.csv", "t,alt_m,accuracy,precision") {
            altimeter.startAbsoluteAltitudeUpdates(to: queue) { d, _ in
                guard let d = d else { return }
                out.write("\(ProcessInfo.processInfo.systemUptime.f(6)),\(d.altitude.f(3)),\(d.accuracy.f(3)),\(d.precision.f(3))")
            }
        }
        if recordLocation {
            _ = w("location.csv", "t,unix,lat,lon,alt,ellipsoidal_alt,hacc,vacc,speed,speed_acc,course,course_acc,floor,simulated")
            _ = w("heading.csv", "t,true_heading,magnetic_heading,accuracy,x,y,z")
            location.delegate = self
            location.desiredAccuracy = kCLLocationAccuracyBestForNavigation
            location.distanceFilter = kCLDistanceFilterNone
            location.headingFilter = kCLHeadingFilterNone
            location.activityType = .other
            location.startUpdatingLocation()
            if CLLocationManager.headingAvailable() { location.startUpdatingHeading() }
        }
        let status = w("status.csv", "t,unix,thermal,battery,battery_state,low_power")
        let clock = w("clock.csv", "uptime,unix")
        DispatchQueue.main.async {
            UIDevice.current.isBatteryMonitoringEnabled = true
            self.statusTimer = Timer.scheduledTimer(withTimeInterval: 1.0, repeats: true) { _ in
                let up = ProcessInfo.processInfo.systemUptime
                let now = Date().timeIntervalSince1970
                let p = ProcessInfo.processInfo
                status?.write("\(up.f(6)),\(now.f(6)),\(p.thermalState.rawValue),\(Double(UIDevice.current.batteryLevel).f(3)),\(UIDevice.current.batteryState.rawValue),\(p.isLowPowerModeEnabled ? 1 : 0)")
                clock?.write("\(up.f(6)),\(now.f(6))")
            }
            self.statusTimer?.fire()
        }
    }

    func stop() {
        motion.stopDeviceMotionUpdates()
        motion.stopAccelerometerUpdates()
        motion.stopGyroUpdates()
        motion.stopMagnetometerUpdates()
        altimeter.stopRelativeAltitudeUpdates()
        altimeter.stopAbsoluteAltitudeUpdates()
        location.stopUpdatingLocation()
        location.stopUpdatingHeading()
        let timer = statusTimer
        statusTimer = nil
        DispatchQueue.main.async { timer?.invalidate() }
        queue.waitUntilAllOperationsAreFinished()
        writers.values.forEach { $0.close() }
        writers.removeAll()
    }

    // MARK: CLLocationManagerDelegate

    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        guard let out = writers["location.csv"] else { return }
        let nowUp = ProcessInfo.processInfo.systemUptime
        let nowWall = Date()
        for l in locations {
            // GPS fixes carry wall-clock time; map them onto the uptime clock.
            let t = nowUp - nowWall.timeIntervalSince(l.timestamp)
            let floor = l.floor.map { String($0.level) } ?? ""
            let sim = l.sourceInformation?.isSimulatedBySoftware == true ? 1 : 0
            out.write([t.f(6), l.timestamp.timeIntervalSince1970.f(6), l.coordinate.latitude.f(8), l.coordinate.longitude.f(8),
                       l.altitude.f(3), l.ellipsoidalAltitude.f(3), l.horizontalAccuracy.f(2), l.verticalAccuracy.f(2),
                       l.speed.f(3), l.speedAccuracy.f(3), l.course.f(2), l.courseAccuracy.f(2), floor, String(sim)]
                .joined(separator: ","))
            locationCount += 1
            lastLocation = l
        }
    }

    func locationManager(_ manager: CLLocationManager, didUpdateHeading h: CLHeading) {
        guard let out = writers["heading.csv"] else { return }
        let t = ProcessInfo.processInfo.systemUptime - Date().timeIntervalSince(h.timestamp)
        out.write("\(t.f(6)),\(h.trueHeading.f(2)),\(h.magneticHeading.f(2)),\(h.headingAccuracy.f(2)),\(h.x.f(3)),\(h.y.f(3)),\(h.z.f(3))")
    }

    func locationManager(_ manager: CLLocationManager, didFailWithError error: Error) {}
}
