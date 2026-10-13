// Compile with production Swift files except LunarRobotConsoleApp.swift; run only
// with an explicitly started isolated NUC simulator. No physical robot access.
import Foundation

@main
struct SimulationAcceptance {
    @MainActor
    static func main() async {
        func wait(_ milliseconds: UInt64) async {
            try? await Task.sleep(nanoseconds: milliseconds * 1_000_000)
        }

        let combo = DriveKey.axes([.w, .a])
        precondition(combo.forward == 1 && combo.turn == 1, "W+A")
        precondition(DriveKey.axes([.s, .d]).forward == -1, "S+D")
        precondition(DriveKey.axes([.w, .s]).forward == 0, "W+S cancel")
        precondition(DriveKey.axes([.a, .d]).turn == 0, "A+D cancel")

        let model = SimulationModel()
        model.start()
        for _ in 0..<45 {
            if model.ready && model.map != nil && model.state?.odom != nil { break }
            await wait(100)
        }
        guard model.ready, let map = model.map, let start = model.state?.odom else {
            fatalError("Mac -> SSH -> NUC simulation unavailable: \(model.message)")
        }
        precondition(map.valid)
        print("Mac tunnel and map OK: \(map.width)x\(map.height)")

        model.setFocus(true)
        model.setKeys([.w, .a])
        var moved = false
        for _ in 0..<25 {
            await wait(100)
            if let v = model.state?.odom {
                if hypot(v.x - start.x, v.y - start.y) > 0.12 && abs(atan2(sin(v.yaw - start.yaw), cos(v.yaw - start.yaw))) > 0.20 { moved = true }
            }
        }
        precondition(moved, "WASD combined motion")

        model.setKeys([])
        await wait(500)
        guard let stopped = model.state?.odom else { fatalError("missing stopped odometry") }
        precondition(abs(stopped.linear) < 0.001 && abs(stopped.angular) < 0.001, "key-up must stop")

        model.setKeys([.s, .d])
        var reverseSuccess = false
        for _ in 0..<20 {
            await wait(100)
            if let reverse = model.state?.odom {
                if reverse.linear < -0.10 && reverse.angular < -0.20 {
                    reverseSuccess = true
                    break
                }
            }
        }
        precondition(reverseSuccess, "reverse + right turn must be observed")
        model.setFocus(false)
        var didStop = false
        for _ in 0..<14 {
            await wait(100)
            if let latest = model.state?.odom {
                if abs(latest.linear) < 0.001 && abs(latest.angular) < 0.001 {
                    didStop = true
                    break
                }
            }
        }
        precondition(didStop, "focus loss must stop and report zero velocity")

        model.stop()
        print("PASS: Mac -> SSH -> NUC P6, map, W+A, S+D, release, focus loss")
    }
}
