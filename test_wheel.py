# -*- coding: utf-8 -*-

import argparse
import time

from PyDynamixel import DxlComm, Wheel


def parse_args():
    parser = argparse.ArgumentParser(description="Wheel test for IDs (default 1..4)")
    parser.add_argument("--ids", nargs="*", type=int, default=[1, 2, 3, 4], help="Servo IDs to test")
    parser.add_argument("--id", type=int, help="Single Servo ID (overrides --ids if provided)")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port device")
    parser.add_argument("--baudrate", type=int, default=57600, help="Serial baudrate")
    parser.add_argument("--rpm", type=float, default=20.0, help="Test speed in RPM (magnitude)")
    parser.add_argument("--hold", type=float, default=2.0, help="Hold time at each step (s)")
    return parser.parse_args()


def main():
    args = parse_args()

    comm = DxlComm(port=args.port, baudrate=args.baudrate)

    # Resolve IDs list
    ids = [args.id] if args.id is not None else list(dict.fromkeys(args.ids))

    attached = []
    for sid in ids:
        try:
            wheel = Wheel(sid)
            comm.attach(wheel)
            proto = getattr(getattr(wheel, 'adapter', None), 'version', 'unknown')
            print(f"Detected protocol for ID {sid}: {proto}")
            attached.append(wheel)
        except Exception as e:
            print(f"Skipping ID {sid}: {e}")

    if not attached:
        print("No devices attached. Exiting.")
        return

    # Enable torque after all attached
    comm.enable_all_torque()

    try:
        for wheel in attached:
            sid = wheel.servo_id
            for target_rpm in (args.rpm, 0.0, -args.rpm, 0.0):
                print(f"Setting ID {sid} to {target_rpm:.2f} RPM")
                wheel.send_velocity(target_rpm)
                t0 = time.time()
                while time.time() - t0 < args.hold:
                    measured = wheel.get_velocity()
                    print(f"[ID {sid}] Measured: {measured:.2f} RPM")
                    time.sleep(0.3)
            # stop before moving to next
            wheel.send_velocity(0.0)
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        print("Stopping all and disabling torque...")
        for wheel in attached:
            wheel.send_velocity(0.0)
        time.sleep(0.2)
        comm.disable_all_torque()


if __name__ == "__main__":
    main()


