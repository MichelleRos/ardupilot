#!/usr/bin/env python3

#device = "/dev/serial/by-id/usb-CubePilot_CubeOrange+_350052000D51323031393637-if00"
device = "tcp:localhost:6789"

from pymavlink import mavutil
import sys

class TestForRSError:
    def __init__(self):
        pass

    def sysid_thismav(self):
        return 9

    def progress(self, msg):
        print(f"tfre: {msg}")

    def send_reboot_command(self):
        self.mav.mav.command_long_send(
            self.sysid_thismav(),
            1,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            1,  # confirmation
            1, # reboot autopilot
            0,
            0,
            0,
            0,
            0,
            0)

    def run(self):
        self.mav = mavutil.mavlink_connection(
            device,
            source_system=7,
            source_component=7,
        )

        rs_errors = 0

        reboots = 0
        while True:
            m = self.mav.recv_match(blocking=True)
            t = m.get_type()
            if t == "ATTITUDE":
                uptime = m.time_boot_ms
                uptime_threshold_minutes = 30
                print(f"{uptime=} {rs_errors=} {reboots=}")
                if (rs_errors == 0 and
                    uptime > uptime_threshold_minutes*60*1000):
                    print("uptime limit!")
                    self.send_reboot_command()
                    reboots += 1
                continue

            if t == 'NAMED_VALUE_FLOAT':
                if m.name != "RSERR":
                    continue
                rs_errors = m.value

#            print(m)

t = TestForRSError()
t.run()
