'''
Fly Blimp in SITL

AP_FLAKE8_CLEAN
'''

from __future__ import print_function
import os
import shutil

from pymavlink import mavutil
from pymavlink import mavextra

from common import AutoTest

#Autotest command: python3 Tools/autotest/autotest.py test.Blimp
#with map: python3 Tools/autotest/autotest.py test.Blimp --map --speedup=2

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
SITL_START_LOCATION = mavutil.location(-35.362938, 149.165085, 584, 0)

# Flight mode switch positions are set-up in blimp.parm to be
#   switch 1 = Land
#   switch 2 = Manual
#   switch 3 = Velocity
#   switch 4 = Loiter
#   switch 5 = Manual
#   switch 6 = Manual


class AutoTestBlimp(AutoTest):
    @staticmethod
    def get_not_armable_mode_list():
        return []

    @staticmethod
    def get_not_disarmed_settable_modes_list():
        return []

    @staticmethod
    def get_no_position_not_settable_modes_list():
        return []

    @staticmethod
    def get_position_armable_modes_list():
        return []

    @staticmethod
    def get_normal_armable_modes_list():
        return []

    def log_name(self):
        return "Blimp"

    def default_mode(self):
        return "MANUAL"

    def test_filepath(self):
        return os.path.realpath(__file__)

    def default_speedup(self):
        return 100

    def set_current_test_name(self, name):
        self.current_test_name_directory = "Blimp_Tests/" + name + "/"

    def sitl_start_location(self):
        return SITL_START_LOCATION

    def sitl_streamrate(self):
        return 5

    def vehicleinfo_key(self):
        return 'Blimp'

    def default_frame(self):
        return "Blimp"

    def apply_defaultfile_parameters(self):
        # Blimp passes in a defaults_filepath in place of applying
        # parameters afterwards.
        pass

    def defaults_filepath(self):
        return self.model_defaults_filepath(self.frame)

    def wait_disarmed_default_wait_time(self):
        return 120

    def close(self):
        super(AutoTestBlimp, self).close()

        # [2014/05/07] FC Because I'm doing a cross machine build
        # (source is on host, build is on guest VM) I cannot hard link
        # This flag tells me that I need to copy the data out
        if self.copy_tlog:
            shutil.copy(self.logfile, self.buildlog)

    def is_blimp(self):
        return True

    def get_stick_arming_channel(self):
        return int(self.get_parameter("RCMAP_YAW"))

    def get_disarm_delay(self):
        return int(self.get_parameter("DISARM_DELAY"))

    def set_autodisarm_delay(self, delay):
        self.set_parameter("DISARM_DELAY", delay)

    def FlyManual(self):
        '''test manual mode'''
        self.change_mode('MANUAL')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        # make sure we don't drift:
        start = self.mav.location()
        self.set_rc(2, 2000)
        self.wait_distance_to_location(start, 2, 10, timeout=40)
        self.disarm_vehicle()

    def FlyLoiter(self):
        '''test loiter mode''' #10 sec

        self.change_mode('LOITER')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        # make sure we don't drift:
        bl = self.mav.location()
        tl = self.offset_location_ne(location=bl, metres_north=2, metres_east=0)
        tr = self.offset_location_ne(location=bl, metres_north=2, metres_east=2)
        br = self.offset_location_ne(location=bl, metres_north=0, metres_east=2)

        print("Locations are:")
        print("bottom left  ", bl.lat, bl.lng)
        print("top left     ", tl.lat, tl.lng)
        print("top right    ", tr.lat, tr.lng)
        print("bottom right ", br.lat, br.lng)

        if self.mavproxy is not None:
            self.mavproxy.send(f"map icon {bl.lat} {bl.lng} barrell\n")
            self.mavproxy.send(f"map icon {tl.lat} {tl.lng} barrell\n")
            self.mavproxy.send(f"map icon {tr.lat} {tr.lng} barrell\n")
            self.mavproxy.send(f"map icon {br.lat} {br.lng} barrell\n")

        acc = 0.1
        tim = 30

        #Temp bit to ensure it really is pointing North.
        self.set_rc(4, 1600)
        self.wait_heading(0, accuracy=2, timeout=40)
        self.set_rc(4, 1500)

        self.set_rc(2, 2000)
        self.wait_distance_to_location(tl, 0, acc, timeout=tim)
        self.set_rc(2, 1500)
        self.set_rc(1, 2000)
        self.wait_distance_to_location(tr, 0, acc, timeout=tim)
        self.set_rc(1, 1500)
        self.set_rc(2, 1000)
        self.wait_distance_to_location(br, 0, acc, timeout=tim)
        self.set_rc(2, 1500)
        self.set_rc(1, 1000)
        self.wait_distance_to_location(bl, 0, acc, timeout=tim)

        #self.wait_heading(170)
        #self.delay_sim_time(10)

        self.disarm_vehicle()

    def tests(self):
        '''return list of all tests'''
        # ret = super(AutoTestBlimp, self).tests()
        ret = []
        ret.extend([
            # self.FlyManual,
            self.FlyLoiter,
        ])
        return ret

    def disabled_tests(self):
        return {
        }
