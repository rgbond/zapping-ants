#!/usr/bin/python

# Copyright 2016 Robert Bond
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
# http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Reads commands from a shared memory location and sends them to 
# the EIBOT board

import os
import sys
import signal
import mmap
import argparse
import time
from eibot import motor

class clicker(object):
    def __init__(self, coms, motor, verbose=0):
        self.coms = coms
        self.motor = motor
        self.verbose = verbose
        self.laser_on = False
        self.switch_sleep = 0.075
        self.button_sleep = 0.5

    def unpack(self, s, n):
        v = 0
        shift = 0
        for i in range(s, s + n):
            v += ord(self.coms[i]) << shift
            shift += 8
        return (v, s + n)

    ### CHANGE ME IF THE STRUCT CHANGES ###
    def get_ok(self):
        return ord(self.coms[12])
        
    ### CHANGE ME IF THE STRUCT CHANGES ###
    def clear_ok(self):
        self.coms[12] = chr(0)
        
    def unpack_coms(self):
        (magic, p) = self.unpack(0, 4)
        (ms, p) = self.unpack(p, 2)
        (m1_steps, p) = self.unpack(p, 2)
        (m2_steps, p) = self.unpack(p, 2)
        (flags, p) = self.unpack(p, 2)
        (ok, p) = self.unpack(p, 2)
        if (flags & 0x08):
            m1_steps = -m1_steps
        if (flags & 0x10):
            m2_steps = -m2_steps
        if self.verbose > 1:
            print "unpack_coms", hex(magic), hex(ms), hex(m1_steps),
            print hex(m2_steps), hex(flags), hex(ok) 
        return (magic, ms, m1_steps, m2_steps, flags, ok)

    def button_pressed(self, sw):
        threshold = 1024 / 3
        # check for pulled to 0
        if sw < threshold:
            return True
        # Otherwise down or pulled high
        return False 

    def switch_up(self, sw):
        threshold = 1024 / 3
        # check for pulled to 0
        if sw < threshold:
            return True
        # Otherwise down or pulled high
        return False 

    def switch_down(self, sw):
        threshold = 1024 / 3
        # check for pulled high
        if sw > threshold * 2: 
            return False
        # check for divided by 2
        if sw > threshold:
            return True
        # Must be up
        return False 

    def stopnow(self):
        self.motor.set_laser(False)
        self.motor.motors_off()
        exit(0)

    def run(self):
        (magic, ms, m1_steps, m2_steps, flags, ok) = self.unpack_coms()
        if magic != 0x12344321:
            print "Shared mem file not setup"
            exit(1)
        running = True
        button_state = 0
        while running:
            # shared mem cmds
            if self.get_ok() == 1:
                (magic, ms, m1_steps, m2_steps, flags, ok) = self.unpack_coms()
                if self.verbose > 0:
                    print "run", hex(magic), hex(ms), hex(m1_steps),
                    print hex(m2_steps), hex(flags), hex(ok) 
                if flags & 0x01:
                    if not self.laser_on:
                        self.motor.set_laser(True)
                        self.laser_on = True
                else:
                    self.motor.set_laser(False)
                    self.laser_on = False
                if not (flags & 0x02 == 0x02):
                    self.motor.motors_off()
                if flags & 0x04:
                    self.motor.set_laser(False)
                    self.laser_on = False
                    self.motor.motors_off()
                    running = False
                if (m1_steps != 0) or (m2_steps != 0):
                    motor.ramp2(ms, m1_steps, m2_steps)
                self.clear_ok()
            # Switches
            # (sw1_str, sw2_str, button_str) = (1023, 1023, 1023)
            (sw1_str, sw2_str, button_str) = self.motor.get_analog()
            sw1 = int(sw1_str)
            sw2 = int(sw2_str)
            button = int(button_str)
            if self.switch_up(sw1):
                self.motor.m1_move_out(1)
                time.sleep(self.switch_sleep)
            elif self.switch_down(sw1):
                self.motor.m1_move_in(1)
                time.sleep(self.switch_sleep)
            if self.switch_up(sw2):
                self.motor.m2_move_out(1)
                time.sleep(self.switch_sleep)
            elif self.switch_down(sw2):
                self.motor.m2_move_in(1)
                time.sleep(self.switch_sleep)
            if self.button_pressed(button):
                self.motor.set_laser(False)
                self.motor.motors_off()
                if button_state == 0:
                    time.sleep(3)
                    button_state = 1
                elif button_state == 1:
                    os.system("/sbin/shutdown -h now")
                    button_state = 0
            else:
                if button_state == 1:
                    os.system("/etc/init.d/ants restart")
                    button_state = 0

def sig_term_handler(signum, frame):
    global click
    click.stopnow()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--verbose', dest='verbose', type=int,
                        help='set verbose mode')
    args = parser.parse_args()

    print "clicker.py starting"
    sys.stdout.flush()
    while True:
        try:
            f = open("/home/rgb/shmem", "r+b")
            break
        except:
            time.sleep(1)
            print "retrying /home/rgb/shmem" 
            sys.stdout.flush()

    coms = mmap.mmap(f.fileno(), 0)
    # motor = motor(tty_name='/dev/ttyACM0', verbose=args.verbose)
    try:
        motor = motor(tty_name='/dev/ttyACM0')
        click = clicker(coms, motor, verbose=args.verbose)
        motor.set_laser(True)
        time.sleep(1)
        motor.set_laser(False)
        click.run()
        signal.signal(signal.SIGTERM, sig_term_handler)
    except:
        print "Top level exception and restart..."
        sys.stdout.flush()
        sys.stderr.flush()
        os.system("/etc/init.d/ants restart")
