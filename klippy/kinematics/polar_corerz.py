# Code for handling the kinematics of polar corerz robots
#
# Copyright (C) 2022  Sam Apostel <sam@apostel.be>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
import stepper

class PolarCoreRZKinematics:
    def __init__(self, toolhead, config):
        # Setup steppers
        stepper_bed = stepper.PrinterStepper(config.getsection('stepper_bed'),
                                             units_in_radians=True)
        self.rails = [ stepper.PrinterRail(config.getsection('stepper_a')),
                       stepper.PrinterRail(config.getsection('stepper_b')) ]
        self.rails[0].get_endstops()[0][0].add_stepper(
            self.rails[1].get_steppers()[0])
        self.rails[1].get_endstops()[0][0].add_stepper(
            self.rails[0].get_steppers()[0])
        stepper_bed.setup_itersolve('polar_corerz_stepper_alloc', b'a')
        self.rails[0].setup_itersolve('polar_corerz_stepper_alloc', b'+')
        self.rails[1].setup_itersolve('polar_corerz_stepper_alloc', b'-')
        self.steppers = [stepper_bed] + [ s for r in self.rails
                                          for s in r.get_steppers() ]
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        config.get_printer().register_event_handler("stepper_enable:motor_off",
                                                    self._motor_off)
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', max_velocity, above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat(
            'max_z_accel', max_accel, above=0., maxval=max_accel)
        self.limit_z = (1.0, -1.0)
        self.limit_r = -1.
        max_xy = self.rails[0].get_range()[1]
        min_z, max_z = self.rails[1].get_range()
        self.axes_min = toolhead.Coord(-max_xy, -max_xy, min_z, 0.)
        self.axes_max = toolhead.Coord(max_xy, max_xy, max_z, 0.)
    def get_steppers(self):
        return list(self.steppers)
    def calc_position(self, stepper_positions):
        rz_pos = [stepper_positions[rail.get_name()] for rail in self.rails]
        bed_angle = stepper_positions[self.steppers[0].get_name()]
        r = 0.5 * (pos[0] + pos[1])
        x = math.cos(bed_angle) * r
        y = math.sin(bed_angle) * r
        z = 0.5 * (pos[0] - pos[1])
        return [x, y, z]
    def set_position(self, newpos, homing_axes):
        self.steppers[0].set_position(newpos)
        for rail in self.rails:
            rail.set_position(newpos)
        if 2 in homing_axes:
            self.limit_z = self.rails[1].get_range()
        if 0 in homing_axes and 1 in homing_axes:
            self.limit_r = self.rails[0].get_range()[1]**2
    def note_z_not_homed(self):
        # Helper for Safe Z Home
        self.limits[1] = (1.0, -1.0)
    def _home_axis(self, homing_state, axis, rail):
        # Determine movement
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        if axis == 0:
            homepos[1] = 0.
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        # Perform homing
        homing_state.home_rails([rail], forcepos, homepos)
    def home(self, homing_state):
        # Always home XY together
        homing_axes = homing_state.get_axes()
        home_r = 0 in homing_axes or 1 in homing_axes
        home_z = 2 in homing_axes
        updated_axes = []
        if home_r:
            updated_axes = [0, 1]
        if home_z:
            updated_axes.append(2)
        homing_state.set_axes(updated_axes)
        # Do actual homing
        if home_r:
            self._home_axis(homing_state, 0, self.rails[0])
        if home_z:
            self._home_axis(homing_state, 2, self.rails[1])
    def _motor_off(self, print_time):
        self.limit_z = (1.0, -1.0)
        self.limit_r = -1.
    def check_move(self, move):
        xpos, ypos, zpos = move.end_pos[:3]
        r = xpos**2 + ypos**2
        if r > self.limit_r:
            if self.limit_r < 0.:
                raise move.move_error("Must home axis first")
            raise move.move_error()
        if zpos < self.limit_z[0] or zpos > self.limit_z[1]:
            if self.limit_z[0] > self.limit_z[1]:
                raise move.move_error("Must home axis first")
            raise move.move_error()
    def get_status(self, eventtime):
        r_home = "xy" if self.limit_r >= 0. else ""
        z_home = "z" if self.limit_z[0] <= self.limit_z[1] else ""
        return {
            'homed_axes': r_home + z_home,
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return PolarCoreRZKinematics(toolhead, config)
