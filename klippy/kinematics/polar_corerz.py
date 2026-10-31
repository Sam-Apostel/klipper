# Code for handling the kinematics of polar CoreRZ printers
#
# A rotating bed (stepper_bed, in radians) plus an arm whose radius (R)
# and height (Z) are driven together by two motors in an H-bot / CoreXZ
# belt layout:
#   stepper_a = r + z   (carries the R endstop)
#   stepper_b = r - z   (carries the Z endstop)
#
# Homing R uses a temporary linear (CoreXZ) mapping so the homing move
# never passes through the bed centre, where polar kinematics are
# singular and the bed would be commanded to spin 180 degrees instantly.
#
# Copyright (C) 2022-2026  Sam Apostel <sam@apostel.be>
# Based on polar.py, Copyright (C) 2018-2021  Kevin O'Connor
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
import stepper, chelper


def distance_to_center(p1, p2):
    ab_x = p2[0]-p1[0]
    ab_y = p2[1]-p1[1]
    ap_x = -p1[0]
    ap_y = -p1[1]
    ab_ap_dot_product = ab_x * ap_x + ab_y * ap_y
    ab_length = math.sqrt(ab_x ** 2 + ab_y ** 2)
    if ab_ap_dot_product <= 0:
        dist = math.sqrt(ap_x ** 2 + ap_y ** 2)
    elif ab_ap_dot_product >= ab_length ** 2:
        dist = math.sqrt(p2[0] ** 2 + p2[1] ** 2)
    else:
        dist = abs(ab_x * ap_y - ab_y * ap_x) / ab_length
    return dist


class PolarCoreRZKinematics:
    def __init__(self, toolhead, config):
        self.toolhead = toolhead
        # Setup steppers
        stepper_bed = stepper.PrinterStepper(config.getsection('stepper_bed'),
                                             units_in_radians=True)
        rail_a = stepper.LookupMultiRail(config.getsection('stepper_a'))
        rail_b = stepper.LookupMultiRail(config.getsection('stepper_b'))
        # Both motors move on every R and every Z move, so each endstop
        # must stop both motors
        for s in rail_b.get_steppers():
            rail_a.get_endstops()[0][0].add_stepper(s)
        for s in rail_a.get_steppers():
            rail_b.get_endstops()[0][0].add_stepper(s)
        stepper_bed.setup_itersolve('polar_corerz_stepper_alloc', b'a')
        rail_a.setup_itersolve('polar_corerz_stepper_alloc', b'+')
        rail_b.setup_itersolve('polar_corerz_stepper_alloc', b'-')
        self.stepper_bed = stepper_bed
        self.rails = [rail_a, rail_b]
        self.steppers = [stepper_bed] + [s for r in self.rails
                                         for s in r.get_steppers()]
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
        # Linear kinematics used while homing R (see _home_r)
        ffi_main, ffi_lib = chelper.get_ffi()
        self.homing_sks = {
            stepper_bed.get_name(): ffi_main.gc(
                ffi_lib.cartesian_stepper_alloc(b'y'), ffi_lib.free)}
        for rail, t in ((rail_a, b'+'), (rail_b, b'-')):
            for s in rail.get_steppers():
                self.homing_sks[s.get_name()] = ffi_main.gc(
                    ffi_lib.corexz_stepper_alloc(t), ffi_lib.free)
        # Setup boundary checks
        self.max_velocity, self.max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', self.max_velocity, above=0.,
            maxval=self.max_velocity)
        self.max_z_accel = config.getfloat(
            'max_z_accel', self.max_accel, above=0., maxval=self.max_accel)
        self.v_rad_max = config.getfloat(
            'max_angular_velocity', above=0., default=0)
        self.limit_z = (1.0, -1.0)
        self.limit_xy2 = -1.
        max_xy = self.rails[0].get_range()[1]
        min_z, max_z = self.rails[1].get_range()
        self.axes_min = toolhead.Coord((-max_xy, -max_xy, min_z))
        self.axes_max = toolhead.Coord((max_xy, max_xy, max_z))
    def get_steppers(self):
        return list(self.steppers)
    def calc_position(self, stepper_positions):
        bed_angle = stepper_positions[self.stepper_bed.get_name()]
        a = stepper_positions[self.rails[0].get_name()]
        b = stepper_positions[self.rails[1].get_name()]
        r = 0.5 * (a + b)
        z = 0.5 * (a - b)
        # While homing R the bed stepper runs a linear mapping that always
        # reads 0, so this also yields x = r (which may be negative), y = 0
        return [math.cos(bed_angle) * r, math.sin(bed_angle) * r, z]
    def set_position(self, newpos, homing_axes):
        for s in self.steppers:
            s.set_position(newpos)
        if "z" in homing_axes:
            self.limit_z = self.rails[1].get_range()
        if "x" in homing_axes and "y" in homing_axes:
            self.limit_xy2 = self.rails[0].get_range()[1]**2
    def clear_homing_state(self, clear_axes):
        if "x" in clear_axes or "y" in clear_axes:
            # X and Y cannot be cleared separately
            self.limit_xy2 = -1.
        if "z" in clear_axes:
            self.limit_z = (1.0, -1.0)
    def _homing_moves(self, rail, axis):
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        return forcepos, homepos
    def _swap_kinematics(self, sks):
        prev = {}
        for s in self.steppers:
            prev[s.get_name()] = s.set_stepper_kinematics(sks[s.get_name()])
        return prev
    def _home_r(self, homing_state):
        forcepos, homepos = self._homing_moves(self.rails[0], 0)
        forcepos[1] = homepos[1] = 0.
        # Home R along a straight line with linear kinematics: the arm
        # may start anywhere (even past the bed centre) and the bed
        # stepper is held still for the whole move
        self.toolhead.flush_step_generation()
        prev = self._swap_kinematics(self.homing_sks)
        try:
            homing_state.home_rails([self.rails[0]], forcepos, homepos)
        finally:
            self.toolhead.flush_step_generation()
            self._swap_kinematics(prev)
            # Re-sync stepper positions with the restored polar mapping
            self.toolhead.set_position(self.toolhead.get_position())
    def _home_z(self, homing_state):
        # Z moves keep R constant, so polar kinematics are safe here
        forcepos, homepos = self._homing_moves(self.rails[1], 2)
        homing_state.home_rails([self.rails[1]], forcepos, homepos)
    def home(self, homing_state):
        # Always home XY (R) together, and R before Z
        homing_axes = homing_state.get_axes()
        home_r = 0 in homing_axes or 1 in homing_axes
        home_z = 2 in homing_axes
        updated_axes = []
        if home_r:
            updated_axes = [0, 1]
        if home_z:
            updated_axes.append(2)
        homing_state.set_axes(updated_axes)
        if home_r:
            self._home_r(homing_state)
        if home_z:
            self._home_z(homing_state)
    def check_move(self, move):
        end_pos = move.end_pos
        xy2 = end_pos[0]**2 + end_pos[1]**2
        # Only check R when the move changes X/Y, so Z can be homed
        # and moved before R (as on cartesian printers)
        if (move.axes_d[0] or move.axes_d[1]) and xy2 > self.limit_xy2:
            if self.limit_xy2 < 0.:
                raise move.move_error("Must home axis first")
            raise move.move_error()
        if move.axes_d[2]:
            if end_pos[2] < self.limit_z[0] or end_pos[2] > self.limit_z[1]:
                if self.limit_z[0] > self.limit_z[1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()
            # Move with Z - update velocity and accel for slower Z axis
            z_ratio = move.move_d / abs(move.axes_d[2])
            move.limit_speed(self.max_z_velocity * z_ratio,
                             self.max_z_accel * z_ratio)
        # Slow down near center, where the bed has to spin fastest
        if (move.axes_d[0] or move.axes_d[1]) and self.v_rad_max:
            min_dist = distance_to_center(move.start_pos[0:2],
                                          move.end_pos[0:2])
            if min_dist == 0:
                return
            v_angular = math.sqrt(move.max_cruise_v2) / min_dist
            if self.v_rad_max < v_angular:
                scale_radius = self.v_rad_max / v_angular
                move.limit_speed(self.max_velocity * scale_radius,
                                 self.max_accel * scale_radius)
    def get_status(self, eventtime):
        xy_home = "xy" if self.limit_xy2 >= 0. else ""
        z_home = "z" if self.limit_z[0] <= self.limit_z[1] else ""
        return {
            'homed_axes': xy_home + z_home,
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return PolarCoreRZKinematics(toolhead, config)
