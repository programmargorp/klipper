# Code for handling the kinematics of delxy robots

# Axis laveling is ABZ and follows delta naming convention
# The left axis is A and the right axis is B.

import logging, math
import stepper, homing

# Slow X moves once the tower to x ratio exceeds SLOW_RATIO
SLOW_RATIO = 1.41

class DelXYKinematics:
    def __init__(self, toolhead, config):
        stepper_configs = [config.getsection('stepper_' + a) for a in 'abz']
        self.rails = [ stepper.PrinterRail(stepper_configs[0],
                            need_position_minmax = False),
                       stepper.PrinterRail(stepper_configs[1],
                            need_position_minmax = False),
                       stepper.PrinterRail(stepper_configs[2]) ]

        # Set the need homing flag
        self.needs_homing = True

        # Get arm length and tower position parameters
        self.arm_lengths = [ stepper_configs[0].getfloat('arm_length'),
                        stepper_configs[1].getfloat('arm_length')]
        self.arm2 = [arm**2 for arm in self.arm_lengths]
        self.tower_dists = [
        stepper_configs[0].getfloat('tower_pos', minval=-self.arm_lengths[0]),
        stepper_configs[1].getfloat('tower_pos', maxval=self.arm_lengths[1])
        ]
        self.height_offset = config.getfloat('delxy_height', above=0.)
        self.abs_endstop_locations = [rail.get_homing_info().position_endstop
                                    for rail in self.rails]
        self.abs_endstop_locations[0] = self.abs_endstop_locations[0] + \
            math.sqrt(self.arm2[0] - self.tower_dists[0] ** 2) + \
                self.height_offset
        self.abs_endstop_locations[1] = self.abs_endstop_locations[1] + \
            math.sqrt(self.arm2[1] - self.tower_dists[1] ** 2) + \
                self.height_offset
        self.max_bed_y = config.getfloat('delxy_max_y', above=0.)
        self.max_z = self.rails[2].get_homing_info().position_endstop
        self.min_z = config.getfloat('delxy_min_z', 0, maxval=self.max_z)

        # Set up homing position
        self.home_position = tuple(self._calc_delxy_cartesian_pos(
                self.abs_endstop_locations))

        # Allocate stepper solvers
        self.rails[0].setup_itersolve(
            'delxy_stepper_alloc', self.arm2[0], self.tower_dists[0])
        self.rails[1].setup_itersolve(
            'delxy_stepper_alloc', self.arm2[1], self.tower_dists[1])
        self.rails[2].setup_itersolve('cartesian_stepper_alloc', 'z')
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)

        config.get_printer().register_event_handler(
            "stepper_enable:motor_off", self._motor_off)

        # Setup boundary checks
        self.max_velocity, self.max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', self.max_velocity, above=0.,
            maxval=self.max_velocity)
        self.max_z_accel = config.getfloat(
            'max_z_accel', self.max_accel, above=0.,
            maxval=self.max_accel)
        self.limits = [(1.0, -1.0)] * 3

        # Setup stepper max halt velocity
        max_halt_velocity = toolhead.get_max_axis_halt()
        max_xy_halt_velocity = max_halt_velocity * SLOW_RATIO
        max_xy_accel = self.max_accel * SLOW_RATIO
        self.rails[0].set_max_jerk(max_xy_halt_velocity, max_xy_accel)
        self.rails[1].set_max_jerk(max_xy_halt_velocity, max_xy_accel)
        self.rails[2].set_max_jerk(
            min(self.max_z_velocity, max_halt_velocity), self.max_z_accel)

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def calc_tag_position(self):
        spos = [rail.get_tag_position for rail in self.rails]
        return self._calc_delxy_cartesian_pos(spos)

    def set_position(self, newpos, homing_axes):
        for rail in self.rails:
            rail.set_position(newpos)
        if tuple(homing_axes) == (0, 1, 2):
            self.needs_homing = False

    def home(self, homing_state):
        # Home all axes simultaneously
        homing_state.set_axes([0, 1, 2])
        forcepos = list(self.home_position)
        # Force position to (0, 0, 0) for homing
        forcepos[0] = 0
        forcepos[1] = 0
        forcepos[2] = 0
        homing_state.home_rails(self.rails, forcepos, self.home_position)

    def check_move(self, move):
        end_pos = move.end_pos
        if self.needs_homing:
            raise move.move_error(end_pos, "Must home first")
        # Check X position limit
        if end_pos[0] < self.tower_dists[0] or end_pos[0] > self.tower_dists[1]:
            raise move.move_error(end_pos)
        if end_pos[1] > self.max_bed_y:
            if end_pos[:2] != self.home_position[:2]:
                    raise move.move_error(end_pos)
        # Check Z safety
        if end_pos[2] < self.min_z:
            raise move.move_error(end_pos)

    def get_status(self, eventtime):
        return {'homed_axes': '' if self.needs_homing else 'xyz'}

    def _motor_off(self, print_time):
        self.need_home = True

    def _calc_delxy_cartesian_pos(self, spos):
        d = math.sqrt((self.tower_dists[1] - self.tower_dists[0]) ** 2 \
                        + (spos[1] - spos[0]) ** 2)
        a = (self.arm2[0] - self.arm2[1] + d ** 2) / (2 * d)
        h = math.sqrt(self.arm2[0] - a ** 2)
        x2 = self.tower_dists[0]
        x2 = x2 + a * (self.tower_dists[1] - self.tower_dists[0]) / d
        y2 = spos[0] + a * (spos[1] - spos[0]) / d
        x3 = [x2 + h * (spos[1] - spos[0]) / d,
              x2 - h * (spos[1] - spos[0]) / d]
        y3 = [y2 - h * (self.tower_dists[1] - self.tower_dists[0]) / d,
              y2 - h * (self.tower_dists[1] - self.tower_dists[0]) / d]
        # Pick the set [x3[0], y3[0]] or [x3[1], y3[1]] which gives a y
        # coordinate < both spos[0] and spos[1]
        if(y3[0] < spos[0] and y3[0] < spos[1]):
            return [x3[0], y3[0], spos[2]]
        else:
            return [x3[1], y3[1], spos[2]]

def load_kinematics(toolhead, config):
    return DelXYKinematics(toolhead, config)