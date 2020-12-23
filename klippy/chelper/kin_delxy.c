// CoreXY kinematics stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord

struct delxy_stepper{
    struct stepper_kinematics sk;
    double arm_2, tower_pos;
}

static double
delxy_stepper_a_calc_position(struct stepper_kinematics *sk, struct move *m
                                , double move_time)
{
    struct delxy_stepper *ds = container_of(sk, struct delxy_stepper, sk);
    struct coord c = move_get_coord(m, move_time);

    double dx = ds->tower_pos - c.x;

    return sqrt(ds->arm_2 - dx * dx) + c.y;
}

struct stepper_kinematics * __visible
delxy_stepper_alloc(double arm_2, double tower_pos)
{
    struct delxy_stepper *ds = malloc(sizeof(*ds));
    memset(ds, 0, sizeof(*ds));

    ds->arm_2 = arm_2;
    ds->tower_pos = tower_pos;
    ds->sk.calc_position_cb = delxy_stepper_a_calc_position;
    ds->sk.active_flags = AF_X | AF_Y;

    return &ds->sk;
}
