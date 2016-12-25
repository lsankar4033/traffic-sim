# First order model for traffic simulation. The goal is to get something simple working so that I can practice
# plotting the data via Jupyter.

# TODO A quick realism improvement would be to change collision detection to if a car is within 'car_distance' of
# the car in front of it.

# 2 presents a bit of a problem above because if there are a sequence of cars decelerating, won't they crash?
# Additionally, the current model assumes singular acceleration changes. Humans change acceleration
# continuously...
# I should probably put thought into a more complicated model of deceleration.
# TODO - more complicated model of deceleration

# TODO Cars could respond to cars behind them as well

import random

from collections import namedtuple
from itertools import count, islice

Params = namedtuple('Params', 'num_cars v_0 v_jitter v_max t_spacing_0 t_spacing_min acc_neg acc_pos')

# displacements, velocities, accelerations
SimState = namedtuple('SimState', 'xs vs accs')

# Represents the returned simulation data
SimData = namedtuple('SimData', 'xs vs accs collisions')

def combine_data(data1, data2):
    return SimData(data1.xs + data2.xs,
                   data1.vs + data2.vs,
                   data1.accs + data2.accs,
                   data1.collisions + data2.collisions)

# TODO - see how changing this number affects things
TIME_STEP = 1.0

def simulate(params, num_steps):
    state = build_init_state(params.num_cars, params.v_0, params.t_spacing_0)
    data = SimData([], [], [], [])

    for i in range(num_steps):
        (state, new_data) = simulate_step(state,
                                          params.v_jitter,
                                          params.v_max,
                                          params.t_spacing_min,
                                          params.acc_neg,
                                          params.acc_pos)
        data = combine_data(data, new_data)

    return []

def simulate_step(state, v_jitter, v_max, t_spacing_min, acc_neg, acc_pos):
    state = jitter_velocities(state, v_jitter)
    state = set_accelerations(state, v_max, t_spacing_min, acc_neg, acc_pos)
    return step_time(state)

def jitter_velocities(state, v_jitter):
    new_vs = [v + random.gauss(0, v_jitter) for v in state.vs]
    return state._replace(vs = new_vs)

def set_accelerations(state, v_max, t_spacing_min, acc_neg, acc_pos):
    new_accs = []

    for i in range(len(state.vs) - 1):
        t_spacing = (state.xs[i+1] - state.xs[i]) / state.vs[i]
        if t_spacing <= t_spacing_min:
            new_accs.append(acc_neg)
        elif state.vs[i] < v_max:
            new_accs.append(acc_pos)
        else:
            new_accs.append(0)

    # front car acceleration doesn't depend on other cars
    if state.vs[-1] < v_max:
        new_accs.append(acc_pos)
    else:
        new_accs.append(state.accs[-1])

    return state._replace(accs = new_accs)

# dx = vdt + (1/2) * a(dt^2)
def calculate_dx(v, a, dt):
    return v * dt + (a * (dt ** 2)) / 2

# TODO - test
def step_time(state):
    data = SimData([], [], [], [])

    # determine where each car would go without obstacles
    projected_xs = [state.xs[i] + calculate_dx(state.vs[i], state.accs[i], TIME_STEP)
                    for i in range(len(state.xs))]

    # account for collisions
    new_vs = state.vs.copy()
    new_xs = projected_xs.copy()
    for i in reversed(range(len(state.xs) - 1)):
        if new_xs[i] > new_xs[i+1]:
            new_xs[i] = new_xs[i+1]
            new_vs[i] = 0
            data.collisions.append(set([i, i+1]))
    new_state = state._replace(xs = new_xs, vs = new_vs)

    # TODO consolidate collisions

    # record non-collision data
    data.xs = state.xs
    data.vs = state.vs
    data.accs = state.accs

    return (new_state, data)

def build_init_state(num_cars, v_0, t_spacing_0):
    x_spacing_0 = v_0 * t_spacing_0
    xs = list(islice(count(0, x_spacing_0), num_cars))
    return SimState(xs, [v_0] * num_cars, [0] * num_cars)
