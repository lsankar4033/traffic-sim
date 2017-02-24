import random

from collections import defaultdict,namedtuple
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

TIME_STEP = 1.0

def simulate(params, num_steps):
    """Main entry point of this module. This method will simulate traffic on a single lane one way road using
    the specified parameters for the specified number of time steps.
    """
    (state, data) = initialize(params.num_cars, params.v_0, params.t_spacing_0)

    for i in range(num_steps):
        (state, new_data) = simulate_step(state,
                                          params.v_jitter,
                                          params.v_max,
                                          params.t_spacing_min,
                                          params.acc_neg,
                                          params.acc_pos)
        data = combine_data(data, new_data)

    return data

def simulate_step(state, v_jitter, v_max, t_spacing_min, acc_neg, acc_pos):
    state = jitter_velocities(state, v_jitter)
    state = set_accelerations(state, v_max, t_spacing_min, acc_neg, acc_pos)
    return step_time(state)

def jitter_velocities(state, v_jitter):
    new_vs = [max(v + random.gauss(0, v_jitter), 0.0) for v in state.vs]
    return state._replace(vs = new_vs)

def set_accelerations(state, v_max, t_spacing_min, acc_neg, acc_pos):
    new_accs = []

    for i in range(len(state.vs) - 1):
        if state.vs[i] == 0 and state.xs[i+1] == state.xs[i]:
            new_accs.append(0)
        elif state.vs[i] == 0:
            new_accs.append(acc_pos)
        elif (state.xs[i+1] - state.xs[i]) / state.vs[i] <= t_spacing_min:
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
def calculate_dx(v, a):
    return max(v * TIME_STEP + (a * (TIME_STEP ** 2)) / 2, 0.0)

def consolidate_collisions(collision_pairs):
    """Turn a list of collision tuples into a list of sets consisting of all 'connected' collisions
    """
    collision_map = {}
    for (i,j) in collision_pairs:
        if i not in collision_map and j not in collision_map:
            cs = set([i, j])
            collision_map[i] = cs
            collision_map[j] = cs

        elif i not in collision_map:
            collision_map[j].add(i)
            collision_map[i] = collision_map[j]

        elif j not in collision_map:
            collision_map[i].add(j)
            collision_map[j] = collision_map[i]

        else:
            collision_map[j].add(i)
            collision_map[i].add(j)

    collisions = []
    seen_indices = set()
    for (i, collision) in collision_map.items():
        if i not in seen_indices:
            collisions.append(collision)
            seen_indices |= collision

    return collisions

def step_time(state):
    """Steps the provided total state by one TIME_STEP and collects all simulation data.

    Return value is a tuple of (new_state, data)
    """
    projected_xs = [state.xs[i] + calculate_dx(state.vs[i], state.accs[i])
                    for i in range(len(state.xs))]
    # velocities should never drop below 0
    projected_vs = [max(state.vs[i] + state.accs[i] * TIME_STEP, 0.0)
                    for i in range(len(state.vs))]

    # account for collisions
    new_vs = projected_vs.copy()
    new_xs = projected_xs.copy()
    new_accs = state.accs.copy()
    collision_pairs = []
    for i in reversed(range(len(state.xs) - 1)):
        if new_xs[i] > new_xs[i+1]:
            new_xs[i] = new_xs[i+1]
            new_vs[i] = 0.0
            new_accs[i] = 0.0

            collision_pairs.append((i, i+1))
    new_state = state._replace(xs = new_xs, vs = new_vs, accs = new_accs)

    # record data
    data = SimData([new_state.xs],
                   [new_state.vs],
                   [new_state.accs],
                   [consolidate_collisions(collision_pairs)])

    return (new_state, data)

def initialize(num_cars, v_0, t_spacing_0):
    x_spacing_0 = v_0 * t_spacing_0
    xs = list(islice(count(0, x_spacing_0), num_cars))
    vs = [v_0] * num_cars
    accs = [0] * num_cars
    state = SimState(xs, vs, accs)
    data = SimData([xs], [vs], [accs], [[]])
    return (state, data)
