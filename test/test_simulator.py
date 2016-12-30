import unittest
from traffic_sim.simulator import *

# TODO - use a test framework that allows further nesting of test cases

class TestInitialize(unittest.TestCase):
    def test_simple(self):
        self.assertEqual(initialize(2, 1, 1),
                         (SimState([0, 1], [1, 1], [0, 0]),
                          SimData([[0, 1]], [[1, 1]], [[0, 0]], [[]])))

class TestAcceleration(unittest.TestCase):
    def test_no_accelerations(self):
        state = SimState([0, 10], [1, 1], [0, 0])
        self.assertEqual(set_accelerations(state, 1, 5, -10, 10), state)

    def test_deceleration(self):
        state = SimState([0, 2], [10, 20], [0, 0])
        self.assertEqual(set_accelerations(state, 20, 1, -10, 10),
                         state._replace(accs = [-10, 0]))

    def test_acceleration(self):
        state = SimState([0, 100], [1, 1], [0, 0])
        self.assertEqual(set_accelerations(state, 20, 1, -10, 10),
                         state._replace(accs = [10, 10]))

def same_collisions(s1, s2):
    if len(s1) is not len(s2):
        return False

    for i in s1:
        matching = [c for c in s2 if c == i]
        if [i] != matching:
            return False

    for i in s2:
        matching = [c for c in s1 if c == i]
        if [i] != matching:
            return False

    return True

class TestConsolidateCollisions(unittest.TestCase):
    def test_disjoint(self):
        collision_pairs = [(1, 2), (3, 4), (5, 6)]
        self.assertTrue(
            same_collisions(consolidate_collisions(collision_pairs),
                            [{1, 2}, {3, 4}, {5, 6}]))

    def test_intersecting(self):
        collision_pairs = [(1, 2), (2, 3), (4, 5)]
        self.assertTrue(
            same_collisions(consolidate_collisions(collision_pairs),
                            [{1, 2, 3}, {4, 5}]))

# NOTE - these methods are dependent on our kinematics model
def compute_new_xs(xs, vs, accs):
    return [xs[i] + vs[i] * TIME_STEP + (accs[i] * (TIME_STEP ** 2)) / 2
            for i in range(len(xs))]

def compute_new_vs(vs, accs):
    return [vs[i] + accs[i] * TIME_STEP
            for i in range(len(vs))]

# TODO - make it easier to specify these test fixtures
class TestStepTime(unittest.TestCase):
    def test_no_collisions(self):
        xs = [0, 5]
        vs = [1, 1]
        accs = [2, 2]
        new_xs = compute_new_xs(xs, vs, accs)
        new_vs = compute_new_vs(vs, accs)
        new_accs = accs

        self.assertEqual(step_time(SimState(xs, vs, accs)),
                         (SimState(new_xs, new_vs, accs),
                          SimData([new_xs], [new_vs], [new_accs], [[]])))

    def test_single_collision(self):
        xs = [0, 1]
        vs = [1, 0]
        accs = [2, 0]
        new_xs = compute_new_xs(xs, vs, accs)
        new_xs[0] = 1.0
        new_vs = compute_new_vs(vs, accs)
        new_vs[0] = 0.0
        new_accs = accs.copy()
        new_accs[0] = 0.0

        self.assertEqual(step_time(SimState(xs, vs, accs)),
                         (SimState(new_xs, new_vs, new_accs),
                          SimData([new_xs], [new_vs], [new_accs], [[set([0, 1])]])))

    def test_multi_collision(self):
        xs = [0, 1, 1.5]
        vs = [2, 1, 0]
        accs = [0, 0, 0]
        new_xs = compute_new_xs(xs, vs, accs)
        new_xs[0] = 1.5
        new_xs[1] = 1.5
        new_vs = compute_new_vs(vs, accs)
        new_vs[0] = 0.0
        new_vs[1] = 0.0
        new_accs = accs.copy()
        new_accs[0] = 0.0
        new_accs[1] = 0.0

        self.assertEqual(step_time(SimState(xs, vs, accs)),
                         (SimState(new_xs, new_vs, accs),
                          SimData([new_xs], [new_vs], [new_accs], [[set([0, 1, 2])]])))

    def test_nonnegative_velocities(self):
        (state, _) = step_time(SimState([0], [1], [-10]))
        self.assertEqual(state.vs[0], 0.0)
