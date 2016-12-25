import unittest
from traffic_sim.simulator import *

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
