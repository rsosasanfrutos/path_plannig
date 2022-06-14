from path_planing import Robot, RelativePosition
import numpy as np
import unittest

class TestPathPlan(unittest.TestCase):

    def test_segment_relative_pose(self):
        rob = np.array((1, 1, 1))
        a = np.array((2, 2, 2))
        b = np.array((0, 0, 0))
        relative_pose = Robot.get_relative_pose_to_segment(rob, a, b)
        self.assertEqual(relative_pose, RelativePosition.BETWEEN)

        rob = np.array((-5, -5, -5))
        a = np.array((2, 2, 2))
        b = np.array((0, 0, 0))
        relative_pose = Robot.get_relative_pose_to_segment(rob, a, b)
        self.assertEqual(relative_pose, RelativePosition.BEFORE)

        rob = np.array((10, 10, 10))
        a = np.array((2, 2, 2))
        b = np.array((0, 0, 0))
        relative_pose = Robot.get_relative_pose_to_segment(rob, a, b)
        self.assertEqual(relative_pose, RelativePosition.AFTER)

if __name__ == '__main__':
    unittest.main()