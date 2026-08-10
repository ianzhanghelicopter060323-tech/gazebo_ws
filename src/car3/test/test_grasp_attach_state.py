#!/usr/bin/env python3

from pathlib import Path
import sys
import unittest
from unittest import mock

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from grasp_attach import GraspAttach  # noqa: E402


class GraspAttachStateTest(unittest.TestCase):
    @staticmethod
    def _node(state="GRASPING", joint=0.76):
        node = GraspAttach.__new__(GraspAttach)
        node.state = state
        node.r_joint_pos = joint
        node.close_threshold = 0.8
        node.release_command_threshold = 1.2
        node._grasp_attempt_period = 0.5
        node._last_grasp_attempt_at = None
        node._do_grasp = mock.Mock()
        node._do_release = mock.Mock()
        return node

    def test_joint_feedback_rebound_does_not_release_attachment(self):
        node = self._node(joint=1.0)

        message = JointState(name=["r_joint"], position=[1.0])
        node._joint_cb(message)

        node._do_release.assert_not_called()

    def test_explicit_open_command_releases_attachment(self):
        node = self._node()

        node._gripper_command_cb(Float64(data=1.5))

        node._do_release.assert_called_once_with(
            "open command 1.500 >= 1.200"
        )

    def test_closed_command_does_not_release_attachment(self):
        node = self._node()

        node._gripper_command_cb(Float64(data=0.76))

        node._do_release.assert_not_called()

    def test_closed_joint_still_attempts_grasp_while_idle(self):
        node = self._node(state="IDLE", joint=0.79)

        node._tick_state()

        node._do_grasp.assert_called_once_with()

    def test_closed_joint_grasp_attempts_are_throttled(self):
        node = self._node(state="IDLE", joint=0.79)

        with mock.patch(
            "grasp_attach.time.monotonic",
            side_effect=[10.0, 10.1, 10.5],
        ):
            node._tick_state()
            node._tick_state()
            node._tick_state()

        self.assertEqual(2, node._do_grasp.call_count)


if __name__ == "__main__":
    unittest.main()
