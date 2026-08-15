#!/usr/bin/env python3

"""Gazebo world-log playback entry point, plus unit tests.

Run this file directly with a log path to open the recorded world paused in
Gazebo. Pass ``--run-immediately`` to start its timeline without clicking
Gazebo's play button. Unit-test discovery still imports the test cases below
normally.
"""

import sys
import types
import unittest
from unittest import mock

import play_gazebo_world_log as playback


class CommandLineTest(unittest.TestCase):
    def test_log_file_is_positional_argument(self):
        path = playback.Path("/tmp/gazebo_world_state.log")

        args = playback.parse_args([str(path)])

        self.assertEqual(args.recording, path)


class RenderingEnvironmentTest(unittest.TestCase):
    @mock.patch.object(playback.Path, "is_file", return_value=True)
    def test_wsl_defaults_to_nvidia_d3d12(self, _is_file):
        args = types.SimpleNamespace(
            software_rendering=False,
            gpu_adapter="NVIDIA",
        )
        with mock.patch.dict(playback.os.environ, {}, clear=True):
            environment, mode = playback.rendering_environment(args)

        self.assertEqual(environment["LIBGL_ALWAYS_SOFTWARE"], "0")
        self.assertEqual(environment["GALLIUM_DRIVER"], "d3d12")
        self.assertEqual(
            environment["MESA_D3D12_DEFAULT_ADAPTER_NAME"], "NVIDIA"
        )
        self.assertIn("hardware", mode)

    def test_software_fallback_forces_llvmpipe(self):
        args = types.SimpleNamespace(
            software_rendering=True,
            gpu_adapter="NVIDIA",
        )
        with mock.patch.dict(
            playback.os.environ,
            {"MESA_D3D12_DEFAULT_ADAPTER_NAME": "NVIDIA"},
            clear=True,
        ):
            environment, mode = playback.rendering_environment(args)

        self.assertEqual(environment["LIBGL_ALWAYS_SOFTWARE"], "1")
        self.assertEqual(environment["GALLIUM_DRIVER"], "llvmpipe")
        self.assertNotIn("MESA_D3D12_DEFAULT_ADAPTER_NAME", environment)
        self.assertIn("software", mode)


if __name__ == "__main__":
    sys.exit(playback.main(sys.argv[1:]))
