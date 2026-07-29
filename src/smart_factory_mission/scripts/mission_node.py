#!/usr/bin/env python3

import rospy

from smart_factory_mission.mission_server import MissionServer


def main():
    rospy.init_node("smart_factory_mission")
    MissionServer()
    rospy.spin()


if __name__ == "__main__":
    main()
