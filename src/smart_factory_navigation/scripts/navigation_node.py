#!/usr/bin/env python3

import rospy

from smart_factory_navigation.action_server import NavigationActionServer


def main():
    rospy.init_node("smart_factory_navigation")
    NavigationActionServer()
    rospy.spin()


if __name__ == "__main__":
    main()
