#! /usr/bin/python

import rospy

from nbv_underwater.bot import Bot

if __name__ == "__main__":
    rospy.init_node("bluerov_nbv_planner", anonymous=False)

    BlueROV = Bot()

    BlueROV.explore()
