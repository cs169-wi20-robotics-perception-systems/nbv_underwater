import rospy
import cv2
import math

from sensor_msgs.msg import Image
from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import OverrideRCIn
from ORB_SLAM2.msg import Points
from cv_bridge import CvBridge, CvBridgeError

import planner


class Bot:
    def __init__(self):
        self.rate = rospy.Rate(10)
        self.bridge = CvBridge()

        self.img_width_half = 480
        self.img_height_half = 270

        self.neutral_channels = [1500, 1500, 1496, 1500, 1500, 1500, 1500, 1500]
        self.pwm_speed = 25

        self.target = [0, 0]

        self.pre_slam = True
        self.pre_slam_back_and_forth = True     # True: backward; False: forward
        self.pre_slam_set_time = 60             # 6 seconds
        self.pre_slam_timer = self.pre_slam_set_time

        try:
            arm_robot = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            ret = arm_robot(True)
            print("Armed: " + str(ret))
        except rospy.ServiceException, e:
            print("Service call failed: %s" %e)

        rospy.Subscriber("orb_slam2/data", Points, self.slam_callback)

        self.command_pub = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size=1)

        rospy.on_shutdown(self.is_shutdown)


    def slam_callback(self, msg):
        """
        Callback for retrieving the SLAM points.
        Will idenitfy the target direction for the robot to move towards.

        Args:
            msg: Points, which has SLAM feature pixel points and corresponding depth estimates.
        """
        if self.pre_slam:
            self.pre_slam = False

        # Find target point aka average point location
        if len(msg.points) > 0:
            x = 0.0
            y = 0.0
            for pt in msg.points:
                # Transform pixel points in sudo camera frame
                # Mid point of the frame should be origin
                x = x + (pt.x - self.img_width_half)
                y = y + (pt.y - self.img_height_half)
            x = x / len(msg.points)
            y = y / len(msg.points)
            self.target = [x, y]

        else:rospy.Subscriber("orb_slam2/data", Points, self.slarospy.Subrospy.Subscriber("orb_slam2/data", Points, self.slam_callback)

        self.command_pub = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size=1)scriber("orb_slam2/data", Points, self.slam_callback)

        self.command_pub = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size=1)m_callback)

        self.command_pub = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size=1)
            self.target = [0, 0]


    def explore(self):
        """
        Loop type function that will continuously send commands for the robot to move in
        different directions to find new good views. Will stop if no view has been definedself.
        """

        while not rospy.is_shutdown():
            cmd_msg = OverrideRCIn()
            cmd_msg.channels = self.neutral_channels[:]

            # Wait for SLAM to initialize
            if self.pre_slam:
                # move backward
                if self.pre_slam_back_and_forth:
                    cmd_msg.channels[4] = cmd_msg.channels[4] - self.pwm_speed
                # move forward
                else:
                    cmd_msg.channels[4] = cmd_msg.channels[4] + self.pwm_speed

                self.pre_slam_timer = self.pre_slam_timer - 1
                if self.pre_slam_timer < 0:
                    self.pre_slam_back_and_forth =  not self.pre_slam_back_and_forth
                    self.pre_slam_timer = self.pre_slam_set_time

            # SLAM is initialized
            else:
                # Normalize target point
                dist = math.sqrt( pow(self.target[0], 2) + pow(self.target[1], 2) )

                # If there is a target point
                if dist > 0.0:
                    target_x = self.target[0] / dist
                    target_y = self.target[1] / dist

                    target_x = int(target_x * self.pwm_speed)
                    target_y = int(target_y * self.pwm_speed)

                    print(str(target_x) + " " + str(target_y))

                    cmd_msg.channels[5] = cmd_msg.channels[5] + target_x
                    cmd_msg.channels[2] = cmd_msg.channels[2] + target_y

            self.command_pub.publish(cmd_msg)
            self.rate.sleep()


    def is_shutdown(self):
        """
        Shutdown handler for the robot to ensure that it will stop moving before ending node.
        """

        rospy.loginfo("Shutting down.")

        cmd_msg = OverrideRCIn()
        cmd_msg.channels = self.neutral_channels[:]
        self.command_pub.publish(cmd_msg)
        self.rate.sleep()
