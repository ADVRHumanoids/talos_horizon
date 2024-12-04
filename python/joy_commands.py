import numpy as np
from horizon.rhc.taskInterface import TaskInterface
from phase_manager import pyphase, pymanager
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from sensor_msgs.msg import Joy
import rospy
import math
from visualization_msgs.msg import Marker
from horizon.rhc.gait_manager import GaitManager
from geometry_msgs.msg import Twist

class JoyCommands:
    def __init__(self):
        # self.base_weight = 0.5
        # self.base_rot_weight = 0.5
        # self.com_height_w = 0.1

        self.smooth_joy_msg = None
        self.joy_msg = None


        self.velocity_ref = Twist()

        rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.publisher = rospy.Publisher('smooth_joy', Joy, queue_size=1)
        rospy.wait_for_message('/joy', Joy, timeout=0.5)

        self.__pub = rospy.Publisher('pos_ref', Marker, queue_size=1)

        self.__open_subscribers()

    def __open_subscribers(self):

        self.__base_vel_pub = rospy.Publisher('/horizon/base_velocity/reference', Twist)
        self.__walk_cli = rospy.ServiceProxy('/horizon/walk/switch', SetBool)

    def smooth(self):
        alpha = 0.1
        as_list = list(self.smooth_joy_msg.axes)
        for index in range(len(self.joy_msg.axes)):
            as_list[index] = alpha * self.joy_msg.axes[index] + (1 - alpha) * self.smooth_joy_msg.axes[index]
        self.smooth_joy_msg.axes = tuple(as_list)
        self.publisher.publish(self.smooth_joy_msg)

    def joy_callback(self, msg:Joy):
        if self.smooth_joy_msg is None:
            self.smooth_joy_msg = msg
        self.joy_msg = msg

    def run(self):

        if self.smooth_joy_msg is not None:
            self.smooth()

        if self.joy_msg.buttons[4] == 1:
            self.__walk_cli(True)
        else:
            self.__walk_cli(False)


        if np.abs(self.smooth_joy_msg.axes[0]) > 0.1 or np.abs(self.smooth_joy_msg.axes[1]) > 0.1:
            self.velocity_ref.linear.x = self.smooth_joy_msg.axes[1]
            self.velocity_ref.linear.y = self.smooth_joy_msg.axes[0]
        else:
            self.velocity_ref.linear.x = 0
            self.velocity_ref.linear.y = 0

        if np.abs(self.smooth_joy_msg.axes[3]) > 0.1:
            self.velocity_ref.angular.z = 0.3 * self.smooth_joy_msg.axes[3]
        else:
            self.velocity_ref.angular.z = 0

        if self.joy_msg.buttons[0] == 1:
            # change com height
            self.velocity_ref.linear.z = 0.05

        if self.joy_msg.buttons[2] == 1:
            # change com height
            self.velocity_ref.linear.z = - 0.05

        if self.joy_msg.buttons[2] == 0 and self.joy_msg.buttons[0] == 0:
            self.velocity_ref.linear.z = 0

        self.__base_vel_pub.publish(self.velocity_ref)


