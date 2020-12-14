from typing import Tuple
import rospy
from rospy.topics import Subscriber
from nav_msgs.msg import Odometry
from squaternion import Quaternion

class OdomSensor:
    __odom_sub: Subscriber
    __roll = __pitch = __yaw = 0.0

    def __init__(self):
        self.__odom_sub = rospy.Subscriber('/odom', Odometry, self.__odom_callback)
        
    def __odom_callback(self, msg: Odometry):
        orientation_q = msg.pose.pose.orientation
        q = Quaternion(orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z)
        (self.__roll, self.__pitch, self.__yaw) = q.to_euler()
        # VERBOSE: print(self.roll, self.pitch, self.yaw)

    def get_odometry_as_tuple(self) -> Tuple[float, float, float]:
        return (self.__roll, self.__pitch, self.__yaw)