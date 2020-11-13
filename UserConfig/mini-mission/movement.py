import math
import time
import rospy
from geometry_msgs.msg import Twist
from rospy.topics import Publisher
from rospy.timer import Rate

from common.modules.misc.Utilities import rotation_is_close
from common.architectural.Singleton import Singleton
from common.modules.sensors.odom.controllers.OdomSensor import OdomSensor
from common.modules.movement.models.RotationDirection import RotationDirection

class MovementController(metaclass=Singleton):
    # Controllers
    odom_controller: OdomSensor = None

    # Publishers
    __cmd_pub: Publisher
    ros_rate: Rate

    # Rotation variables
    default_traverse_time:          float = 2.5
    default_speed:                  float = 0.6  # Default speed at which robot moves
    self_steering_modifier:         float = 4.0

    rotation_time_threshold:        int   = 1     # Time after which to check if close to target SMALL ROTATION
    full_rotation_time_threshold:   int   = 3     # Time after which to check if close to target FULL ROTATION
    turn_90_degrees:                float = math.pi / 2
    turn_180_degrees:               float = math.pi

    rotation_base_speed:            float = 0.8
    rotation_to_target_threshold:   float = 0.4 # Was 0.3
    rotation_minimal_speed:         float = 0.01 # NEEDED: robot does otherwise not have enough torque to reach target.

    def __init__(self, ros_rate: Rate):
        self.odom_controller = OdomSensor()
        self.__cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.ros_rate = ros_rate

    def publish(self, cmd: Twist) -> None:
        """Handle all publishing responsibility for movement"""
        self.__cmd_pub.publish(cmd)
        self.ros_rate.sleep()

    def calculate_self_steering_angular_vel(self, current_heading, yaw) -> int:
        return (self.self_steering_modifier * self.calculate_self_steering_speed(current_heading, yaw))

    def stop(self) -> None:
        self.publish(Twist()) # Empty twist has all values 0.0, thus motors stop.

    def drive(self, cmd: Twist) -> None:
        self.publish(cmd)

    def drive_forward_with_speed(self, speed: float) -> None:
        cmd = Twist()
        cmd.linear.x = speed
        self.publish(cmd)
    
    def drive_to_heading_with_speed(self, heading: float, speed: float) -> None:
        roll, pitch, yaw = self.odom_controller.get_odometry_as_tuple()
        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = self.calculate_self_steering_angular_vel(heading, yaw)
        self.publish(cmd)

    def drive_to_heading_with_speed_for_seconds(self, heading: float, speed: float, seconds: int) -> None:
        old_time = time.time()
        cmd = Twist()
        while time.time() - old_time <= seconds:
            roll, pitch, yaw = self.odom_controller.get_odometry_as_tuple()
            cmd.linear.x = speed
            cmd.angular.z = self.calculate_self_steering_angular_vel(heading, yaw)
            self.publish(cmd)

    # turn_degrees must be in radians (90 degrees = math.pi / 2 etc)
    def turn_in_degrees(self, heading: float, turn_degrees: int, direction: RotationDirection) -> None:
        print("-- TURNING --")
        cmd = Twist()

        self.stop()

        print(f"Turning degrees: {turn_degrees}")
        turn_degrees = turn_degrees * (math.pi / 180)
        print(f"Turning radians: {turn_degrees} {direction}")
        
        roll = pitch = yaw = 0.0
        while yaw == 0.0:
            (roll, pitch, yaw) = self.odom_controller.get_odometry_as_tuple()

        if direction == RotationDirection.CLCKWISE:
            target_rad = heading - turn_degrees
            if target_rad <= (math.pi * -1):
                delta = target_rad + math.pi
                target_rad = math.pi + (delta)
        elif direction == RotationDirection.CNTR_CLCKWISE:
            target_rad = heading + turn_degrees
            if target_rad >= math.pi:
                delta = target_rad - math.pi
                target_rad = (math.pi - delta) * -1

        old_time = time.time()

        while True:
            (roll, pitch, yaw) = self.odom_controller.get_odometry_as_tuple()

            speed = self.rotation_base_speed

            if (time.time() - old_time > self.rotation_time_threshold) and \
            ((yaw > target_rad - self.rotation_to_target_threshold and yaw <= target_rad) or \
            (yaw >= target_rad and yaw <  target_rad + self.rotation_to_target_threshold)):
                speed = speed * (target_rad - yaw)
            else:             
                if direction == RotationDirection.CLCKWISE:
                    speed = speed * -1

            # VERBOSE:
            print(f"Speed= {speed} | Target={target_rad} | Current={yaw} | Approx={rotation_is_close(target_rad, yaw)}")
            
            if(speed < 0):
                speed -= self.rotation_minimal_speed
            else:
                speed += self.rotation_minimal_speed

            cmd.angular.z = speed
            self.publish(cmd)
            self.ros_rate.sleep()

            if (time.time() - old_time > self.rotation_time_threshold) and rotation_is_close(target_rad, yaw):
                break

        self.stop()

    def calculate_self_steering_speed(self, heading, yaw):
        speed: float = 0.0
        
        # VERBOSE
        #print(f"YAW: {yaw} and HDG: {heading}")

        if (yaw > 0 and heading < 0) or (yaw < 0 and heading > 0):
            delta_cw = 0.0
            delta_ccw = 0.0

            if yaw < 0:
                delta_cw = (math.pi - (yaw * -1)) + (math.pi - heading)
                delta_ccw = (yaw * -1) + heading
            else:
                delta_cw = yaw + (heading * -1)
                delta_ccw = (math.pi - yaw) + ((math.pi * -1) - heading)

            speed = (delta_cw * -1) if ((delta_cw < delta_ccw) and delta_cw != 0.0) or delta_ccw == 0.0 else delta_ccw
        else:
            speed = heading - yaw

        return speed
        # VERBOSE
        #print(f"\tSPEED: {speed}")