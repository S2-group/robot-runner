import sys
import time
import rospy
import random
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class TestRun:
    cmd_vel = None

    def __init__(self):
        rospy.init_node("test_run")

        # Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.do_test()

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())

        stop_robot_runner = rospy.Publisher('/robot_runner/run_completed', Bool, queue_size=1)
        rospy.sleep(1)

        while True:
            if stop_robot_runner.get_num_connections() > 0:
                stop_robot_runner.publish(True)
                break

        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)
        sys.exit(0)

    def do_test(self):
        # TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(10)

        # Twist is a datatype for velocity
        move_cmd = Twist()
        # let's go forward at 0.2 m/s
        move_cmd.linear.x = 0.2
        # let's turn at 0 radians/s
        move_cmd.angular.z = 0

        # Perform 10 times, random sleeps so time is undecidable.
        for i in range(0, 2):
            # publish the velocity
            for x in range(0, 20):  # 20 = 2 seconds, r.sleep takes 0.1s (10Hz)
                self.drive_forwards()
                r.sleep()

            self.stop_driving()
            rnd = random.randint(1, 3)
            time.sleep(rnd)  # Random sleep to generate random energy usage patterns

            for y in range(0, 10):
                self.drive_backwards()
                r.sleep()

            self.stop_driving()
            rnd = random.randint(1, 3)
            time.sleep(rnd)  # Random sleep to generate random energy usage patterns

            for y in range(0, 10):
                self.drive_backwards()
                r.sleep()

            self.stop_driving()

        # Test run done, shutdown
        self.shutdown()

    def drive_forwards(self):
        # Twist is a datatype for velocity
        move_cmd = Twist()
        # let's go forward at 0.2 m/s
        move_cmd.linear.x = 0.2
        # let's turn at 0 radians/s
        move_cmd.angular.z = 0

        self.cmd_vel.publish(move_cmd)

    def drive_backwards(self):
        # Twist is a datatype for velocity
        move_cmd = Twist()
        # let's go forward at 0.2 m/s
        move_cmd.linear.x = -0.2
        # let's turn at 0 radians/s
        move_cmd.angular.z = 0

        self.cmd_vel.publish(move_cmd)

    def stop_driving(self):
        self.cmd_vel.publish(Twist())


if __name__ == "__main__":
    TestRun()
