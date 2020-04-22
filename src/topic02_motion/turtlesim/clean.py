#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleCleaner():

    def __init__(self, cmd_vel_topic, position_topic):
        #Pose initialisation
        self.__current_x = 0.0
        self.__current_y = 0.0
        self.__current_theta = 0.0

        self.__cmd_vel_topic = cmd_vel_topic
        self.__position_topic = position_topic

        self.__velocity_publisher = rospy.Publisher(self.__cmd_vel_topic, Twist, queue_size=10)
        self.__pose_subscriber = rospy.Subscriber(self.__position_topic, Pose, self.__poseCallback)
        rospy.sleep(1) # Give him time to get current Pose


    def __poseCallback(self, msg):
        self.__current_x = msg.x
        self.__current_y = msg.y
        self.__current_theta = msg.theta


    def _go2goal(self, target_x=1.0, target_y=1.0, target_theta=None, tolerance=0.01):
        """ Moves turtle to target coordinates facing the desired angle (if given).
        """
        velocity_msg = Twist()

        while(True):
            K_linear = 0.5
            euclidean_distance = math.sqrt((target_x - self.__current_x) ** 2 + (target_y - self.__current_y) ** 2)
            linear_vel = K_linear * euclidean_distance

            K_angular = 4.0
            theta =  math.atan2((target_y - self.__current_y),(target_x - self.__current_x))
            angular_vel = K_angular * (theta - self.__current_theta)

            velocity_msg.linear.x = linear_vel
            velocity_msg.angular.z = angular_vel
            self.__velocity_publisher.publish(velocity_msg)

            #Break condition
            if (euclidean_distance < tolerance):
                break
            
        #Sets desired orientation (if given)
        if target_theta is not None:
            self._set_desired_orientation(target_angle_in_rad=target_theta)


    def _move(self, linear_velocity, target_distance, is_forward=True):
        """ Moves turtle linearly for a given distance with a given speed.
        """
        velocity_msg = Twist()

        if (is_forward):
            velocity_msg.linear.x = abs(linear_velocity)
        else:
            velocity_msg.linear.x = -abs(linear_velocity)

        t0 = rospy.Time.now().to_sec()
        distance_moved = 0.0

        loop = rospy.Rate(10)
        rospy.loginfo('STARTED MOVING...')
        while True:
            self.__velocity_publisher.publish(velocity_msg)
            loop.sleep()

            t1 = rospy.Time.now().to_sec()
            distance_moved = linear_velocity * (t1 - t0)

            #Break condition
            if not distance_moved < target_distance:
                rospy.loginfo('DISTANCE REACHED!')

                #Stop moving
                velocity_msg.linear.x = 0
                self.__velocity_publisher.publish(velocity_msg)
                break


    def _rotate(self, angular_vel_in_deg, relative_angle_in_deg, clockwise=False):
        """ Rotates the turtle with the given angular velocity for the given degrees.
        """
        velocity_msg = Twist()

        angular_velocity = math.radians(abs(angular_vel_in_deg))
        if clockwise:
            velocity_msg.angular.z = -abs(angular_velocity)
        else:
            velocity_msg.angular.z = abs(angular_velocity)
        
        t0 = rospy.Time().now().to_sec()
        angles_moved = 0.0

        loop = rospy.Rate(10)
        rospy.loginfo('STARTED ROTATING...')
        while True:
            self.__velocity_publisher.publish(velocity_msg)
            loop.sleep()

            t1 = rospy.Time.now().to_sec()
            angles_moved = math.degrees(angular_velocity * (t1 - t0))

            #Break condition
            if not angles_moved < relative_angle_in_deg:
                rospy.loginfo('ROTATION COMPLETED')

                #Stop rotating
                velocity_msg.angular.z = 0
                self.__velocity_publisher.publish(velocity_msg)
                break


    def _set_desired_orientation(self, target_angle_in_rad):
        """ Sets the desired orientation of the turtle.
        """
        relative_angle_in_rad = target_angle_in_rad - self.__current_theta
        clockwise = True if relative_angle_in_rad < 0 else False

        self._rotate(30, math.degrees(abs(relative_angle_in_rad)), clockwise=clockwise)

    def spiral_cleaning(self):
        """ Spiral cleaning!
        """
        velocity_msg = Twist()
        rk = 0
        wk = 4

        loop = rospy.Rate(1)
        while ((self.__current_x < 10.5) and (self.__current_y < 10.5)):
            rk += 1
            velocity_msg.linear.x = rk
            velocity_msg.angular.z = wk
            self.__velocity_publisher.publish(velocity_msg)

            loop.sleep()

        #Stop spiral cleaning
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 0
        self.__velocity_publisher.publish(velocity_msg)


    def grid_cleaning(self):
        """ Grid cleaning!
        """
        target_x = 1.0
        target_y = 1.0
        target_theta = 0.0
        self._go2goal(target_x, target_y, target_theta, tolerance=0.01)

        self._move(2.0, 9.0, is_forward=True)
        self._rotate(20, 90, clockwise=False)

        self._move(2.0, 9.0, is_forward=True)
        self._rotate(20, 90, clockwise=False)

        self._move(2.0, 1.0, is_forward=True)
        self._rotate(20, 90, clockwise=False)

        self._move(2.0, 9.0, is_forward=True)
        self._rotate(20, 90, clockwise=True)

        self._move(2.0, 1.0, is_forward=True)
        self._rotate(20, 90, clockwise=True)

        self._move(2.0, 9.0, is_forward=True)
        self._rotate(20, 90, clockwise=False)

        self._move(2.0, 1.0, is_forward=True)
        self._rotate(20, 90, clockwise=False)

        self._move(2.0, 9.0, is_forward=True)
        self._rotate(20, 90, clockwise=True)

        #And so on..
        pass



def menu():
    while True:
        print('\nFOR SPIRAL CLEANING PRESS: 1')
        print('FOR GRID CLEANING PRESS: 2\n')
        try:
            cleaning_method = int(raw_input())
            if cleaning_method in [1,2]:
                return cleaning_method
        except ValueError:
            sys.exit(1)


if __name__ == "__main__":
    try:
        cmd_vel_topic = '/turtle1/cmd_vel'
        position_topic = "/turtle1/pose"

        rospy.init_node('turtle_cleaner_node', anonymous=True)
        robot = TurtleCleaner(cmd_vel_topic, position_topic)

        #Testing
        #robot._move(linear_velocity=1, target_distance=1, is_forward=True)
        #robot._rotate(angular_vel_in_deg=30, relative_angle_in_deg=45, clockwise=False)
        #robot._set_desired_orientation(target_angle_in_rad=math.radians(90))
        #robot._go2goal(target_x=1.0, target_y=1.0, target_theta=math.radians(90), tolerance=0.01)
    
        cleaning_method = menu()
        if cleaning_method == 1:
            robot.spiral_cleaning()
        elif cleaning_method == 2:
            robot.grid_cleaning()

    except rospy.ROSInitException:
        rospy.loginfo('NODE SHUTDOWN...')
