#! /usr/bin/env python
import rospy
import actionlib
import numpy as np
import sys
from math import pow, atan2, sqrt
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Quaternion


class SimpleNavigationGoals:
    def __init__(self):
        # Creating a node
        # rospy.init_node("turtlebot3_ctl", anonymous=True)

        # Using publisher to edit linear and angular velocity
        self.velocity_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        # Uing publisher to edit pose
        self.pose_subscriver = rospy.Subscriber(
            "/amcl_pose", PoseWithCovarianceStamped, self.update_pose,
        )

        self.pose = PoseWithCovarianceStamped()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        self.pose = data
        self.pose.pose.pose.position.x = round(self.pose.pose.pose.position.x, 4)
        self.pose.pose.pose.position.y = round(self.pose.pose.pose.position.y, 4)
        self.pose.pose.pose.orientation.x = round(self.pose.pose.pose.orientation.x, 4)
        self.pose.pose.pose.orientation.y = round(self.pose.pose.pose.orientation.y, 4)
        self.pose.pose.pose.orientation.z = round(self.pose.pose.pose.orientation.z, 4)
        self.pose.pose.pose.orientation.w = round(self.pose.pose.pose.orientation.w, 4)

    def distance(self, goal_pose):
        # distance entre la position actuelle et le goal
        return sqrt(
            pow((goal_pose.pose.pose.position.x - self.pose.pose.pose.position.x), 2)
            + pow((goal_pose.pose.pose.position.y - self.pose.pose.pose.position.y), 2)
        )

    def line_speed(self, goal_pose, constant=0.1):
        # Definition de la vitesse lineaire
        return constant * self.distance(goal_pose)

    def steering_angle(self, goal_pose):
        # Definition angle rotation
        return atan2(
            goal_pose.pose.pose.position.y - self.pose.pose.pose.position.y,
            goal_pose.pose.pose.position.x - self.pose.pose.pose.position.x,
        )

    def quaternion_to_euler(self, yaw):
        pitch = 0
        roll = 0
        q = (
            np.cos(roll / 2)
            * np.cos(pitch / 2)
            * np.cos(yaw / 2)
            * np.sin(roll / 2)
            * np.sin(pitch / 2)
            * np.sin(yaw / 2)
        )
        return q

    def angular_speed(self, goal_pose, constant=0.2):
        # Definie le calcul de la vitesse angulaire
        theta = self.quaternion_to_euler(theta)
        return constant * (self.steering_angle(goal_pose) - theta)

    def _shutdown(self):
        speed_msg = Twist()
        rospy.signal_shutdown("Process stoped")
        speed_msg.linear.x = 0
        speed_msg.angular.z = 0
        self.velocity_publisher.publish(speed_msg)
        rospy.sleep(1)

    def go_to(self, x, y, theta):

        theta = self.quaternion_to_euler(theta)

        # Deplace le robot jusqu'au point voulu
        goal_pose = PoseWithCovarianceStamped()

        # recuperation des informations de l'utilisateur
        # goal_pose.pose.pose.position.x = input("Rentrez la position en x :")
        # goal_pose.pose.pose.position.y = input("Rentrez la position en y :")
        # goal_pose.theta = input("Rentrez l'angle :")

        goal_pose.pose.pose.position.x = x
        goal_pose.pose.pose.position.y = y
        goal_pose.pose.pose.orientation = theta

        # Definition tolerance --> gestion de l'espace proche
        distance_tolerance = 0.1

        speed_msg = Twist()

        # print(
        #     "Position du robot: x {}, y {}, theta {}".format(
        #         self.pose.pose.pose.position.x, self.pose.pose.pose.position.y, theta,
        #     )
        # )

        print("Lancement...")

        while self.distance(goal_pose) >= distance_tolerance:

            print("En deplacement ...")
            #
            # # Affiche les positions :
            # #
            # print(
            #     "Position du robot en x : {}, y : {} theta: {}\n".format(
            #         self.pose.pose.pose.position.x,
            #         self.pose.pose.pose.position.y,
            #         theta,
            #     )
            # )
            # print(
            #     "Position souhaitee : x {}, y {}, theta {}\n".format(
            #         goal_pose.pose.pose.position.x,
            #         goal_pose.pose.pose.position.y,
            #         theta,
            #     )
            # )

            # Proportionnel controle

            # Vitesse linaire en x
            speed_msg.linear.x = self.line_speed(goal_pose)
            speed_msg.linear.y = 0
            speed_msg.linear.z = 0

            # Vitesse angulaire en z
            speed_msg.angular.x = 0
            speed_msg.angular.y = 0
            speed_msg.angular.z = self.angular_speed(goal_pose)

            # Edition notre vitesse
            self.velocity_publisher.publish(speed_msg)

            # Edition attente
            self.rate.sleep()

            # Gestion de l'annulation du programme (crl+c)
            if rospy.is_shutdown():
                print("Extinction")
                rospy.on_shutdown("Stop")
                break

            rospy.spin()

        # Arret du robot une fois que le point est atteint
        print("Je suis arrive")
        speed_msg.linear.x = 0
        speed_msg.angular.z = 0
        self.velocity_publisher.publish(speed_msg)

        # Gestion de l'annulation du programme (crl+c)
        if rospy.is_shutdown():
            print("Extinction")
            rospy.on_shutdown("Stop")

        rospy.spin()


if __name__ == "__main__":
    try:
        while not rospy.is_shutdown():
            x = TurtleBot()
            x.goto()
        x.shutdown()
    except rospy.ROSInterruptException:
        pass
