#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import pow, atan2, sqrt, pi


class TurtleControllerToFollow:
    def __init__(self):
        rospy.init_node('controllerTurtle2ToFollowTurtle1')

        self.name_of_turtle1 = 'turtle1'
        self.name_of_turtle2 = 'lucia'

        self.turtle_2_velocity_publisher = rospy.Publisher('/'+self.name_of_turtle2 +'/cmd_vel', Twist, queue_size=10)
        self.turtle_1_position_subscriber = rospy.Subscriber('/'+self.name_of_turtle1+'/pose', Pose, self.update_current_position_of_turtle1)
        self.turtle_2_position_subscriber = rospy.Subscriber('/'+self.name_of_turtle2+'/pose', Pose, self.update_current_position_of_turtle2)

        self.distance_tolerance = 0.5
        self.last_position = Pose()
        self.current_position_to_follow = Pose()
        self.rate = rospy.Rate(10)
        rospy.spin()
    
    def update_current_position_of_turtle2(self, position):
        self.last_position = position
        self.last_position.x = self.last_position.x
        self.last_position.y = self.last_position.y

    def update_current_position_of_turtle1(self, position):
        self.current_position_to_follow = position
        self.current_position_to_follow.x = self.current_position_to_follow.x
        self.current_position_to_follow.y = self.current_position_to_follow.y
        self.move_turtle2_to_follow_turtle1(self.current_position_to_follow)

    def euclidean_distance(self, goal_position):
        value = sqrt(pow((goal_position.x - self.last_position.x), 2) + pow((goal_position.y - self.last_position.y), 2))
        return value

    def linear_vel(self, goal_position, constant=1.5):
        return constant * self.euclidean_distance(goal_position)

    def angle_between_turtles(self, goal_position):        
        return atan2(goal_position.y - self.last_position.y, goal_position.x - self.last_position.x)

    def angular_vel(self, goal_pose, constant=4):
        angle_error = (self.angle_between_turtles(goal_pose) - self.last_position.theta)

        # Convertendo o erro alvo para valores entre -pi e +pi
        heading_error = ((angle_error + pi) % (2*pi)) - pi

        return constant * heading_error

    def move_turtle2_to_follow_turtle1(self, position):
        self.current_position_to_follow = position

        vel_msg_to_turtle2 = Twist()

        last_error = 0
        current_error = self.euclidean_distance(self.current_position_to_follow)

        while current_error >= self.distance_tolerance and last_error != current_error:
            
            # Velocidade linear no eixo x
            vel_msg_to_turtle2.linear.x = self.linear_vel(self.current_position_to_follow)
            vel_msg_to_turtle2.linear.y = 0
            vel_msg_to_turtle2.linear.z = 0

            # Velocidade angular no eixo z
            vel_msg_to_turtle2.angular.x = 0
            vel_msg_to_turtle2.angular.y = 0
            vel_msg_to_turtle2.angular.z = self.angular_vel(self.current_position_to_follow)

            #Publicando a velocidade
            self.turtle_2_velocity_publisher.publish(vel_msg_to_turtle2)

            last_error = current_error

            #Publicando na taxa desejada
            self.rate.sleep()

        #Parando a turtle2 depois que a diferenca de distancias euclidianas foi zerada
        vel_msg_to_turtle2.linear.x = 0
        vel_msg_to_turtle2.angular.z = 0
        self.turtle_2_velocity_publisher.publish(vel_msg_to_turtle2)

if __name__ == '__main__':
    c = TurtleControllerToFollow()