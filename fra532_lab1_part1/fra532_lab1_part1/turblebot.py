import pygame
import math
from math import sin, cos, pi
import time
import random
import numpy as np
import threading
import matplotlib.pyplot as plt

class turtlebot:
    def __init__(self, name):
        # robot specifications
        self.name = name
        self.length = 138  # millimeters
        self.width = 178   # millimeters
        self.height = 192  # millimeters
        self.radius = 80   # millimeters
        self.wheel_radius = 33  # millimeters
        self.wheel_width = 18   # millimeters
        self.mass = 1.0    # kg

        # initial pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def get_specs(self):
        return {
            "name": self.name,
            "length": self.length,
            "width": self.width,
            "height": self.height,
            "radius": self.radius,
            "wheel_radius": self.wheel_radius,
            "wheel_width": self.wheel_width,
            "mass": self.mass
        }
    
    def joint_states(self):
        # 20 Hz update rate
        return {
            "left_wheel_position": 0.0,
            "right_wheel_position": 0.0,
            "left_wheel_velocity": 0.0,
            "right_wheel_velocity": 0.0
        }
    
    def imu_data(self):
        # 20 Hz update rate
        return {
            "orientation": (0.0, 0.0, 0.0, 1.0),
            "angular_velocity": (0.0, 0.0, 0.0),
            "linear_acceleration": (0.0, 0.0, 9.81)
        }
    
    
    
    def scan_data(self):
        # 5 Hz update rate
        return [float('inf')] * 360  # Simulated LIDAR data with no obstacles
    
    def differential_drive(self, left_wheel_velocity, right_wheel_velocity):
        # Simple differential drive kinematics
        linear_velocity = (left_wheel_velocity + right_wheel_velocity) / 2.0
        angular_velocity = (right_wheel_velocity - left_wheel_velocity) / (2 * self.radius)
        return linear_velocity, angular_velocity

    def update_pose(self, linear_velocity, angular_velocity, duration):
        # 20 Hz update rate
        delta_theta = angular_velocity * duration
        delta_x = linear_velocity * -sin(self.theta + delta_theta / 2) * duration
        delta_y = linear_velocity * -cos(self.theta + delta_theta / 2) * duration
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
    
    def ekf_odometry(self):
        # Placeholder for EKF odometry
        return {
            "x": self.x,
            "y": self.y,
            "theta": self.theta
        }
    
class ekf:
    def __init__(self):
        # Placeholder for EKF initialization
        pass
    
    def predict(self, control_input, duration):
        # Placeholder for EKF prediction step
        pass
    
    def update(self, measurement):
        # Placeholder for EKF update step
        pass


class TurtlebotSimulator:
    def __init__(self, turtlebot):
        self.turtlebot = turtlebot
        pygame.init()
        self.screen = pygame.display.set_mode((1500, 1000))
        pygame.display.set_caption(f"{self.turtlebot.name} Simulator")
        self.clock = pygame.time.Clock()
        self.running = True
    
    def turtlebot_specs(self):
        return self.turtlebot.get_specs()

    def turtlebot_draw(self):
        # local
        # body
        pygame.draw.circle(self.screen, (0, 0, 255), (int(self.turtlebot.x), int(self.turtlebot.y)), int(self.turtlebot.radius))
        # left wheel
        pygame.draw.rect(self.screen, (255, 0, 0), (int(self.turtlebot.x)-89, int(self.turtlebot.y)-33, self.turtlebot.wheel_width, self.turtlebot.wheel_radius * 2), 10)
        # right wheel
        pygame.draw.rect(self.screen, (255, 0, 0), (int(self.turtlebot.x)+89 - self.turtlebot.wheel_width, int(self.turtlebot.y)-33, self.turtlebot.wheel_width, self.turtlebot.wheel_radius * 2), 10)

        # heading
        heading_x = self.turtlebot.x + self.turtlebot.radius * -sin(self.turtlebot.theta)
        heading_y = self.turtlebot.y + self.turtlebot.radius * -cos(self.turtlebot.theta)
        pygame.draw.line(self.screen, (0, 255, 0), (int(self.turtlebot.x), int(self.turtlebot.y)), (int(heading_x), int(heading_y)), 5) 

    def keyboard_input(self):
        keys = pygame.key.get_pressed()
        left_wheel_velocity = 0.0
        right_wheel_velocity = 0.0
        if keys[pygame.K_UP]:
            left_wheel_velocity += 50.0
            right_wheel_velocity += 50.0
        if keys[pygame.K_DOWN]:
            left_wheel_velocity -= 50.0
            right_wheel_velocity -= 50.0
        if keys[pygame.K_LEFT]:
            left_wheel_velocity -= 25.0
            right_wheel_velocity += 25.0
        if keys[pygame.K_RIGHT]:
            left_wheel_velocity += 25.0
            right_wheel_velocity -= 25.0
        return left_wheel_velocity, right_wheel_velocity

    def run(self):
        i = 0
        figure, pos = plt.subplots()
        figure1, vel = plt.subplots()
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

            left_wheel_velocity, right_wheel_velocity = self.keyboard_input()
            linear_velocity, angular_velocity = self.turtlebot.differential_drive(left_wheel_velocity, right_wheel_velocity)
            self.turtlebot.update_pose(linear_velocity, angular_velocity, 0.05)

            self.screen.fill((255, 255, 255))
            self.turtlebot_draw()
            pygame.display.flip()
            print(f"Pose: x={self.turtlebot.x:.2f}, y={self.turtlebot.y:.2f}, theta={self.turtlebot.theta:.2f}")
            # plot position-timestep graph
            i += 1
            # self.clock.tick(20)  # 20 Hz
            pos.set_xlabel('Time Step')
            pos.set_ylabel('Position (mm)')
            pos.set_title('Turtlebot Position Over Time')
            pos.plot(i, self.turtlebot.x, 'r.', label='X Position')
            pos.plot(i, self.turtlebot.y, 'b.', label='Y Position')
            pos.legend(handles=pos.get_lines(), labels=['X Position', 'Y Position'])

            # plot velocity-timestep graph
            vel.set_title('Turtlebot Velocity Over Time')
            vel.set_xlabel('Time Step')
            vel.set_ylabel('Velocity (mm/s)')
            vel.plot(i, linear_velocity, 'g.', label='Linear Velocity')
            vel.plot(i, angular_velocity, 'm.', label='Angular Velocity')
            vel.legend(handles=vel.get_lines(), labels=['Linear Velocity', 'Angular Velocity'])
            plt.pause(0.001)

        pygame.quit()

if __name__ == "__main__":
    my_turtlebot = turtlebot("Turtlebot3")
    simulator = TurtlebotSimulator(my_turtlebot)
    simulator.run()#!/usr/bin/env python3   