import numpy as np
import math

front_angle = 30.0
back_angle = 45.0
wheel_dist = (0.078089 + 0.077874) / 2

wheel_angles = [
    math.radians(180-front_angle),
    math.radians(180+back_angle),
    math.radians(360-back_angle),
    math.radians(0+front_angle)
]

bot_to_wheel = np.array([
    [-math.sin(wheel_angles[0]), math.cos(wheel_angles[0]), wheel_dist],
    [-math.sin(wheel_angles[1]), math.cos(wheel_angles[1]), wheel_dist],
    [-math.sin(wheel_angles[2]), math.cos(wheel_angles[2]), wheel_dist],
    [-math.sin(wheel_angles[3]), math.cos(wheel_angles[3]), wheel_dist],
])

target_velocity = np.array([0.0, 0.5, 0.0])

print(bot_to_wheel @ target_velocity)