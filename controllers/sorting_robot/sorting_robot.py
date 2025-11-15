"""sorting_robot controller."""

from controller import Robot
from controller import Supervisor
from helper.pose_finder import getOptimalPose
from helper.kinematics_controller import kinematics_controller

### Tuning
IK_WINDOW = 20
    

### Main Script

# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Get all motors (joint actuators)
motors = []
motors.append(robot.getDevice("shoulder_pan_joint"))
motors.append(robot.getDevice("shoulder_lift_joint"))
motors.append(robot.getDevice("elbow_joint"))
motors.append(robot.getDevice("wrist_1_joint"))
motors.append(robot.getDevice("wrist_2_joint"))
motors.append(robot.getDevice("wrist_3_joint"))

# TODO add gripper joint

# Enable joint position sensors (radians)
sensors = []
for m in motors:
    ps = m.getPositionSensor()
    ps.enable(timestep)
    sensors.append(ps)

# Main loop:
tt = 0

# - perform simulation steps until Webots is stopping the controller
last_q = []
for sensor in sensors:
    last_q.append(sensor.getValue())

kinematics = kinematics_controller(IK_WINDOW)
    
while robot.step(timestep) != -1:
    desired_pose = getOptimalPose()
    desired_command = kinematics.getCommand(tt, desired_pose, last_q)
    last_q = desired_command
    
    # 7 Robot Joints
    for j, motor in enumerate(motors):
        motor.setPosition(desired_command[j])
        
    tt+=1
