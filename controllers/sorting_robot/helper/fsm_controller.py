# you may find all of these packages helpful!
from scipy.spatial.transform import Rotation as R
import math
import numpy as np
import cv2
import helper.inverse_kinematics as IK
import helper.trajectory_generator as Traj

class RobotPlacerWithVision():

    # COORDINATES RELATIVE TO CAMERA TABLE VIEW:
    # X = UP/DOWN, higher = camera moves down, range ~ [0.4, 0.7]
    # Y = LEFT/RIGHT, higher = camera moves right, range ~ [-0.05, 0.35]
    # Z = DEPTH

    # Mat center in world: -0.1 2.75 0.7, so mat->robot = [0.1, -0.55, 0] turned 90 deg left = [-0.55, -0.1, 0], mat->camera = [-0.55, -0.15, 0]
    # Camera: Robot->Camera = [-0.05 0 0], Camera->Robot = [0.05 0 0], Camera->Mat = [0.55, 0.15, 0]
    # Robot center in world: 0 2.2 0.7, rotated 90 deg around Z (left)
    # Camera rotation: -90 deg around Z (right), so camera->robot = 90 deg left

    CAMERA = {
        'fovx': 1.0472,
        'near': 0.01,
        'far': 3,
        'width': 640,
        'height': 480
    }

    COLORS = {
        'red': [1, 0, 0],
        'blue': [0, 0, 1],
        'green': [0, 1, 0]
    }


    BLOCK_HEIGHT = 0.03
    CONVEYOR_Z_OFFSET = 0.1
    CONVEYOR_SPEED = 0.05

    initial_position = [0.55, 0.15, 0.65]
    initial_rotation = [0, 0, -math.pi/2]
    drop_prep_position = [0.45, -0.15, 0.6]
    drop_position = [0, -0.6, 0.6]
    drop_post_position = [0.45, -0.15, 0.65]
    
    gripperClosed = False
    gripperCloseTime = 16 # Timesteps
    gripperOpenTime = 16 # Timesteps
    cameraWaitTime = 0 # Timesteps

    def __init__(self):
        self.fsmState = self.resetToDefault
        self.__block_cur_index__ = 0
        self.trajectory = {}
        self.pauseStartTime = None

        # Camera intrinsics
        self.CAMERA['fovy'] = 2 * math.atan(math.tan(self.CAMERA['fovx']*0.5) * (self.CAMERA['height']/self.CAMERA['width']))
        self.CAMERA['fx'] = self.CAMERA['width']/(2*math.tan(self.CAMERA['fovx']/2))
        self.CAMERA['fy'] = self.CAMERA['height']/(2*math.tan(self.CAMERA['fovy']/2))

    def getRobotCommand(self, tt, current_q, current_image_bgr):        
        # Update current joints and task-space transformations
        self.current_joints = current_q
        self.next_joints = current_q
        current_transform = IK.getFK(current_q)
        self.current_position = current_transform[:3,3]
        self.current_rotation = R.from_matrix(current_transform[:3,:3]).as_rotvec()

        # Update current time
        self.timestep = tt

        # Update current image
        self.current_image = current_image_bgr

        # Call current FSM function
        self.fsmState()

        return self.next_joints + [self.gripperClosed]
    

    ### Helper functions

    # Get centers of detected objects within the current image
    def getPixelLocations(self):
        color = (1, 0, 0)
        b, g, r = cv2.split(self.current_image)

        r = cv2.threshold(r, 200*color[0], 255*color[0], cv2.THRESH_BINARY)[1]
        g = cv2.threshold(g, 200*color[1], 255*color[1], cv2.THRESH_BINARY)[1]
        b = cv2.threshold(b, 200*color[2], 255*color[2], cv2.THRESH_BINARY)[1]
        merged = cv2.merge([b, g, r])
        objects = cv2.threshold(cv2.cvtColor(merged, cv2.COLOR_BGR2GRAY), 1, 255, cv2.THRESH_BINARY)[1]
        
        contours, hierarchy = cv2.findContours(objects, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        centers = []
        width, height = objects.shape
        for i in contours:
            # Skip if touching edge of image
            x, y, w, h = cv2.boundingRect(i)
            if x == 0 or y == 0 or x+w == width or y+h == height: continue

            # Get center point
            M = cv2.moments(i)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                centers.append((cx, cy))
        
        return centers
    
    # Given a start location, find where the target object will be by the time
    # the gripper is able to get down to the belt.
    def getLocationAfterMotion(self, start):
        # Check time required to get to the position of the object at each timestep
        # until the time required <= the time elapsed
        elapsed = 0
        required = Traj.getRequiredTime(self.current_joints, IK.getInverseKinematics(start, self.current_joints)) + self.gripperCloseTime
        pos = start[:]
        while elapsed < required:
            if elapsed > 500: return None
            elapsed += 1
            pos[1] -= self.CONVEYOR_SPEED * Traj.TIMESTEP_LENGTH
            required = Traj.getRequiredTime(self.current_joints, IK.getInverseKinematics(pos, self.current_joints)) + self.gripperCloseTime
            print(elapsed, required)
        print('Start:', start, Traj.getRequiredTime(self.current_joints, IK.getInverseKinematics(start, self.current_joints)) + self.gripperCloseTime)
        print('Position:', pos, Traj.getRequiredTime(self.current_joints, IK.getInverseKinematics(pos, self.current_joints)) + self.gripperCloseTime, elapsed)
        return pos
    
    ### FSM States

    # Reset to default position
    def resetToDefault(self):
        self.desired_pose = self.initial_position + self.initial_rotation
        self.gripperClosed = False
        self.fsmState = self.generateTrajectory

    def prepMoveToBin(self):
        self.desired_pose = self.drop_prep_position + self.initial_rotation
        self.fsmState = self.generateTrajectory
    
    def postMoveToBin(self):
        self.desired_pose = self.drop_post_position + self.initial_rotation
        self.fsmState = self.generateTrajectory

    # Set goal position to bin
    def moveToBin(self):
        self.desired_pose = self.drop_position + self.initial_rotation
        self.fsmState = self.generateTrajectory

    # Wait for valid block(s) to appear and be fully within the camera image
    def waitForBlock(self):
        # Wait for a small number of timesteps to ensure camera is done moving
        if self.timestep - self.pauseStartTime < self.cameraWaitTime:
            return

        # Wait for valid block(s)
        centers = self.getPixelLocations()
        if len(centers) == 0:
            return
        
        # If valid block visible, get block furthest down the belt and move to next state
        else:
            self.block_cur = min(centers, key=lambda point: point[0])
            self.fsmState = self.getBlockLocation

    # Get location of block furthest down the belt
    def getBlockLocation(self):
        # Assumes only one object center is returned
        u, v = self.block_cur
        
        # convert pixel location to camera-space location
        cx = self.CAMERA['width']/2
        cy = self.CAMERA['height']/2
        z = self.current_position[2] - self.CONVEYOR_Z_OFFSET
        x = (u - cx)*(z/self.CAMERA['fx'])
        y = -(v - cy)*(z/self.CAMERA['fy'])

        # Move from current camera position to object location,
        # accounting for block height, camera offset, and gripper height
        block_position = np.array(self.current_position) + np.array([-y, x-0.05, 0])
        block_position[2] = 0.165
        block_position[2] += self.CONVEYOR_Z_OFFSET
        start_pos = block_position.tolist() + self.initial_rotation
        self.desired_pose = self.getLocationAfterMotion(start_pos)
        if self.desired_pose:
            self.fsmState = self.generateTrajectory
        else:
            self.fsmState = self.resetToDefault


    # Generate trajectory to current desired position
    def generateTrajectory(self):
        desired_joints = IK.getInverseKinematics(self.desired_pose, self.current_joints)
        self.trajectory['T'] = Traj.getRequiredTime(self.current_joints, desired_joints)
        self.trajectory['t'] = 0
        self.trajectory['start'] = self.current_joints
        self.trajectory['end'] = desired_joints
        self.fsmState = self.moveToLocation

    # Given desired location and current location, update the next joint
    # position along the trajectory until the goal is reached
    def moveToLocation(self):
        # Move to next position along trajectory
        self.next_joints = Traj.getPosition(self.trajectory['start'], self.trajectory['end'], self.trajectory['t'], self.trajectory['T'])
        self.trajectory['t'] = min(self.trajectory['t'] + 1, self.trajectory['T'])
        
        if self.trajectory['t'] == self.trajectory['T']:
            self.trajectory = {}
            if self.desired_pose == self.initial_position + self.initial_rotation:
                self.pauseStartTime = self.timestep
                self.fsmState = self.waitForBlock
            elif self.desired_pose == self.drop_prep_position + self.initial_rotation:
                self.fsmState = self.moveToBin
            elif self.desired_pose == self.drop_position + self.initial_rotation:
                self.fsmState = self.dropAndMarkDone
            elif self.desired_pose == self.drop_post_position + self.initial_rotation:
                self.fsmState = self.resetToDefault
            else:
                self.fsmState = self.gripBlock

    # Grip block_cur, wait for enough time for the gripper to fully close, then move on
    def gripBlock(self):
        if not self.gripperClosed:
            self.gripperClosed = True
            self.pauseStartTime = self.timestep
        elif self.timestep - self.pauseStartTime >= self.gripperCloseTime:
            self.fsmState = self.prepMoveToBin

    # Drop block_cur; once gripper is fully open, verify if block is in basket,
    # then update block_cur_index and move to first state
    def dropAndMarkDone(self):
        if self.gripperClosed:
            self.gripperClosed = False
            self.pauseStartTime = self.timestep
        elif self.timestep - self.pauseStartTime >= self.gripperOpenTime:
            self.__block_cur_index__ += 1
            self.fsmState = self.postMoveToBin
