# you may find all of these packages helpful!
from scipy.spatial.transform import Rotation as R
import math
import numpy as np
import cv2
import helper.inverse_kinematics as IK
import helper.trajectory_generator as Traj
from enum import Enum

class RobotPlacerWithVision():
    # Camera setup vars
    CAMERA = {
        'fovx': 1.0472,
        'near': 0.01,
        'far': 3,
        'width': 640,
        'height': 480
    }

    # Colors lookups that can be searched for, in BGR
    COLORS = {
        'base': {
            'red': np.array([0, 0, 1]),
            'blue': np.array([1, 0, 0]),
            'green': np.array([0, 1, 0]),
            'yellow': np.array([0, 1, 1]),
            'purple': np.array([1, 0, 1]),
            'cyan': np.array([1, 1, 0]),
            'white': np.array([1, 1, 1]),
            'orange': np.array([0, 0.5, 1]),
            'lime': np.array([0, 1, 0.5]),
            'mint': np.array([0.5, 1, 0]),
            'indigo': np.array([1, 0, 0.5]),
            'pink': np.array([0.5, 0.5, 1])
        }
    }

    # In-world object dimensions
    BLOCK_HEIGHT = 0.03
    CONVEYOR_Z_OFFSET = 0.1
    CONVEYOR_SPEED = 0.05

    # Location tracking vars
    class Location(Enum):
        INITIAL = 1
        DROP = 2
        OBJECT = 3
    
    initial_position = [0.55, 0.15, 0.65]
    initial_rotation = [0, 0, math.pi/2]
    drop_prep_position = [0.55, 0.15, 0.5]
    
    gripperClosed = False
    gripperCloseTime = 12 # Timesteps
    gripperOpenTime = 32 # Timesteps

    def __init__(self, colors):
        self.colors = colors

        self.fsmState = self.resetToDefault
        self.__block_cur_index__ = 0
        self.trajectory = {}
        self.pauseStartTime = None
        self.initial_joints = None
        self.drop_joints = None

        self.COLORS['lower'] = {name: color*200 for name, color in self.COLORS['base'].items()}
        self.COLORS['upper'] = {name: np.maximum(color*255, 100) for name, color in self.COLORS['base'].items()}

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
        centers = []
        b, g, r = cv2.split(self.current_image)

        for name in self.colors:
            lower = self.COLORS['lower'][name]
            upper = self.COLORS['upper'][name]

            threshold = cv2.inRange(self.current_image, lower, upper)
            
            contours, hierarchy = cv2.findContours(threshold, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            width, height = threshold.shape
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
        return pos
    
    ### FSM States

    # Reset to default position
    def resetToDefault(self):
        if not self.initial_joints:
            self.initial_joints = IK.getInverseKinematics(self.initial_position + self.initial_rotation, self.current_joints)
        self.desired_pose = self.initial_position + self.initial_rotation
        self.desired_joints = self.initial_joints
        self.location = self.Location.INITIAL
        self.gripperClosed = False
        self.fsmState = self.generateTrajectoryFromJoints

    # Set goal position to bin
    def moveToDrop(self):
        if not self.drop_joints:
            self.drop_joints = IK.getInverseKinematics(self.drop_prep_position + self.initial_rotation, self.current_joints)
            self.drop_joints[0] -= math.pi/2
            self.drop_joints[5] -= math.pi/2
        self.desired_joints = self.drop_joints
        self.location = self.Location.DROP
        self.fsmState = self.generateTrajectoryFromJoints

    # Wait for valid block(s) to appear and be fully within the camera image
    def waitForBlock(self):
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
        z = self.current_position[2] - self.CONVEYOR_Z_OFFSET - self.BLOCK_HEIGHT
        x = (u - cx)*(z/self.CAMERA['fx'])
        y = -(v - cy)*(z/self.CAMERA['fy'])

        # Move from current camera position to object location,
        # accounting for block height, camera offset, and gripper height
        block_position = np.array(self.current_position) + np.array([y, -x+0.05, 0])
        block_position[2] = 0.165
        block_position[2] += self.CONVEYOR_Z_OFFSET
        start_pos = block_position.tolist() + self.initial_rotation
        self.desired_pose = self.getLocationAfterMotion(start_pos)
        if self.desired_pose:
            self.location = self.Location.OBJECT
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
    
    def generateTrajectoryFromJoints(self):
        self.trajectory['T'] = Traj.getRequiredTime(self.current_joints, self.desired_joints)
        self.trajectory['t'] = 0
        self.trajectory['start'] = self.current_joints
        self.trajectory['end'] = self.desired_joints
        self.fsmState = self.moveToLocation

    # Given desired location and current location, update the next joint
    # position along the trajectory until the goal is reached
    def moveToLocation(self):
        # Move to next position along trajectory
        self.next_joints = Traj.getPosition(self.trajectory['start'], self.trajectory['end'], self.trajectory['t'], self.trajectory['T'])
        self.trajectory['t'] = min(self.trajectory['t'] + 1, self.trajectory['T'])
        
        if self.trajectory['t'] == self.trajectory['T']:
            self.trajectory = {}
            if self.location == self.Location.INITIAL:
                self.fsmState = self.waitForBlock
            elif self.location == self.Location.DROP:
                self.fsmState = self.dropBlock
            else:
                self.fsmState = self.gripBlock

    # Grip block_cur, wait for enough time for the gripper to fully close, then move on
    def gripBlock(self):
        if not self.gripperClosed:
            self.gripperClosed = True
            self.pauseStartTime = self.timestep
        elif self.timestep - self.pauseStartTime >= self.gripperCloseTime:
            self.fsmState = self.moveToDrop

    # Drop block_cur; once gripper is fully open, verify if block is in basket,
    # then update block_cur_index and move to first state
    def dropBlock(self):
        if self.gripperClosed:
            self.gripperClosed = False
            self.pauseStartTime = self.timestep
        elif self.timestep - self.pauseStartTime >= self.gripperOpenTime:
            self.__block_cur_index__ += 1
            self.fsmState = self.resetToDefault
