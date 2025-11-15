import helper.inverse_kinematics as IK
import helper.trajectory_generator as TrajGen

### Class to handle getting the next robot command
class kinematics_controller:
    pose_d = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # Desired pose is 6 floats
    command_d = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # Commands for the 6 joints

    # Start and end timesteps for trajectory
    tt_start = 0
    tt_end = 0
    
    def __init__(self, ik_window):
        self.ik_window = ik_window

    def getCommand(self, tt: int, desiredPose: list[float], last_q: list[float]) -> list[float]:
        # Recalculate trajectory every IK_WINDOW number of timesteps
        if tt % self.ik_window == 0:
            self.pose_d = desiredPose
            self.tt_start = tt
            self.command_d = IK.getInverseKinematics(self.pose_d, last_q)
            self.tt_end = self.tt_start + TrajGen.getRequiredTime(last_q, self.command_d)

        # Calculate current pose along trajectory
        currentCommand = TrajGen.getPosition(last_q, self.command_d, tt, self.tt_end - self.tt_start)

        # Determine if gripper should close

        return [0, 0, 0, 0, 0, 0, 0]