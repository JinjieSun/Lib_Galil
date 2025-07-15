import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from CTCRKinematics import CTCRKinematics
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np



# --- IK function from before ---
def inverse_kinematics_dls(delta_pose, jacobian, beta, tube_lengths, lambda0, lambda1, deg, scale):
    J = jacobian
    delta_pose = np.array(delta_pose)

    W0 = np.eye(6)
    W1 = np.eye(6)
    W0[0:3, 0:3] *= 1e6
    W0[3:5, 3:5] *= 82
    W0[5, 5] = 820
    W1[0:3, 0:3] *= 19
    W1[3:, 3:] *= 1e6


    beta_scaled = np.array(beta) * scale
    length_scaled = np.array(tube_lengths) * scale
    

    e = 20
    b3, b2, b1 = beta_scaled
    l3, l2, l1 = length_scaled

    # Distances
    df1 = l1 + b1
    df2 = l2 + b2 - (l1 + b1)
    df3 = l3 + b3 - (l2 + b2)

    de1 = -b1
    de2 = b1 - b2
    de3 = b2 - b3

    # Losses
    def safe_loss(d): return -1 / (min(e, d)**2)
    lf1 = safe_loss(df1)
    lf2 = safe_loss(df2)
    lf3 = safe_loss(df3)
    le1 = safe_loss(de1)
    le2 = safe_loss(de2)
    le3 = safe_loss(de3)

    # Masks (derivative exists only if below threshold)
    dmf1 = 0 if df1 > e else 1
    dmf2 = 0 if df2 > e else 1
    dmf3 = 0 if df3 > e else 1
    dme1 = 0 if de1 > e else 1
    dme2 = 0 if de2 > e else 1
    dme3 = 0 if de3 > e else 1

    # Gradient terms
    b1_dlf1 = dmf1 * lf1
    b1_dle1 = dme1 * le1 * -1
    b1_dlf2 = dmf2 * lf2 * -1
    b1_dle2 = dme2 * le2

    b2_dlf2 = dmf2 * lf2
    b2_dle2 = dme2 * le2 * -1
    b2_dlf3 = dmf3 * lf3 * -1
    b2_dle3 = dme3 * le3

    b3_dlf3 = dmf3 * lf3
    b3_dle3 = dme3 * le3 * -1

    # Build gradient vector (6D, only beta entries used)
    grad = np.zeros(6)
    grad[5] = -1 / scale * (b1_dlf1 + b1_dle1 + b1_dlf2 + b1_dle2)
    grad[4] = -1 / scale * (b2_dlf2 + b2_dle2 + b2_dlf3 + b2_dle3)
    grad[3] = -1 / scale * (b3_dlf3 + b3_dle3)

    lhs = J.T @ W0 @ J + W1
    rhs = J.T @ W0 @ delta_pose + W1 @ grad
    rhs = J.T @ W0 @ delta_pose 
    delta_joint = np.linalg.pinv(lhs) @ rhs
    print(delta_joint)
    print(grad)
    return delta_joint


# --- Robot Controller ---
class CTCRController:
    def __init__(self, alpha, beta, robotKinematics, tube_length):
        # Initial robot state
        self.alpha = alpha
        self.beta = beta
        self.tube_lengths = tube_length
        self.scale = 1000
        self.lambda0, self.lambda1, self.deg = 1.0, 1.0, 1.0
        self.robotKinematics = robotKinematics
        # Current robot pose (6d): [dx, dy, dz, droll, dpitch, dyaw]
        self.current_pose = np.zeros(6)
        self.results = self.robotKinematics.forward_kinematics(self.alpha, self.beta)
        self.current_backbone = self.results['backbone_positions']
        print("Robot Init Pose:")
        print("Alpha(rad): " + str(self.alpha))
        print("Beta(m): " + str(self.beta) )
    
    def update_robot_joint(self, new_alpha, new_beta):
        self.alpha = new_alpha
        self.beta = new_beta
        
    def get_back_bone(self):
        return self.current_backbone
    
    def get_delta_joints(self, lin_vel, ang_vel):
        # Update target pose based on velocity commands
        self.current_pose[:3] = lin_vel
        self.current_pose[3:] = ang_vel
        # self.movement_cmd = self.convert_6d_pose(self.current_pose)
        print("vel: " + str(self.current_pose))
        # Call FK + IK to update joint angles
        results = self.robotKinematics.forward_kinematics(self.alpha, self.beta)
        body_jacob = results['jacobian_body']
        # Compute IK delta joints for the current pose error
        
        delta_joints = inverse_kinematics_dls(
            self.current_pose, body_jacob, self.beta,
            self.tube_lengths, self.lambda0, self.lambda1, self.deg, self.scale
        )
        return delta_joints
    
    def apply_delta_joints(self, delta_joints):
        """
        delta_joints: array-like with order [a3, b3, a2, b2, a1, b1]
        Apply these deltas to current alpha/beta, call FK and update pose.
        """
        if len(delta_joints) != 6:
            raise ValueError("delta_joints must have length 6")
        
        # Extract deltas by tube (reverse order to alpha/beta arrays)
        delta_alpha = delta_joints[:3]  # a1, a2, a3
        delta_beta = delta_joints[3:]   # b1, b2, b3

        self.alpha= self.alpha + delta_alpha
        self.beta =  self.beta + delta_beta
    
    def convert_from_galil_to_beta(self, beta_galil):
        beta_galil = np.array(beta_galil) * 1e-3
        beta_robot = beta_galil - np.array(self.tube_lengths)
        return beta_robot

if __name__ == "__main__":


    dll_path = "./ctcr_kinematics/Kinematics_CLib.dll"
    xml_path = "./ctcr_kinematics/Galil42.xml"
    ctcrKinematics = CTCRKinematics(dll_path, xml_path)
    robot_length = [139*1e-3, 72*1e-3, 42*1e-3]
    alpha_rad = [0.0, 0.0, 0.0]   # degrees
    beta_m = [-0.060, -0.032, -0.010]  # meters
    robot_controller = CTCRController(alpha_rad, beta_m, ctcrKinematics, robot_length)


    result = ctcrKinematics.forward_kinematics(alpha_rad, beta_m)
    dx = np.arange(10)* 1e-3
    
    N = 50
    P = 100
    backbone_all = np.zeros((N, P, 3))
    
    # visualizer = CTCRVisualizer(get_backbone_fn=robot_controller.get_back_bone)
    # visualizer.show()
    ee_pose = []

    dq = robot_controller.get_delta_joints([0.0005, 0.00090, 0.0000], [0, 0, 0])
    print(dq)
 
