import ctypes
import numpy as np
import os
from ctypes import POINTER, c_double, c_int, c_bool, c_char_p

class CTCRKinematics:
    def __init__(self, dll_path: str, xml_path: str, num_points: int = 100):
        if not os.path.exists(dll_path):
            raise FileNotFoundError(f"Could not find DLL: {dll_path}")
        if not os.path.exists(xml_path):
            raise FileNotFoundError(f"Could not find XML model: {xml_path}")
        
        self.lib = ctypes.CDLL(dll_path)

        # Setup function prototypes
        self.lib.initKinematic.argtypes = [c_char_p]
        self.lib.initKinematic.restype = c_bool
        self.lib.getErrMsg.restype = c_char_p

        self.lib.resetInitialConditions.argtypes = []
        self.lib.calcForwKinematicJacobian.argtypes = [
            POINTER(c_double), POINTER(c_double), c_int,
            POINTER(c_double), POINTER(c_double),
            POINTER(c_double), POINTER(c_int),
            POINTER(c_double), POINTER(c_double)
        ]
        self.lib.calcForwKinematicJacobian.restype = c_int

        # Save config
        self.xml_path = xml_path.encode('utf-8')
        self.num_points = num_points

        # Allocate buffers
        self.backbone_pos = np.empty((num_points, 3), dtype=np.float64)
        self.backbone_rot = np.empty((num_points, 9), dtype=np.float64)
        self.theta = np.empty((num_points, 2), dtype=np.float64)
        self.tube_end_idx = np.zeros(3, dtype=np.int32)
        self.jacobian_body = np.empty((6, 6), dtype=np.float64)
        self.jacobian_spatial = np.empty((6, 6), dtype=np.float64)

        # Initialize robot model
        if not self.lib.initKinematic(self.xml_path):
            raise RuntimeError(self.lib.getErrMsg().decode())

    def reset_conditions(self):
        self.lib.resetInitialConditions()

    def forward_kinematics(self, alpha_deg, beta_m):
        """Compute FK given joint angles (degrees) and beta (meters)."""
        # Convert degrees to radians for alpha
        alpha_rad = np.array(alpha_deg, dtype=np.float64) * np.pi / 180.0
        beta_m = np.array(beta_m, dtype=np.float64)

        self.reset_conditions()

        written = self.lib.calcForwKinematicJacobian(
            alpha_rad.ctypes.data_as(POINTER(c_double)),
            beta_m.ctypes.data_as(POINTER(c_double)),
            self.num_points,
            self.backbone_pos.ctypes.data_as(POINTER(c_double)),
            self.backbone_rot.ctypes.data_as(POINTER(c_double)),
            self.theta.ctypes.data_as(POINTER(c_double)),
            self.tube_end_idx.ctypes.data_as(POINTER(c_int)),
            self.jacobian_body.ctypes.data_as(POINTER(c_double)),
            self.jacobian_spatial.ctypes.data_as(POINTER(c_double))
        )

        if written == 0:
            raise RuntimeError(self.lib.getErrMsg().decode())

        # Slice results to valid length and reshape rotations and Jacobians
        backbone_pos = self.backbone_pos[:written]
        backbone_rot = self.backbone_rot[:written].reshape(written, 3, 3)
        jacobian_body = self.jacobian_body.reshape(6, 6)
        jacobian_spatial = self.jacobian_spatial.reshape(6, 6)

        return {
            'points_written': written,
            'backbone_positions': backbone_pos,
            'backbone_rotations': backbone_rot,
            'jacobian_body': jacobian_body,
            'jacobian_spatial': jacobian_spatial,
        }
