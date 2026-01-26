
import pinocchio
import numpy as np
import os
from pathlib import Path

def skew(v):
    """Return the skew-symmetric matrix of a 3D vector."""
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

class RobotModel:

    def __init__(self, urdf_file, 
                 p0 = np.zeros(3), 
                 R0 = np.eye(3),
                 Tgp = np.eye(4)):
        self.pin_model = pinocchio.buildModelFromUrdf(urdf_file)
        self.pin_data = self.pin_model.createData()
        
        self.nq = self.pin_model.nq
        
        self.p0 = p0
        self.R0 = R0
        self.T0 = pinocchio.SE3(R0, p0)
        self.Tgp = pinocchio.SE3(Tgp[:3, :3], Tgp[:3, 3])
        self.Ggp = np.eye(6)
            
    def update_kinematics(self, frame_name, q, dq) -> tuple:
            
        pinocchio.forwardKinematics(self.pin_model, self.pin_data, q, dq)
        pinocchio.updateFramePlacements(self.pin_model, self.pin_data)
        
        frame_id = self.pin_model.getFrameId(frame_name)
        T = self.pin_data.oMf[frame_id]
        T = self.T0 * T  # apply base transformation
                
        J = pinocchio.computeFrameJacobian(self.pin_model, self.pin_data, q, frame_id, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        J = np.vstack((J[0:3, :], self.R0 @ J[3:6, :]))  # apply base rotation to angular velocity part
        
        # Transform to the grasp frame
        self.Ggp[0:3, 3:6] = -skew(T.rotation @ self.Tgp.translation)
        J = self.Ggp @ J
        T = T * self.Tgp
        
        return T, J
    
    def update_dynamics(self, q, dq) -> tuple:
        
        # FIXME: the gravity vector is assumed to be [0, 0, -9.81] in the base frame.
        # However, if the base is rotated, this is not correct anymore
        
        M = pinocchio.crba(self.pin_model, self.pin_data, q)
        M = (M+M.T)/2.0
        pinocchio.nonLinearEffects(self.pin_model, self.pin_data, q, dq)
        n = np.copy(self.pin_data.nle)
        g = pinocchio.nonLinearEffects(self.pin_model, self.pin_data, q, np.zeros(self.pin_model.nv))
        c = n - g 
        
        return M, c, g
