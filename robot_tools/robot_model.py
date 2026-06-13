"""Robot kinematics and dynamics based on Pinocchio.

Wraps a Pinocchio model built from a URDF file and adds support for an
arbitrary base placement (``p0``, ``R0``) and a fixed grasp-frame offset
(``Tgp``) relative to a robot frame.
"""

import numpy as np
import pinocchio


def skew(v: np.ndarray) -> np.ndarray:
    """Return the skew-symmetric matrix of a 3D vector."""
    return np.array(
        [
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0],
        ]
    )


class RobotModel:
    """Kinematics and dynamics of a robot loaded from a URDF file.

    Args:
        urdf_file: Path to the URDF file.
        p0: Base position in the world frame, shape (3,).
        R0: Base orientation in the world frame, shape (3, 3).
        Tgp: Homogeneous transform (4x4) from a robot frame to the grasp
            frame; kinematics returned by :meth:`update_kinematics` refer to
            this grasp frame.
    """

    def __init__(self, urdf_file: str, p0=np.zeros(3), R0=np.eye(3), Tgp=np.eye(4)):
        self.pin_model = pinocchio.buildModelFromUrdf(urdf_file)
        self.pin_data = self.pin_model.createData()

        self.nq = self.pin_model.nq

        self.p0 = p0
        self.R0 = R0
        self.T0 = pinocchio.SE3(R0, p0)
        self.Tgp = pinocchio.SE3(Tgp[:3, :3], Tgp[:3, 3])
        self.Ggp = np.eye(6)  # grasp-frame adjoint, updated in update_kinematics

    def update_kinematics(self, frame_name: str, q: np.ndarray, dq: np.ndarray) -> tuple:
        """Compute the grasp-frame pose and Jacobian for a configuration.

        Args:
            frame_name: Name of the robot frame the grasp offset is attached to.
            q: Joint positions, shape (nq,).
            dq: Joint velocities, shape (nv,).

        Returns:
            Tuple ``(T, J)`` with the grasp-frame pose ``T`` (``pinocchio.SE3``,
            world frame) and the 6xN Jacobian ``J`` (linear on top, angular
            below, world-aligned).
        """
        pinocchio.forwardKinematics(self.pin_model, self.pin_data, q, dq)
        pinocchio.updateFramePlacements(self.pin_model, self.pin_data)

        frame_id = self.pin_model.getFrameId(frame_name)
        T = self.pin_data.oMf[frame_id]
        T = self.T0 * T  # apply base transformation

        J = pinocchio.computeFrameJacobian(
            self.pin_model, self.pin_data, q, frame_id, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )
        J = np.vstack((J[0:3, :], self.R0 @ J[3:6, :]))  # rotate angular part into world frame

        # Shift the Jacobian and pose from the robot frame to the grasp frame
        self.Ggp[0:3, 3:6] = -skew(T.rotation @ self.Tgp.translation)
        J = self.Ggp @ J
        T = T * self.Tgp

        return T, J

    def update_frame_kinematics(
        self, frame_name: str, q: np.ndarray, dq: np.ndarray, ddq: np.ndarray
    ) -> tuple:
        """Compute pose, velocity, and acceleration of a robot frame.

        Args:
            frame_name: Name of the robot frame.
            q: Joint positions, shape (nq,).
            dq: Joint velocities, shape (nv,).
            ddq: Joint accelerations, shape (nv,).

        Returns:
            Tuple ``(T, v, a)`` with the frame pose (``pinocchio.SE3``, world
            frame) and its spatial velocity and acceleration (world-aligned).
        """
        pinocchio.forwardKinematics(self.pin_model, self.pin_data, q, dq, ddq)
        pinocchio.updateFramePlacements(self.pin_model, self.pin_data)

        frame_id = self.pin_model.getFrameId(frame_name)
        T = self.pin_data.oMf[frame_id]
        T = self.T0 * T  # apply base transformation

        v = pinocchio.getFrameVelocity(
            self.pin_model, self.pin_data, frame_id, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )
        a = pinocchio.getFrameAcceleration(
            self.pin_model, self.pin_data, frame_id, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )

        return T, v, a

    def update_dynamics(self, q: np.ndarray, dq: np.ndarray) -> tuple:
        """Compute the joint-space dynamics terms for a configuration.

        Note:
            Gravity is assumed to act along ``[0, 0, -9.81]`` in the robot's
            base frame; if the base is rotated via ``R0``, the gravity vector
            is not rotated accordingly.

        Args:
            q: Joint positions, shape (nq,).
            dq: Joint velocities, shape (nv,).

        Returns:
            Tuple ``(M, c, g)`` with the symmetric mass matrix ``M``, the
            Coriolis/centrifugal torques ``c``, and the gravity torques ``g``.
        """
        M = pinocchio.crba(self.pin_model, self.pin_data, q)
        M = (M + M.T) / 2.0
        pinocchio.nonLinearEffects(self.pin_model, self.pin_data, q, dq)
        n = np.copy(self.pin_data.nle)
        g = pinocchio.nonLinearEffects(self.pin_model, self.pin_data, q, np.zeros(self.pin_model.nv))
        c = n - g

        return M, c, g
