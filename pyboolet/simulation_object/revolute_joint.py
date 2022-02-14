from enum import Enum
from typing import List, TYPE_CHECKING, Tuple, Optional

import pybullet

import numpy as np

from .joint import Joint

if TYPE_CHECKING:
    from ..multibody import Multibody


class JointControlMode(Enum):
    TORQUE_CONTROL = 0
    VELOCITY_CONTROL = 1
    POSITION_CONTROL = 2
    NONE = 3


class RevoluteJoint(Joint):
    """
    Represents a revolute joint in the simulation.
    """

    def __init__(self, joint_index: int, body: "Multibody", name: str):
        """
        :param joint_index:         Index of this joint in the simulation.
        :param body:                Body this link is part of.
        :param name:                Name of this joint in the simulation.
        """
        super(RevoluteJoint, self).__init__(joint_index, body, name)
        self.torque: float = self.max_force
        self.target_velocity: float = 0.0
        self.target_position: float = 0.0
        self.controller_position_gain: float = 0.5
        self.controller_velocity_gain: float = 0.5
        self.control_mode: JointControlMode = JointControlMode.VELOCITY_CONTROL
        self.__acceleration_limit = None

        self.__joint_index = joint_index

    def reset_joint_state(self, position: float, velocity: float):
        was_soft_fixed = self.soft_fixed
        self.soft_fixed = False
        self.call(pybullet.resetJointState, targetValue=position, targetVelocity=velocity)
        self.soft_fixed = was_soft_fixed

    @property
    def joint_position(self) -> float:
        return self.call(pybullet.getJointState)[0]

    @property
    def joint_velocity(self):
        return self.call(pybullet.getJointState)[1]

    @property
    def max_force(self) -> float:
        return self.joint_info.joint_max_force

    @property
    def max_velocity(self) -> float:
        return self.joint_info.joint_max_velocity

    @property
    def interval(self) -> Tuple[float, float]:
        info = self.joint_info
        return info.joint_lower_limit, info.joint_upper_limit

    @property
    def acceleration_limit(self) -> Optional[float]:
        """
        The joint acceleration limit. Note that this value is ignored in torque control mode.
        :return:
        """
        return self.__acceleration_limit

    @acceleration_limit.setter
    def acceleration_limit(self, value: Optional[float]):
        self.__acceleration_limit = value

    @property
    def clamped_target_velocity(self) -> float:
        if self.__acceleration_limit is None:
            return self.target_velocity
        else:
            vel = self.joint_velocity
            max_vel_diff = self.__acceleration_limit * self.body.physics_client.time_step
            return max(min(self.target_velocity, vel + max_vel_diff), vel - max_vel_diff)

    @classmethod
    def _act(cls, body: "Multibody", simulation_objects: List["RevoluteJoint"]):
        assert all(isinstance(so, RevoluteJoint) for so in simulation_objects)

        # Handle soft fixed joints
        soft_fixed_joints = [rj for rj in simulation_objects if rj.soft_fixed]
        if len(soft_fixed_joints) > 0:
            joint_indices = [j.joint_index for j in soft_fixed_joints]
            body.call(pybullet.setJointMotorControlArray, jointIndices=joint_indices,
                      controlMode=pybullet.VELOCITY_CONTROL, targetVelocities=[0] * len(joint_indices),
                      forces=[0] * len(joint_indices))

        # Handle torque controlled joints
        torque_control_joints = [
            rj for rj in simulation_objects
            if rj.control_mode == JointControlMode.TORQUE_CONTROL and not rj.soft_fixed
        ]
        if len(torque_control_joints) > 0:
            joint_indices, target_velocities, forces = zip(
                *[(j.joint_index, 1e10 * np.sign(j.torque), abs(j.torque)) for j in torque_control_joints])
            # This is a hack to ensure that the joint will hold the torque
            body.call(pybullet.setJointMotorControlArray, jointIndices=joint_indices,
                      controlMode=pybullet.VELOCITY_CONTROL, targetVelocities=target_velocities, forces=forces)

        # Handle velocity controlled joints
        velocity_control_joints = [
            rj for rj in simulation_objects
            if rj.control_mode == JointControlMode.VELOCITY_CONTROL and not rj.soft_fixed
        ]
        if len(velocity_control_joints) > 0:
            joint_indices, target_velocities, forces = zip(
                *[(j.joint_index, j.clamped_target_velocity, j.torque) for j in velocity_control_joints])
            body.call(pybullet.setJointMotorControlArray, jointIndices=joint_indices,
                      controlMode=pybullet.VELOCITY_CONTROL, targetVelocities=target_velocities, forces=forces)

        # Handle position controlled joints
        position_control_joints = [
            rj for rj in simulation_objects
            if rj.control_mode == JointControlMode.POSITION_CONTROL and not rj.soft_fixed
        ]
        if len(position_control_joints) > 0:
            joint_indices, target_positions, target_velocities, forces, v_gains, p_gains = zip(
                *[(j.joint_index, j.target_position, j.clamped_target_velocity, j.torque, j.controller_velocity_gain,
                   j.controller_position_gain) for j in position_control_joints])
            body.call(pybullet.setJointMotorControlArray, jointIndices=joint_indices,
                      controlMode=pybullet.POSITION_CONTROL, targetPositions=target_positions,
                      targetVelocities=target_velocities, forces=forces)
            # velocityGains=v_gains, positionGains=p_gains)
