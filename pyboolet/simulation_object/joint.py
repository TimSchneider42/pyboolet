from typing import List, TYPE_CHECKING, Callable, NamedTuple, Optional

import pybullet

import numpy as np
from transformation import Transformation

from .simulation_object import SimulationObject
from pyboolet.util import decode_pybullet_tuple
from pyboolet.constraint import Constraint, ConstraintJointType

if TYPE_CHECKING:
    from .link import Link
    from ..simulation_component import Multibody

JointInfo = NamedTuple("JointInfo", (
    ("joint_index", int),
    ("joint_name", str),
    ("joint_type", int),
    ("q_index", int),
    ("u_index", int),
    ("flags", int),
    ("joint_damping", float),
    ("joint_friction", float),
    ("joint_lower_limit", float),
    ("joint_upper_limit", float),
    ("joint_max_force", float),
    ("joint_max_velocity", float),
    ("link_name", str),
    ("joint_axis", np.ndarray),
    ("parent_frame_pos", np.ndarray),
    ("parent_frame_orn", np.ndarray),
    ("parent_index", int),
))

JointState = NamedTuple("JointState", (
    ("joint_position", float),
    ("joint_velocity", float),
    ("joint_reaction_forces", np.ndarray),
    ("applied_joint_motor_torque", float)
))


class Joint(SimulationObject):
    """
    Represents a revolute joint in the simulation.
    """

    def __init__(self, joint_index: int, body: "Multibody", name: str):
        """
        :param joint_index:         Index of this joint in the simulation.
        :param body:                Body this link is part of.
        :param name:                Name of this joint in the simulation.
        """
        self.__joint_index = joint_index
        self.__soft_fix_constraint: Optional[Constraint] = None
        super(Joint, self).__init__(name, body)

    def call(self, func: Callable, *args, **kwargs):
        return self.body.call(func, *args, **kwargs, jointIndex=self.__joint_index)

    @property
    def soft_fixed(self) -> bool:
        return self.__soft_fix_constraint is not None

    @soft_fixed.setter
    def soft_fixed(self, value: bool):
        if not value and self.__soft_fix_constraint is not None:
            self.__soft_fix_constraint.remove()
            self.__soft_fix_constraint = None
        elif value and self.__soft_fix_constraint is None:
            joint_info = self.joint_info
            joint_pose_parent_frame = Transformation.from_pos_quat(
                joint_info.parent_frame_pos, joint_info.parent_frame_orn)
            joint_pose_world_frame = self.parent.pose.transform(joint_pose_parent_frame)
            joint_pose_child_frame = self.child.pose.transform(joint_pose_world_frame, inverse=True)
            parent_frame_pose = Transformation.from_pos_quat(
                joint_info.parent_frame_pos, joint_info.parent_frame_orn)
            self.__soft_fix_constraint = Constraint(
                self.parent, self.child, joint_type=ConstraintJointType.FIXED, joint_axis=joint_info.joint_axis,
                parent_frame_pose=parent_frame_pose, child_frame_pose=joint_pose_child_frame)

    @property
    def joint_index(self) -> int:
        """
        Index of this joint in the simulation.
        :return:
        """
        return self.__joint_index

    @property
    def joint_info(self) -> JointInfo:
        return JointInfo(*decode_pybullet_tuple(self.call(pybullet.getJointInfo)))

    @property
    def joint_state(self):
        return JointState(*decode_pybullet_tuple(self.call(pybullet.getJointState)))

    @property
    def parent(self) -> Optional["Link"]:
        info = self.joint_info
        if info.parent_index == -1:
            return None
        return self.body.links_by_index[info.parent_index]

    @property
    def child(self) -> "Link":
        return self.body.links[self.joint_info.link_name]

    @classmethod
    def retrieve_all_objects(cls, body: "Multibody") -> List["Joint"]:
        from .revolute_joint import RevoluteJoint
        joints = []
        num_joints = body.call(pybullet.getNumJoints)
        pybullet_states = body.call(pybullet.getJointStates, range(num_joints))
        if pybullet_states is not None:
            states = [JointState(*decode_pybullet_tuple(s)) for s in pybullet_states]
            for i, state in enumerate(states):
                info = JointInfo(*decode_pybullet_tuple(body.call(pybullet.getJointInfo, i)))
                if info.joint_type == pybullet.JOINT_REVOLUTE:
                    joints.append(RevoluteJoint(i, body, info.joint_name))
                else:
                    joints.append(Joint(i, body, info.joint_name))
        return joints
