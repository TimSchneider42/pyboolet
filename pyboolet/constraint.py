from enum import Enum
from typing import Optional, TYPE_CHECKING

import numpy as np

import pybullet
from transformation import Transformation

from .simulation_component import SimulationComponent

if TYPE_CHECKING:
    from .simulation_object import Link
    from .physics_client import PhysicsClient


class ConstraintJointType(Enum):
    PRISMATIC = pybullet.JOINT_PRISMATIC
    FIXED = pybullet.JOINT_FIXED
    POINT2POINT = pybullet.JOINT_POINT2POINT
    GEAR = pybullet.JOINT_GEAR


class Constraint(SimulationComponent):
    def __init__(self, parent_link: "Link", child_link: "Link", joint_type: ConstraintJointType, joint_axis: np.ndarray,
                 parent_frame_pose: Transformation, child_frame_pose: Transformation,
                 physics_client: Optional["PhysicsClient"] = None):
        assert joint_axis.shape == (3,)
        super(Constraint, self).__init__(physics_client)
        self.__unique_id = self.physics_client.call(
            pybullet.createConstraint, parent_link.body.unique_id, parent_link.link_index, child_link.body.unique_id,
            child_link.link_index, joint_type.value, joint_axis, parent_frame_pose.translation,
            child_frame_pose.translation, parent_frame_pose.quaternion, child_frame_pose.quaternion)

    def _cleanup(self):
        self.physics_client.call(pybullet.removeConstraint, self.__unique_id)

    @property
    def unique_id(self):
        return self.__unique_id
