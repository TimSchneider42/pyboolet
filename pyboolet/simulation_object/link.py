from typing import List, Optional, TYPE_CHECKING, Callable, Tuple, NamedTuple, Union

import numpy as np
import pybullet
from transformation import Transformation

from pyboolet.util import decode_pybullet_tuple
from .simulation_object import SimulationObject
from .contact_point import ContactPoint

if TYPE_CHECKING:
    from ..multibody import Multibody
    from .joint import Joint

DynamicsInfo = NamedTuple(
    "DynamicsInfo",
    (("mass", float),
     ("lateral_friction", float),
     ("local_inertia_diagonal", np.ndarray),
     ("local_inertial_pos", np.ndarray),
     ("local_inertial_orn", np.ndarray),
     ("restitution", float),
     ("rolling_friction", float),
     ("spinning_friction", float),
     ("contact_damping", float),
     ("contact_stiffness", float),
     ("body_type", int),
     ("collision_margin", float)))


class Link(SimulationObject):
    """
    Represents a link in the simulation.
    """

    def __init__(self, link_index: int, name: str, body: "Multibody"):
        """

        :param link_index:          Index of this link in the simulation.
        :param name:                Name of this link in the simulation.
        :param body:                Body this link is part of.
        """
        self.__observed_position = None  # type: Optional[np.ndarray]
        self.__observed_quaternion = None  # type: Optional[np.ndarray]
        self.__link_index = link_index
        self.__actual_mass = None  # used to store the mass of the object when it is dynamically disabled
        self.__collision_mask = 0xFFFF
        self.__collision_group = 0xFFFF
        super(Link, self).__init__(name, body)

    def call(self, func: Callable, *args, **kwargs):
        return self.body.call(func, *args, **kwargs, linkIndex=self.__link_index)

    @property
    def dynamics_info(self):
        return DynamicsInfo(*decode_pybullet_tuple(self.call(pybullet.getDynamicsInfo)))

    @property
    def static(self):
        return self.dynamics_info.mass == 0

    def set_collidable(self, value: bool):
        group = self.__collision_group
        mask = self.__collision_mask * value
        self.body.call(pybullet.setCollisionFilterGroupMask, linkIndexA=self.__link_index,
                       collisionFilterGroup=group, collisionFilterMask=mask)

    def set_collision_mask(self, collision_filter_group: int, collision_filter_mask: int):
        self.__collision_group = collision_filter_group
        self.__collision_mask = collision_filter_mask
        self.body.call(pybullet.setCollisionFilterGroupMask, linkIndexA=self.__link_index,
                       collisionFilterGroup=collision_filter_group, collisionFilterMask=collision_filter_mask)

    def get_contact_points(self, other: Optional[Union["Link", "Multibody"]] = None) -> List[ContactPoint]:
        from ..multibody import Multibody
        kwargs = {
            "bodyA": self.body.unique_id,
            "linkIndexA": self.__link_index,
        }
        if isinstance(other, Link):
            kwargs.update({
                "bodyB": other.body.unique_id,
                "linkIndexB": other.__link_index
            })
        elif isinstance(other, Multibody):
            kwargs.update({
                "bodyB": other.unique_id
            })
        return [ContactPoint(*cp) for cp in self.body.physics_client.call(pybullet.getContactPoints, **kwargs)]

    def get_closest_points(self, other: Union["Link", "Multibody"], max_distance: float) -> List[ContactPoint]:
        kwargs = {
            "bodyA": self.body.unique_id,
            "linkIndexA": self.__link_index,
            "distance": max_distance
        }
        if isinstance(other, Link):
            kwargs.update({
                "bodyB": other.body.unique_id,
                "linkIndexB": other.__link_index
            })
        else:
            kwargs.update({
                "bodyB": other.unique_id,
                "linkIndexB": -1
            })
        return [ContactPoint(*cp) for cp in self.body.physics_client.call(pybullet.getClosestPoints, **kwargs)]

    def change_dynamics(
            self, mass: Optional[float] = None, lateral_friction: Optional[float] = None,
            spinning_friction: Optional[float] = None, restitution: Optional[float] = None,
            linear_damping: Optional[float] = None, angular_damping: Optional[float] = None,
            contact_stiffness: Optional[float] = None, contact_damping: Optional[float] = None,
            friction_anchor: Optional[int] = None, local_interia_diagonal: Optional[np.ndarray] = None,
            ccd_swept_sphere_radius: Optional[float] = None, contact_processing_threshold: Optional[float] = None,
            activation_state: Optional[int] = None, joint_damping: Optional[float] = None,
            anisotropic_friction: Optional[float] = None, collision_margin: Optional[float] = None):
        kwargs = {
            "mass": mass,
            "lateralFriction": lateral_friction,
            "spinningFriction": spinning_friction,
            "restitution": restitution,
            "linearDamping": linear_damping,
            "angularDamping": angular_damping,
            "contactStiffness": contact_stiffness,
            "contactDamping": contact_damping,
            "frictionAnchor": friction_anchor,
            "localInteriaDiagonal": local_interia_diagonal,
            "ccdSweptSphereRadius": ccd_swept_sphere_radius,
            "contactProcessingThreshold": contact_processing_threshold,
            "activationState": activation_state,
            "jointDamping": joint_damping,
            "anisotropicFriction": anisotropic_friction,
            "collisionMargin": collision_margin,
        }
        kwargs_filtered = {k: v for k, v in kwargs.items() if v is not None}
        self.call(pybullet.changeDynamics, **kwargs_filtered)

    @static.setter
    def static(self, value: bool):
        if value:
            if self.__actual_mass is None:
                self.__actual_mass = self.dynamics_info.mass
            self.change_dynamics(mass=0)
        else:
            if self.dynamics_info.mass == 0 and self.__actual_mass is not None:
                self.change_dynamics(mass=self.__actual_mass)
                self.__actual_mass = None

    @property
    def pose(self) -> Transformation:
        pos, quat, *_ = self.call(pybullet.getLinkState)
        return Transformation.from_pos_quat(pos, quat)

    @property
    def velocity(self) -> Tuple[np.ndarray, np.ndarray]:
        state = self.call(pybullet.getLinkState, computeLinkVelocity=True)
        return np.array(state[6]), np.array(state[7])

    @property
    def link_index(self) -> int:
        """
        Index of this link in the simulation.
        :return:
        """
        return self.__link_index

    @property
    def parent_joint(self) -> "Joint":
        return self.body.joints_by_child[self]

    @property
    def child_joints(self) -> Tuple["Joint"]:
        return self.body.joints_by_parent[self]

    @classmethod
    def retrieve_all_objects(cls, body: "Multibody") -> List["Link"]:
        # This is a ugly hack as it is not possible to get all links directly. The only way of accessing link names is
        # by calling getJointInfo, which will return the parent link index and the child link name, however for some
        # reason it won't return parent link name and child link index. So this is all about guessing.
        links: List[Link] = []
        num_joints = body.call(pybullet.getNumJoints)
        for i in range(num_joints):
            info = body.call(pybullet.getJointInfo, i)
            # Just assume that the index is always the next free index
            links.append(Link(len(links), info[12].decode("utf-8"), body))
        return links
