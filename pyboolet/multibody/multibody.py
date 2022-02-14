from abc import ABC, abstractmethod
from itertools import groupby
from typing import Callable, List, Tuple, Union
from typing import Optional, TYPE_CHECKING

import numpy as np
import pybullet
from transformation import Transformation

from pyboolet.simulation_component import SimulationComponent
from pyboolet.simulation_object import ContactPoint, Joint, DynamicsInfo, SimulationObject, Link, RevoluteJoint

from pyboolet.util import ReadOnlyDict


if TYPE_CHECKING:
    from pyboolet.physics_client import PhysicsClient


class Multibody(SimulationComponent, ABC):
    def __init__(self, physics_client: Optional["PhysicsClient"] = None):
        super(Multibody, self).__init__(physics_client)
        try:
            self.__unique_id: Optional[int] = self._setup()

            joints = Joint.retrieve_all_objects(self)
            self.__joints = ReadOnlyDict({rj.name: rj for rj in joints})
            self.__joints_by_index = ReadOnlyDict({rj.joint_index: rj for rj in joints})

            self.__revolute_joints = tuple(sorted(
                [j for j in self.__joints.values() if isinstance(j, RevoluteJoint)], key=lambda j: j.joint_index))

            links = Link.retrieve_all_objects(self)
            self.__links = ReadOnlyDict({l.name: l for l in links})
            self.__links_by_index = ReadOnlyDict({l.link_index: l for l in links})

            self.__joints_by_child = ReadOnlyDict({rj.child: rj for rj in joints})
            joints_by_parent = {p: () for p in self.__links.values()}
            joints_by_parent.update(
                {p: tuple(js) for p, js in groupby(sorted(joints, key=lambda j: hash(j.parent)), key=lambda j: j.parent)})
            self.__joints_by_parent = ReadOnlyDict(joints_by_parent)
            root_links = [l for l in links if l.parent_joint.parent is None]
            if len(root_links) != 0:
                self.__root_link = root_links[0]
            else:
                self.__root_link = None

            all_simulation_objects = {}
            all_simulation_objects.update(self.__joints)
            all_simulation_objects.update(self.__links)
            self.__simulation_objects = ReadOnlyDict(all_simulation_objects)
        except:
            try:
                self._cleanup()
            except:
                pass
            raise

        self.__actual_mass = None
        self.__base_collision_group = 0xFFFF
        self.__base_collision_mask = 0xFFFF

    def call(self, func: Callable, *args, **kwargs):
        self._check_registered()
        return self.physics_client.call(func, self.__unique_id, *args, **kwargs)

    @abstractmethod
    def _setup(self) -> int:
        """
        Sets up this body in the simulation and returns the body ID.
        :return:
        """
        pass

    def _cleanup(self):
        self.call(pybullet.removeBody)

    def reset_pose(self, pose: Transformation):
        self.call(pybullet.resetBasePositionAndOrientation, posObj=pose.translation, ornObj=pose.quaternion)

    def reset_velocity(self, linear_velocity: np.ndarray, angular_velocity: np.ndarray):
        self.call(pybullet.resetBaseVelocity, linearVelocity=linear_velocity, angularVelocity=angular_velocity)

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
        self.call(pybullet.changeDynamics, linkIndex=-1, **kwargs_filtered)

    def get_contact_points(self, other: Optional[Union[Link, "Multibody"]] = None) -> List[ContactPoint]:
        kwargs = {
            "bodyA": self.__unique_id,
        }
        if isinstance(other, Link):
            kwargs.update({
                "bodyB": other.body.unique_id,
                "linkIndexB": other.link_index
            })
        elif isinstance(other, Multibody):
            kwargs.update({
                "bodyB": other.__unique_id
            })
        return [ContactPoint(*cp) for cp in self.physics_client.call(pybullet.getContactPoints, **kwargs)]

    def get_closest_points(self, other: Union["Link", "Multibody"], max_distance: float) -> List[ContactPoint]:
        kwargs = {
            "bodyA": self.__unique_id,
            "linkIndexA": -1,
            "distance": max_distance
        }
        if isinstance(other, Link):
            kwargs.update({
                "bodyB": other.body.unique_id,
                "linkIndexB": other.link_index
            })
        else:
            kwargs.update({
                "bodyB": other.unique_id,
                "linkIndexB": -1
            })
        return [ContactPoint(*cp) for cp in self.physics_client.call(pybullet.getClosestPoints, **kwargs)]

    def set_collidable(self, value: bool):
        group = self.__base_collision_group
        mask = self.__base_collision_mask * value
        self.call(pybullet.setCollisionFilterGroupMask, linkIndexA=-1, collisionFilterGroup=group,
                  collisionFilterMask=mask)

    def set_collision_mask(self, collision_filter_group: int, collision_filter_mask: int):
        self.__base_collision_group = collision_filter_group
        self.__base_collision_mask = collision_filter_mask
        self.call(pybullet.setCollisionFilterGroupMask, linkIndexA=-1, collisionFilterGroup=collision_filter_group,
                  collisionFilterMask=collision_filter_mask)

    @property
    def unique_id(self) -> int:
        """
        Body ID of this body in the simulation.
        :return:
        """
        return self.__unique_id

    @property
    def simulation_objects(self) -> ReadOnlyDict[str, SimulationObject]:
        return self.__simulation_objects

    @property
    def revolute_joints(self) -> Tuple[RevoluteJoint, ...]:
        return self.__revolute_joints

    @property
    def joints(self) -> ReadOnlyDict[str, Joint]:
        return self.__joints

    @property
    def links(self) -> ReadOnlyDict[str, Link]:
        return self.__links

    @property
    def root_link(self) -> Link:
        return self.__root_link

    @property
    def links_by_index(self) -> ReadOnlyDict[int, Link]:
        return self.__links_by_index

    @property
    def joints_by_index(self) -> ReadOnlyDict[int, Joint]:
        return self.__joints_by_index

    @property
    def joints_by_child(self) -> ReadOnlyDict[Link, Joint]:
        return self.__joints_by_child

    @property
    def joints_by_parent(self) -> ReadOnlyDict[Link, Tuple[Joint, ...]]:
        return self.__joints_by_parent

    @property
    def pose(self) -> Transformation:
        pos, quat = self.call(pybullet.getBasePositionAndOrientation)
        return Transformation.from_pos_quat(pos, quat)

    @property
    def velocity(self) -> Tuple[np.ndarray, np.ndarray]:
        lin, ang = self.call(pybullet.getBaseVelocity)
        return np.array(lin), np.array(ang)

    @property
    def dynamics_info(self) -> DynamicsInfo:
        return DynamicsInfo(*self.call(pybullet.getDynamicsInfo, -1))

    @property
    def base_static(self) -> bool:
        return self.dynamics_info.mass == 0

    @base_static.setter
    def base_static(self, value: bool):
        if value:
            if self.__actual_mass is None:
                self.__actual_mass = self.dynamics_info.mass
            self.change_dynamics(mass=0)
        else:
            if self.dynamics_info.mass == 0 and self.__actual_mass is not None:
                self.change_dynamics(mass=self.__actual_mass)
                self.__actual_mass = None

    def set_base_collidable(self, value: bool):
        group = 0xFFFF * value
        mask = group
        self.call(pybullet.setCollisionFilterGroupMask, linkIndexA=-1,
                  collisionFilterGroup=group, collisionFilterMask=mask)

    # ==================================================================================================================
    # Interface to communicate with PhysicsClient. Do not call these functions directly.

    def _act(self):
        SimulationObject._act_all(self, self.simulation_objects.values())
