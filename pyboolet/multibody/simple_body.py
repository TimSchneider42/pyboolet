from typing import Optional, TYPE_CHECKING, Union

import pybullet
from transformation import Transformation

from .multibody import Multibody
from pyboolet.shape import CollisionShape, CollisionShapeArray, VisualShape, VisualShapeArray

if TYPE_CHECKING:
    from pyboolet import PhysicsClient


class SimpleBody(Multibody):
    def __init__(self, collision_shape: Optional[Union[CollisionShape, CollisionShapeArray]],
                 visual_shape: Optional[Union[VisualShape, VisualShapeArray]],
                 mass: Optional[float] = None, pose: Optional[Transformation] = None,
                 physics_client: "Optional[PhysicsClient]" = None):
        if collision_shape is not None:
            assert isinstance(collision_shape, CollisionShape) or isinstance(collision_shape, CollisionShapeArray)
        if visual_shape is not None:
            assert isinstance(visual_shape, VisualShape) or isinstance(visual_shape, VisualShapeArray)
        self.__visual_shape = visual_shape
        self.__collision_shape = collision_shape
        self.__mass = mass
        self.__pose = pose
        super(SimpleBody, self).__init__(physics_client)

    def _setup(self) -> int:
        if self.__visual_shape is not None:
            assert self.__visual_shape.physics_client == self.physics_client
        if self.__collision_shape is not None:
            assert self.__collision_shape.physics_client == self.physics_client
        assert self.__collision_shape is not None or self.__visual_shape is not None

        if self.__pose is not None:
            pos = self.__pose.translation
            quat = self.__pose.quaternion
        else:
            pos = quat = None
        self.__kwargs = {k: v for k, v in {
            "baseMass": self.__mass,
            "baseCollisionShapeIndex": None if self.__collision_shape is None else self.__collision_shape.unique_id,
            "baseVisualShapeIndex": None if self.__visual_shape is None else self.__visual_shape.unique_id,
            "basePosition": pos,
            "baseOrientation": quat
        }.items() if v is not None}
        return self.physics_client.call(pybullet.createMultiBody, **self.__kwargs)
