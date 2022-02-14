from typing import Sequence, Optional, TYPE_CHECKING

import numpy as np
import pybullet
from transformation import Transformation

from pyboolet.shape import VisualShape, CollisionShape
from .simple_body import SimpleBody

if TYPE_CHECKING:
    from ..physics_client import PhysicsClient


class Sphere(SimpleBody):
    def __init__(self, radius: float, rgba_color: Sequence[float] = (0.5, 0.5, 0.5, 1.0),
                 mass: Optional[float] = None, pose: Optional[Transformation] = None,
                 create_visual_shape: bool = True, create_collision_shape: bool = True,
                 physics_client: "Optional[PhysicsClient]" = None):
        assert len(rgba_color) == 4
        self.__radius = np.array(radius)
        self.__rgba_color = rgba_color
        collision_shape = visual_shape = None
        if create_visual_shape:
            visual_shape = VisualShape(pybullet.GEOM_SPHERE, radius=self.__radius, rgbaColor=rgba_color,
                                       physics_client=physics_client)
        if create_collision_shape:
            collision_shape = CollisionShape(pybullet.GEOM_SPHERE, radius=self.__radius,
                                             physics_client=physics_client)
        super(Sphere, self).__init__(collision_shape, visual_shape, mass, pose, physics_client=physics_client)
