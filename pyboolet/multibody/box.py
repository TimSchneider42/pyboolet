from typing import Sequence, Optional, TYPE_CHECKING

import numpy as np
import pybullet
from transformation import Transformation

from .simple_body import SimpleBody

from pyboolet.shape import VisualShape, CollisionShape

if TYPE_CHECKING:
    from pyboolet import PhysicsClient


class Box(SimpleBody):
    def __init__(self, extents: Sequence[float], rgba_color: Sequence[float] = (0.5, 0.5, 0.5, 1.0),
                 mass: Optional[float] = None, pose: Optional[Transformation] = None,
                 create_visual_shape: bool = True, create_collision_shape: bool = True,
                 physics_client: "Optional[PhysicsClient]" = None):
        assert len(extents) == 3
        assert len(rgba_color) == 4
        self.__extents = np.array(extents)
        self.__rgba_color = rgba_color
        visual_shape = collision_shape = None
        if create_visual_shape:
            visual_shape = VisualShape(pybullet.GEOM_BOX, halfExtents=self.__extents / 2, rgbaColor=rgba_color,
                                       physics_client=physics_client)
        if create_collision_shape:
            collision_shape = CollisionShape(pybullet.GEOM_BOX, halfExtents=self.__extents / 2,
                                             physics_client=physics_client)
        super(Box, self).__init__(collision_shape, visual_shape, mass, pose, physics_client=physics_client)
