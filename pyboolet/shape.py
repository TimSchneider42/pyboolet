from typing import Optional, TYPE_CHECKING, Union, Sequence, Callable

import pybullet

from .simulation_component import SimulationComponent

if TYPE_CHECKING:
    from .physics_client import PhysicsClient


class Shape(SimulationComponent):
    def __init__(self, creation_function: Callable, shape_type: Union[int, Sequence[int]], *args,
                 physics_client: Optional["PhysicsClient"] = None, **kwargs):
        super(Shape, self).__init__(physics_client)
        self.__unique_id = self.physics_client.call(creation_function, shape_type, *args, **kwargs)
        assert self.__unique_id != -1

    def _cleanup(self):
        raise NotImplementedError("PyBullet currently does not provide a way of removing shapes.")

    @property
    def unique_id(self) -> int:
        return self.__unique_id


class CollisionShape(Shape):
    def __init__(self, shape_type: Union[int, Sequence[int]], *args, physics_client: Optional["PhysicsClient"] = None,
                 **kwargs):
        super(CollisionShape, self).__init__(
            pybullet.createCollisionShape, shape_type, *args, physics_client=physics_client, **kwargs)


class CollisionShapeArray(Shape):
    def __init__(self, shape_type: Union[int, Sequence[int]], *args, physics_client: Optional["PhysicsClient"] = None,
                 **kwargs):
        super(CollisionShapeArray, self).__init__(
            pybullet.createCollisionShapeArray, shape_type, *args, physics_client=physics_client, **kwargs)


class VisualShape(Shape):
    def __init__(self, shape_type: Union[int, Sequence[int]], *args, physics_client: Optional["PhysicsClient"] = None,
                 **kwargs):
        super(VisualShape, self).__init__(
            pybullet.createVisualShape, shape_type, *args, physics_client=physics_client, **kwargs)


class VisualShapeArray(Shape):
    def __init__(self, shape_type: Union[int, Sequence[int]], *args, physics_client: Optional["PhysicsClient"] = None,
                 **kwargs):
        super(VisualShapeArray, self).__init__(
            pybullet.createVisualShapeArray, shape_type, *args, physics_client=physics_client, **kwargs)
