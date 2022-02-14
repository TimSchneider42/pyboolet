from pathlib import Path
from typing import Optional, TYPE_CHECKING, Union

import pybullet
from transformation import Transformation

from .multibody import Multibody

if TYPE_CHECKING:
    from pyboolet import PhysicsClient


class URDFBody(Multibody):
    def __init__(self, urdf_filename: Union[Path, str], base_pose: Optional[Transformation] = None,
                 use_maximal_coordinates: bool = False,
                 use_fixed_base: bool = False, flags: int = 0, global_scaling: float = 1.0,
                 physics_client: "Optional[PhysicsClient]" = None):
        self.__urdf_filename = Path(urdf_filename)
        self.__base_pose = base_pose if base_pose is not None else Transformation()
        self.__use_maximal_coordinates = use_maximal_coordinates
        self.__use_fixed_base = use_fixed_base
        self.__flags = flags
        self.__global_scaling = global_scaling
        super(URDFBody, self).__init__(physics_client)

    def _setup(self) -> int:
        return self.physics_client.call(
            pybullet.loadURDF, str(self.__urdf_filename), self.__base_pose.translation, self.__base_pose.quaternion,
            self.__use_maximal_coordinates, self.__use_fixed_base, self.__flags, self.__global_scaling)
