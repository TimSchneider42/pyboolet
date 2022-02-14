from typing import Union, Sequence, Optional, List, Dict

import numpy as np
from scipy.spatial.transform import Rotation


class Transformation:
    def __init__(self, translation: Optional[Sequence[float]] = None, rotation: Optional[Rotation] = None):
        self.__translation = np.zeros((3,)) if translation is None else np.array(translation)
        assert rotation is None or isinstance(rotation, Rotation)
        self.__rotation = Rotation.from_quat(np.array([0, 0, 0, 1])) if rotation is None else rotation

    def transform(self, other: Union["Transformation", Sequence["Transformation"], np.ndarray], inverse: bool = False) \
            -> Union["Transformation", Sequence["Transformation"], np.ndarray]:
        if isinstance(other, np.ndarray):
            return self._transform_positions(other, inverse=inverse)

        if isinstance(other, Sequence):
            transformation_lst = other
        else:
            transformation_lst = [other]

        pos, rot = zip(*[(t.translation, t.rotation) for t in transformation_lst])

        output_pos = self._transform_positions(np.array(pos), inverse=inverse)
        if inverse:
            output_rot = [self.__rotation.inv() * r for r in rot]
        else:
            output_rot = [self.__rotation * r for r in rot]

        output_transformations = [Transformation(p, r) for p, r in zip(output_pos, output_rot)]

        if isinstance(other, Sequence):
            return output_transformations
        else:
            return output_transformations[0]

    def copy(self) -> "Transformation":
        return Transformation.from_matrix(self.matrix)

    def _transform_positions(self, positions: np.ndarray, inverse: bool = False):
        if not inverse:
            return self.__rotation.apply(positions) + self.__translation
        else:
            return self.__rotation.apply((positions - self.__translation), inverse=True)

    def __mul__(self, other: Union["Transformation", Sequence["Transformation"], np.ndarray]):
        return self.transform(other)

    @property
    def rotation(self) -> Rotation:
        return self.__rotation

    @property
    def translation(self) -> np.ndarray:
        return self.__translation

    @property
    def quaternion(self) -> np.ndarray:
        return self.__rotation.as_quat()

    @property
    def angle(self) -> float:
        q = self.__rotation.as_quat()
        return np.abs(2 * np.arctan2(np.linalg.norm(q[:-1]), np.abs(q[-1])))

    @property
    def rotvec(self) -> np.ndarray:
        return self.__rotation.as_rotvec()

    @property
    def matrix(self) -> np.ndarray:
        output = np.zeros((4, 4))
        output[:3, :3] = self.__rotation.as_matrix()
        output[3, 3] = 1
        output[:3, 3] = self.__translation
        return output

    @property
    def inv(self) -> "Transformation":
        pos = - self.__rotation.apply(self.__translation, inverse=True)
        rot = self.__rotation.inv()
        return Transformation(pos, rot)

    @classmethod
    def from_pos_quat(cls, position: Optional[Sequence[float]] = None,
                      quaternion: Optional[Sequence[float]] = None) -> "Transformation":
        return cls(position, None if quaternion is None else Rotation.from_quat(quaternion))

    @classmethod
    def from_pos_euler(cls, position: Optional[Sequence[float]] = None,
                       euler_angles: Optional[Sequence[float]] = None, sequence: str = "xyz") -> "Transformation":
        return cls(position, None if euler_angles is None else Rotation.from_euler(sequence, euler_angles))

    @classmethod
    def from_pos_rotvec(cls, position: Optional[Sequence[float]] = None,
                        rotvec: Optional[Sequence[float]] = None) -> "Transformation":
        return cls(position, None if rotvec is None else Rotation.from_rotvec(rotvec))

    @classmethod
    def from_matrix(cls, matrix: Union[List, np.ndarray]) -> "Transformation":
        if not isinstance(matrix, np.ndarray):
            matrix = np.array(matrix)
        translation = matrix[:3, 3]
        rotation = Rotation.from_matrix(matrix[:3, :3])
        return cls(translation, rotation)

    def __repr__(self):
        return "Transformation({}, {})".format(self.__translation.tolist(), self.__rotation.as_quat().tolist())

    def to_dict(self):
        return {
            "translation": self.__translation.tolist(),
            "rotation": self.__rotation.as_quat().tolist()
        }

    @classmethod
    def from_dict(cls, transformation_dict: Dict[str, Sequence[float]]) -> "Transformation":
        return Transformation.from_pos_quat(transformation_dict["translation"], transformation_dict["rotation"])
