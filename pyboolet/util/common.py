from typing import Tuple, Sequence

import numpy as np


def decode_field(field):
    if isinstance(field, bytes):
        return field.decode("utf-8")
    elif isinstance(field, Sequence):
        return np.array(field)
    else:
        return field


def decode_pybullet_tuple(pybullet_tuple: Tuple) -> Tuple:
    return tuple(decode_field(s) for s in pybullet_tuple)
