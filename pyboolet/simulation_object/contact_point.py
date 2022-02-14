from typing import NamedTuple
import numpy as np

ContactPoint = NamedTuple(
    "ContactPoint",
    (("contact_flag", int),
     ("body_unique_id_a", int),
     ("body_unique_id_b", int),
     ("link_index_a", int),
     ("link_index_b", int),
     ("position_on_a", np.ndarray),
     ("position_on_b", np.ndarray),
     ("contact_normal_on_b", np.ndarray),
     ("contact_distance", float),
     ("normal_force", float),
     ("lateral_friction_1", float),
     ("lateral_friction_dir_1", np.ndarray),
     ("lateral_friction_2", float),
     ("lateral_friction_dir_2", np.ndarray)))
