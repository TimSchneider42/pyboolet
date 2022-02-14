import pybullet
from typing import List, NamedTuple, Sequence
from typing import Optional, Tuple

import numpy as np

from . import context

from .multibody import Multibody
from .default_physics_client_manager import DefaultPhysicsClientManager
from .simulation_component import SimulationComponent

RayIntersection = NamedTuple("RayIntersection", (
    ("object_unique_id", int),
    ("link_index", int),
    ("hit_fraction", float),
    ("hit_position", np.ndarray),
    ("hit_normal", np.ndarray)))


class PhysicsClient:
    def __init__(self):
        self.__physics_client_id: Optional[int] = None
        self.__bodies: List[Multibody] = []
        self.__simulation_components: List[SimulationComponent] = []
        self.__real_time_simulation = False
        self.__gravity = np.zeros(3)

    def __connect(self, *args, **kwargs):
        assert not self.is_connected, "This client has already been connected."
        self.__physics_client_id = pybullet.connect(*args, **kwargs)
        with context._context_lock:
            context._connected_physics_clients.append(self)

    def connect_direct(self):
        self.__connect(pybullet.DIRECT)

    def connect_gui(self, options: str = ""):
        self.__connect(pybullet.GUI, options=options)

    def connect_udp(self, hostname: str, port: Optional[int] = None):
        if port is not None:
            self.__connect(pybullet.UDP, hostname, port)
        else:
            self.__connect(pybullet.UDP, hostname)

    def connect_tcp(self, hostname: str, port: Optional[int] = None):
        if port is not None:
            self.__connect(pybullet.TCP, hostname, port)
        else:
            self.__connect(pybullet.TCP, hostname)

    def connect_shared_memory(self, key: Optional[int] = None):
        if key is not None:
            self.__connect(pybullet.SHARED_MEMORY, key=key)
        else:
            self.__connect(pybullet.SHARED_MEMORY)

    def disconnect(self):
        self.reset_simulation()
        self.call(pybullet.disconnect)
        self.__physics_client_id = None
        with context._context_lock:
            context._connected_physics_clients.remove(self)

    def reset_simulation(self):
        for component in self.__simulation_components:
            component._set_unregistered()
        self.__bodies.clear()
        self.__simulation_components.clear()
        self.call(pybullet.resetSimulation)

    def call(self, func, *args, **kwargs):
        assert self.is_connected, "This client has not yet been connected."
        assert "physicsClientId" not in kwargs, \
            "The argument physicsClientId will be filled in by the PhysicsClient instance and thus needs to be left " \
            "empty."
        args_no_numpy = [v if not isinstance(v, np.ndarray) else v.tolist() for v in args]
        kwargs_no_numpy = {
            k: v if not isinstance(v, np.ndarray) else v.tolist() for k, v in kwargs.items()
        }
        return func(*args_no_numpy, **kwargs_no_numpy, physicsClientId=self.physics_client_id)

    def _register_component(self, simulation_component: SimulationComponent):
        self.__simulation_components.append(simulation_component)
        if isinstance(simulation_component, Multibody):
            self.__bodies.append(simulation_component)

    def _unregister_component(self, simulation_component: SimulationComponent):
        self.__simulation_components.remove(simulation_component)
        if isinstance(simulation_component, Multibody):
            self.__bodies.remove(simulation_component)

    def step_simulation(self):
        for body in self.__bodies:
            body._act()
        self.call(pybullet.stepSimulation)

    def reset_debug_visualizer_camera(self, camera_distance: float, camera_yaw: float, camera_pitch: float,
                                      camera_target_position: np.ndarray):
        self.call(pybullet.resetDebugVisualizerCamera, camera_distance, camera_yaw, camera_pitch,
                  camera_target_position)

    def configure_debug_visualizer(self, flag: int, enable: bool):
        self.call(pybullet.configureDebugVisualizer, flag, enable)

    def set_additional_search_path(self, path: str):
        self.call(pybullet.setAdditionalSearchPath, path)

    def ray_test_batch(self, ray_from_positions: np.ndarray, ray_to_positions: np.ndarray,
                       parent_object_unique_id: int = -1, parent_link_index: int = -1, num_threads: int = -1,
                       report_hit_number: bool = 0, collision_filter_mask: int = 0xFFFF,
                       fraction_epsilon: float = -1) -> List[RayIntersection]:
        return [
            RayIntersection(*[e if not isinstance(e, Sequence) else np.array(e) for e in r])
            for r in self.call(pybullet.rayTestBatch, ray_from_positions, ray_to_positions, parent_object_unique_id,
                               parent_link_index, num_threads, report_hit_number, collision_filter_mask,
                               fraction_epsilon)
        ]

    def as_default(self) -> DefaultPhysicsClientManager:
        return DefaultPhysicsClientManager(self)

    def set_as_default(self):
        assert self.is_connected
        with context._context_lock:
            context._global_default_physics_client = self

    @property
    def gravity(self):
        return self.__gravity

    @gravity.setter
    def gravity(self, value: np.ndarray):
        assert value.shape == (3,)
        self.__gravity = value
        self.call(pybullet.setGravity, *value)

    @property
    def real_time_simulation(self) -> bool:
        return self.__real_time_simulation

    @real_time_simulation.setter
    def real_time_simulation(self, value: bool):
        self.__real_time_simulation = value
        self.call(pybullet.setRealTimeSimulation, value)

    @property
    def time_step(self) -> float:
        return self.call(pybullet.getPhysicsEngineParameters)["fixedTimeStep"]

    @time_step.setter
    def time_step(self, value: float):
        self.call(pybullet.setTimeStep, value)

    @property
    def physics_client_id(self) -> Optional[int]:
        return self.__physics_client_id

    @property
    def is_connected(self) -> bool:
        return self.__physics_client_id is not None

    @property
    def bodies(self) -> Tuple[Multibody, ...]:
        return tuple(self.__bodies)

    def __del__(self):
        if self.is_connected:
            self.disconnect()
