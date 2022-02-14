from typing import Optional, TYPE_CHECKING

from . import context

if TYPE_CHECKING:
    from .physics_client import PhysicsClient


class SimulationComponent:
    def __init__(self, physics_client: Optional["PhysicsClient"] = None):
        if physics_client is None:
            physics_client = context.get_default_physics_client(ensure_not_none=True)
        self.__physics_client: PhysicsClient = physics_client
        self.__physics_client._register_component(self)
        self.__registered = True

    def remove(self):
        self._check_registered()
        self._cleanup()
        self.__physics_client._unregister_component(self)
        self._set_unregistered()

    def _cleanup(self):
        pass

    def _set_unregistered(self):
        self.__registered = False

    def _check_registered(self):
        assert self.__registered, "The object does not exist in the simulation anymore."

    @property
    def registered(self) -> bool:
        return self.__registered

    @property
    def physics_client(self) -> "PhysicsClient":
        return self.__physics_client
