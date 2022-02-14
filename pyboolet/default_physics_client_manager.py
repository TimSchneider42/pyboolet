from typing import TYPE_CHECKING

from . import context

if TYPE_CHECKING:
    from .physics_client import PhysicsClient


class DefaultPhysicsClientManager:
    def __init__(self, physics_client: "PhysicsClient"):
        self.__physics_client = physics_client
        self.__prev_default_client = None

    def __enter__(self):
        assert self.__prev_default_client is None, "Cannot enter DefaultPhysicsClientManager twice"
        with context._context_lock:
            self.__prev_default_client = context._thread_context_default_physics_client.get()
            context._thread_context_default_physics_client.set(self.__physics_client)

    def __exit__(self, exc_type, exc_val, exc_tb):
        with context._context_lock:
            context._thread_context_default_physics_client.set(self.__prev_default_client)
            self.__prev_default_client = None
