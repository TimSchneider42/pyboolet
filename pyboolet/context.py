from threading import Lock

from typing import TYPE_CHECKING
from contextvars import ContextVar

if TYPE_CHECKING:
    from typing import List, Optional
    from .physics_client import PhysicsClient

_context_lock = Lock()
_connected_physics_clients: "List[PhysicsClient]" = []
_global_default_physics_client: "Optional[PhysicsClient]" = None
_thread_context_default_physics_client: "ContextVar[Optional[PhysicsClient]]" = ContextVar(
    "_thread_context_default_physics_client", default=None)


def get_default_physics_client(ensure_not_none: bool = False) -> "Optional[PhysicsClient]":
    with _context_lock:
        if _thread_context_default_physics_client.get() is not None:
            output = _thread_context_default_physics_client.get()
        elif _global_default_physics_client in _connected_physics_clients:
            output = _global_default_physics_client
        elif len(_connected_physics_clients) > 0:
            output = _connected_physics_clients[0]
        else:
            output = None
        if ensure_not_none:
            assert output is not None, "No physics client has been conntected yet."
        return output
