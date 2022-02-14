from abc import ABC
from typing import List, TYPE_CHECKING, Iterable

import itertools
from typing import Type

if TYPE_CHECKING:
    from pyboolet.multibody import Multibody


class SimulationObject(ABC):
    """
    Represents an object in the simulation.
    """

    def __init__(self, name: str, body: "Multibody"):
        """
        :param name:    Name of the object in the simulator.
        """
        self.__name = name
        self.__body = body

    @property
    def name(self) -> str:
        """
        Name of the object in the simulator.
        :return:
        """
        return self.__name

    @property
    def body(self) -> "Multibody":
        return self.__body

    def __repr__(self) -> str:
        return self.name

    @classmethod
    def _act_all(cls, body: "Multibody", simulation_objects: Iterable["SimulationObject"]):
        """
        A call to this function will make all SimulationObject instances in the given list act in the simulation.
        This means that for each type of SimulationObject in the list, the _act function is called once with all
        instances of that type in the list.
        :param simulation_objects:  SimulationObject instances to act.
        :return:
        """
        # Group by type and iterate
        for t, sim_objs in itertools.groupby(simulation_objects, type):  # type: Type[SimulationObject]
            t._act(body, list(sim_objs))

    @classmethod
    def _act(cls, body: "Multibody", simulation_objects: List["SimulationObject"]):
        """
        Makes all SimulationObject instances in the given list act. This function is supposed to be overridden by a
        subclass if needed.
        :param body:     Body of all given simulation_objects.
        :param simulation_objects:  SimulationObjects to act (expected to be of the type of the overriding subclass).
        :return:
        """
        pass
