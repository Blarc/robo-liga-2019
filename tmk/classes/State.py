#   tu je implementiran razred "Stanje"

from enum import Enum


class State(Enum):
    """
    Stanja robota.
    """

    def __str__(self):
        return str(self.name)
    GET_APPLE = 0
    TURN = 1
    STRAIGHT = 2
    HOME = 3
    BACK_OFF = 4
