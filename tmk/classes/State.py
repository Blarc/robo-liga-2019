#   tu je implementiran razred "Stanje"

from enum import Enum


class State(Enum):
    """
    Stanja robota.
    """

    def __str__(self):
        return str(self.name)
    GET_APPLE = 0
    GET_TURN = 1
    GET_STRAIGHT = 2
    HOME = 3
    HOME_TURN = 4
    HOME_STRAIGHT = 5
    BACK_OFF = 6
