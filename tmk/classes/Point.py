# tu je implementiran razred "Point"


class Point:
    """
    Točka na poligonu.
    """

    def __init__(self, position):
        self.x = position[0]
        self.y = position[1]

    def __str__(self):
        return '('+str(self.x)+', '+str(self.y)+')'
