from graphics import *
from enum import Enum
import heapq

ACT_WIDTH = 3555
ACT_HEIGHT = 2055
WIN_WIDTH = 1185  # 3555 / 3  3555 / 15 = 273
WIN_HEIGHT = 685  # 2055 / 3  2055 / 15 = 137
NODE_WIDTH = 273
NODE_HEIGHT = 137
FACTOR = 15


class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]


class NodeType(Enum):
    def __str__(self):
        return str(self.name)

    UNCHECKED = 0
    CHECKED = 1
    GOOD_APPLE = 2
    BAD_APPLE = 3


class Node:
    """
    g_cost = distance from starting node
    h_cost = distance from end node
    f_cost = g_cost + h_cost
    """
    g_cost = 0
    h_cost = 0
    f_cost = 0
    parent = None
    type = NodeType.UNCHECKED

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def set(self, g_cost, h_cost, parent):
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.f_cost = g_cost + h_cost
        self.parent = parent


def calc_cost(curr: Node, end: Node):
    x = abs(end.x - curr.x)
    y = abs(end.y - curr.y)

    # x bigger, y smaller
    if x < y:
        # x smaller, y bigger
        temp = x
        x = y
        y = temp
    return y * 14 + (x - y) * 10


def draw_map(win):
    for i in game:
        for j in i:
            if j.type == NodeType.GOOD_APPLE:
                a = Point(j.x, j.y)
                b = Point(j.x + 5, j.y + 5)
                r = Rectangle(a, b)
                r.setFill(color_rgb(255, 0, 255))
                r.draw(win)
            elif j.type == NodeType.CHECKED:
                a = Point(j.x, j.y)
                b = Point(j.x + 5, j.y + 5)
                r = Rectangle(a, b)
                r.draw(win)


def put_apple(pos: Point, apple_type: NodeType):
    """
    :param pos: sredisce jabolka v milimetrih
    :param apple_type: tip jabolka
    :return: pretvori enote in jabolko doda na mapo
    """
    x = int(pos.x - 50) // FACTOR
    y = int(pos.y - 50) // FACTOR
    for i in range(x, x + 7):
        for j in range(y, y + 7):
            game[i][j].type = apple_type


def comparator(node: Node):
    return node.f_cost


def pathfiding(start_point: Point, end_point: Point):
    # nastavimo zacetno tocko
    start_x = int(start_point.x) // FACTOR
    start_y = int(start_point.y) // FACTOR
    start_node = game[start_x][start_y]
    start_node.type = NodeType.CHECKED
    start_node.f_cost = -10
    start_node.g_cost = 0

    # nastavimo ciljno tocko
    end_x = int(end_point.x) // FACTOR
    end_y = int(end_point.y) // FACTOR
    end_node = game[end_x][end_y]

    # nodes to be checked
    ongoing = [game[start_x][start_y]]
    cost = 0

    while True:
        # priority queue would be faster
        # najdemo najboljšo točko
        best_node = min(ongoing, key=comparator)
        best_node.type = NodeType.CHECKED
        cost += best_node.g_cost
        # ponastavimo seznam
        ongoing = []
        # iteriramo čez sosede
        for i in range(best_node.x - 1, best_node.x + 1):
            for j in range(best_node.y - 1, best_node.y + 1):
                if 0 <= i < NODE_WIDTH and 0 <= j < NODE_HEIGHT and game[i][j].type == NodeType.UNCHECKED:
                    temp = game[i][j]
                    g_cost = calc_cost(temp, best_node) + cost
                    h_cost = calc_cost(temp, end_node)
                    temp.set(g_cost, h_cost, best_node)
                    ongoing.append(temp)


def main():
    win = GraphWin("Pathfinding by Jacob", WIN_WIDTH, WIN_HEIGHT)
    put_apple(Point(600, 600), NodeType.GOOD_APPLE)
    put_apple(Point(2000, 1600), NodeType.GOOD_APPLE)

    pathfiding(Point(200, 200), Point(1000, 1000))
    draw_map(win)

    win.getMouse()  # pause for click in window
    win.close()


game = [[Node(i * 5, j * 5) for j in range(137)] for i in range(273)]
main()
