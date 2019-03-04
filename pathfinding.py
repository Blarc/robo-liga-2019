from graphics import *
from enum import Enum
import heapq
from time import time

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
    START = 4
    END = 5
    PATH = 6
    CLOSED = 7


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

    def __str__(self):
        return str(int(self.x / 5)) + " " + str(int(self.y / 5))

    def __eq__(self, other):
        return other.x == self.x and other.y == self.y

    def __lt__(self, other):
        return self.f_cost < other.f_cost or self.h_cost < other.h_cost

    def set(self, g_cost, h_cost, parent):
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.f_cost = g_cost + h_cost
        self.parent = parent


def calc_cost(curr: Node, end: Node):
    x = abs(end.x / 5 - curr.x / 5)
    y = abs(end.y / 5 - curr.y / 5)

    # x bigger, y smaller
    if x < y:
        # x smaller, y bigger
        temp = x
        x = y
        y = temp
    return y * 14 + (x - y) * 10


def draw_map(win):
    # for i in game[0]:
    #     a = Point(i.x, i.y)
    #     b = Point(i.x + WIN_WIDTH, i.y)
    #     line = Line(a, b)
    #     line.draw(win)
    #
    for i in game:
        #     a = Point(i[0].x, i[0].y)
        #     b = Point(i[0].x, i[0].y + WIN_HEIGHT)
        #     line = Line(a, b)
        #     line.draw(win)

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
                r.setFill(color_rgb(255, 255, 0))
                r.draw(win)
            elif j.type == NodeType.START:
                a = Point(j.x, j.y)
                b = Point(j.x + 5, j.y + 5)
                r = Rectangle(a, b)
                r.setFill(color_rgb(255, 0, 0))
                r.draw(win)
            elif j.type == NodeType.END:
                a = Point(j.x, j.y)
                b = Point(j.x + 5, j.y + 5)
                r = Rectangle(a, b)
                r.setFill(color_rgb(0, 255, 0))
                r.draw(win)
            elif j.type == NodeType.PATH:
                a = Point(j.x, j.y)
                b = Point(j.x + 5, j.y + 5)
                r = Rectangle(a, b)
                r.setFill(color_rgb(0, 255, 255))
                r.draw(win)
            elif j.type == NodeType.CLOSED:
                a = Point(j.x, j.y)
                b = Point(j.x + 5, j.y + 5)
                r = Rectangle(a, b)
                r.setFill(color_rgb(255, 155, 55))
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
    start_node.type = NodeType.START

    # nastavimo ciljno tocko
    end_x = int(end_point.x) // FACTOR
    end_y = int(end_point.y) // FACTOR
    end_node = game[end_x][end_y]
    end_node.type = NodeType.END

    print("start: " + str(start_node))
    print("end: " + str(end_node))

    # nodes to be checked
    ongoing = PriorityQueue()
    ongoing.put(start_node, 0)

    while True:
        if ongoing.empty():
            print("NO PATH")
            break

        best_node = ongoing.get()
        best_node.type = NodeType.CLOSED
        # print(best_node)
        # print("Parent: " + str(best_node.parent))
        if best_node.__eq__(end_node):
            break
        # print(best_node)
        # print(best_node.type)
        # print(cost)
        for i in range(int(best_node.x / 5) - 1, int(best_node.x / 5) + 2):
            for j in range(int(best_node.y / 5) - 1, int(best_node.y / 5) + 2):
                if 0 <= i < NODE_WIDTH and 0 <= j < NODE_HEIGHT:
                    temp = game[i][j]
                    if temp.type == NodeType.UNCHECKED or temp.type == NodeType.END or temp.type == NodeType.CHECKED:
                        if temp.parent is None or temp.parent.g_cost > best_node.g_cost:
                            temp.parent = best_node
                            temp.type = NodeType.CHECKED
                            g_cost = calc_cost(temp, best_node) + best_node.g_cost
                            h_cost = calc_cost(temp, end_node)
                            temp.set(g_cost, h_cost, best_node)
                            ongoing.put(temp, temp.f_cost)

    path = []
    iter_node = end_node
    while not iter_node.__eq__(start_node):
        path.append(iter_node)
        iter_node = iter_node.parent
        iter_node.type = NodeType.PATH

    return path


def main():
    put_apple(Point(2000, 1600), NodeType.GOOD_APPLE)
    put_apple(Point(1500, 1600), NodeType.GOOD_APPLE)
    # put_apple(Point(2000, 1700), NodeType.GOOD_APPLE)
    # put_apple(Point(2000, 1800), NodeType.GOOD_APPLE)
    # put_apple(Point(2000, 1500), NodeType.GOOD_APPLE)
    # put_apple(Point(2000, 1400), NodeType.GOOD_APPLE)
    put_apple(Point(2000, 1900), NodeType.GOOD_APPLE)
    put_apple(Point(1500, 1900), NodeType.GOOD_APPLE)
    # put_apple(Point(2000, 2000), NodeType.GOOD_APPLE)
    # put_apple(Point(2000, 1400), NodeType.GOOD_APPLE)
    # put_apple(Point(2000, 1300), NodeType.GOOD_APPLE)
    # put_apple(Point(2000, 1200), NodeType.GOOD_APPLE)
    put_apple(Point(2000, 1100), NodeType.GOOD_APPLE)
    # put_apple(Point(2000, 1000), NodeType.GOOD_APPLE)
    # put_apple(Point(2000, 900), NodeType.GOOD_APPLE)
    # put_apple(Point(2000, 800), NodeType.GOOD_APPLE)
    # put_apple(Point(2000, 700), NodeType.GOOD_APPLE)
    put_apple(Point(2000, 600), NodeType.GOOD_APPLE)
    # put_apple(Point(2000, 500), NodeType.GOOD_APPLE)
    # put_apple(Point(2000, 400), NodeType.GOOD_APPLE)
    # put_apple(Point(2000, 300), NodeType.GOOD_APPLE)
    # put_apple(Point(2000, 200), NodeType.GOOD_APPLE)
    put_apple(Point(2000, 100), NodeType.GOOD_APPLE)

    start_time = time()
    path = pathfiding(Point(200, 200), Point(2500, 1900))
    path.reverse()
    for point in path:
        print(point)
    end_time = time()
    print((end_time - start_time) * 1000)
    win = GraphWin("Pathfinding by Jacob", WIN_WIDTH, WIN_HEIGHT)
    draw_map(win)
    win.getMouse()  # pause for click in window
    win.close()


game = [[Node(i * 5, j * 5) for j in range(137)] for i in range(273)]
main()
