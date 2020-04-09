import matplotlib.pyplot as plt
import matplotlib.patches as pat
import numpy as np

# def inputs():
print("Enter start node information --> x, y and theta separated by commas")
print("---------------------------------------------------------------------")
startX, startY, startTheta = list(map(int, input().split()))
print("Enter goal node information --> x, y and theta separated by commas")
print("---------------------------------------------------------------------")
goalX, goalY, goalTheta = list(map(int, input().split()))


def world():
    circle_1 = pat.Circle((5, 5), radius=1, fc='k')
    circle_2 = plt.Circle((7, 8), radius=1, fc='k')
    circle_3 = plt.Circle((3, 2), radius=1, fc='k')
    circle_4 = plt.Circle((7, 2), radius=1, fc='k')
    rectangle_1 = plt.Rectangle((2.25, 7.25), 1.5, 1.5, fc='k')
    rectangle_2 = plt.Rectangle((0.25, 4.25), 1.5, 1.5, fc='k')
    rectangle_3 = plt.Rectangle((8.25, 4.25), 1.5, 1.5, fc='k')
    boundary_1 = plt.Rectangle((0, -0.1), 10, 0.1, fc='k')
    boundary_2 = plt.Rectangle((-0.1, 0), 0.1, 10, fc='k')
    boundary_3 = plt.Rectangle((0, 10), 10, 0.1, fc='k')
    boundary_4 = plt.Rectangle((10, 0), 0.1, 10, fc='k')

    ax.add_patch(circle_1)
    ax.add_patch(circle_2)
    ax.add_patch(circle_3)
    ax.add_patch(circle_4)
    ax.add_patch(rectangle_1)
    ax.add_patch(rectangle_2)
    ax.add_patch(rectangle_3)
    ax.add_patch(boundary_1)
    ax.add_patch(boundary_2)
    ax.add_patch(boundary_3)
    ax.add_patch(boundary_4)

    return fig


####################################################################
#################### Define Shape Patches ##########################

circle_1 = pat.Circle((5, 5), radius=1, fc='k')
circle_2 = pat.Circle((7, 8), radius=1, fc='k')
circle_3 = pat.Circle((3, 2), radius=1, fc='k')
circle_4 = pat.Circle((7, 2), radius=1, fc='k')
rectangle_1 = pat.Rectangle((2.25, 7.25), 1.5, 1.5, fc='k')
rectangle_2 = pat.Rectangle((0.25, 4.25), 1.5, 1.5, fc='k')
rectangle_3 = pat.Rectangle((8.25, 4.25), 1.5, 1.5, fc='k')
boundary_1 = pat.Rectangle((0, -0.1), 10, 0.1, fc='b')
boundary_2 = pat.Rectangle((-0.1, 0), 0.1, 10, fc='b')
boundary_3 = pat.Rectangle((0, 10), 10, 0, fc='b')
boundary_4 = pat.Rectangle((10, 0), 0.1, 10, fc='b')


#####################################################################


def checkInputs(xs, ys, xg, yg):
    # Checking the obstacle condition for start point and goal point inputs
    if (circle_1.contains_point((xs, ys)) or circle_2.contains_point((xs, ys)) or
            circle_3.contains_point((xs, ys)) or circle_4.contains_point((xs, ys)) or
            circle_1.contains_point((xg, yg)) or circle_2.contains_point((xg, yg)) or
            circle_3.contains_point((xg, yg)) or circle_4.contains_point((xg, yg)) or
            rectangle_1.contains_point((xs, ys)) or rectangle_2.contains_point((xs, ys)) or
            rectangle_3.contains_point((xs, ys)) or rectangle_1.contains_point((xg, yg)) or
            rectangle_2.contains_point((xg, yg)) or rectangle_3.contains_point((xg, yg)) or
            boundary_1.contains_point((xs, ys)) or boundary_2.contains_point((xs, ys)) or
            boundary_3.contains_point((xs, ys)) or boundary_4.contains_point((xs, ys)) or
            boundary_1.contains_point((xg, yg)) or boundary_2.contains_point((xg, yg)) or
            boundary_3.contains_point((xg, yg)) or boundary_4.contains_point((xg, yg))):
        return False
    else:
        return True


def obstacleNodeCheck(Node):
    x = Node[0]
    y = Node[1]

    # Constraining the nodes
    if (circle_1.contains_point((x, y)) or circle_2.contains_point((x, y)) or
            circle_3.contains_point((x, y)) or circle_4.contains_point((x, y)) or
            rectangle_1.contains_point((x, y)) or rectangle_2.contains_point((x, y)) or
            rectangle_3.contains_point((x, y)) or
            boundary_1.contains_point((x, y)) or boundary_2.contains_point((x, y)) or
            boundary_3.contains_point((x, y)) or boundary_4.contains_point((x, y))):
        return False
    else:
        return True


if __name__ == '__main__':

    fig = plt.figure()
    fig.set_dpi(100)
    fig.set_size_inches(7, 6.5)
    ax = plt.axes(xlim=(-1, 11), ylim=(-1, 11))
    if checkInputs(startX, startY, goalX, goalY):
        world()
        plt.show()
    else:
        print("ERROR! Invalid Inputs")
