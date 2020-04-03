import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation

def world():
    circle_1 = plt.Circle((5, 5), radius=1, fc='k')
    circle_2 = plt.Circle((7, 8), radius=1, fc='k')
    circle_3 = plt.Circle((3, 2), radius=1, fc='k')
    circle_4 = plt.Circle((7, 2), radius=1, fc='k')

    rectangle_1 = plt.Rectangle((2.25,7.25), 1.5, 1.5, fc='k')
    rectangle_2 = plt.Rectangle((0.25, 4.25), 1.5, 1.5, fc='k')
    rectangle_3 = plt.Rectangle((8.25, 4.25), 1.5, 1.5, fc='k')

    ax.add_patch(circle_1)
    ax.add_patch(circle_2)
    ax.add_patch(circle_3)
    ax.add_patch(circle_4)
    ax.add_patch(rectangle_1)
    ax.add_patch(rectangle_2)
    ax.add_patch(rectangle_3)
    return fig

def init():

    patch.center = (sx, sy)
    ax.add_patch(patch)
    return patch,

def animate(i):
    x, y = patch.center
    x = sx + 3 * i
    y = sy + 3
    patch.center = (x, y)

    if patch.center[0] == gx and patch.center[1] == gy:
        print("goal node reached")
    return patch,




if __name__ == '__main__':
    print("Enter the start point")
    sx, sy = list(map(float, input().split()))

    print("Enter the goal point")
    gx, gy = list(map(float, input().split()))

    fig = plt.figure()
    fig.set_dpi(100)
    fig.set_size_inches(7, 6.5)
    ax = plt.axes(xlim=(-1, 11), ylim=(-1, 11))
    patch = plt.Circle((sx, sy), 0.1, fc='y')
    anim = animation.FuncAnimation(world(), animate,
                                   init_func=init,
                                   frames=360,
                                   interval=50,
                                   blit=True)
    plt.show()