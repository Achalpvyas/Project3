import math
import matplotlib.pyplot as plt
import time
import matplotlib.animation as anim
import sys
import os

show_animation = True


class AStarAlgo:

    def __init__(self, obstaclex, obstacley, resolution, robot_radius, theta_s, step_size, startx, starty):
        self.resolution = resolution
        self.robot_radius = robot_radius
        self.obstacle_map(obstaclex, obstacley)
        self.motion = self.motion_mode(theta_s, startx, starty)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

    def planner(self, startx, starty, goalx, goaly):

        start_node = self.Node(self.xy_index(startx, self.minx),
                               self.xy_index(starty, self.miny), 0.0, -1)
        goal_node = self.Node(self.xy_index(goalx, self.minx),
                              self.xy_index(goaly, self.miny), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.grid_index(start_node)] = start_node
        # print(start_node)

        while 1:
            if len(open_set) == 0:
                print("No Solution Found")
                break

            current_node_index = min(open_set,
                                     key=lambda o: open_set[o].cost + self.euclidean_distance(goal_node, open_set[o]))
            current_node = open_set[current_node_index]

            if show_animation:
                plt.plot(self.grid_position(current_node.x, self.minx),
                         self.grid_position(current_node.y, self.miny), "xc")
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current_node.x == goal_node.x and current_node.y == goal_node.y:
                print("Goal node found")
                goal_node.parent_index = current_node.parent_index
                goal_node.cost = current_node.cost
                break

            # Remove the item from the open set
            del open_set[current_node_index]

            # Add it to the closed set
            closed_set[current_node_index] = current_node

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current_node.x + self.motion[i][0],
                                 current_node.y + self.motion[i][1],
                                 current_node.cost + self.motion[i][2], current_node_index)
                # print(node)
                node_index = self.grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if node_index in closed_set:
                    continue

                if node_index not in open_set:
                    open_set[node_index] = node  # discovered a new node
                else:
                    if open_set[node_index].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[node_index] = node

        resultx, resulty = self.final_path(goal_node, closed_set)

        return resultx, resulty



    def final_path(self, goal_node, closedset):
        # generate final course
        resultx, resulty = [self.grid_position(goal_node.x, self.minx)], [
            self.grid_position(goal_node.y, self.miny)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closedset[parent_index]
            resultx.append(self.grid_position(n.x, self.minx))
            resulty.append(self.grid_position(n.y, self.miny))
            parent_index = n.parent_index

        return resultx, resulty

    @staticmethod
    def euclidean_distance(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def grid_position(self, index, minp):

        pos = index * self.resolution + minp
        return pos

    def xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def grid_index(self, node):
        return (node.y - self.miny) * self.xwidth + (node.x - self.minx)

    def verify_node(self, node):
        px = self.grid_position(node.x, self.minx)
        py = self.grid_position(node.y, self.miny)

        if px < self.minx:
            return False
        elif py < self.miny:
            return False
        elif px >= self.maxx:
            return False
        elif py >= self.maxy:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def obstacle_map(self, obstaclex, obstacley):

        self.minx = round(min(obstaclex))
        self.miny = round(min(obstacley))
        self.maxx = round(max(obstaclex))
        self.maxy = round(max(obstacley))

        self.xwidth = round((self.maxx - self.minx) / self.resolution)
        self.ywidth = round((self.maxy - self.miny) / self.resolution)

        # obstacle map generation
        self.obstacle_map = [[False for i in range(self.ywidth)]
                             for i in range(self.xwidth)]
        for ix in range(self.xwidth):
            x = self.grid_position(ix, self.minx)
            for iy in range(self.ywidth):
                y = self.grid_position(iy, self.miny)
                for iobstaclex, iobstacley in zip(obstaclex, obstacley):
                    d = math.hypot(iobstaclex - x, iobstacley - y)
                    if d <= self.robot_radius:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def motion_mode(theta_s, startx, starty):

        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion

    @staticmethod
    def plot_curve(Xi, Yi, Thetai, UL, UR):
        t = 0
        r = 0.038
        L = 0.354
        dt = 0.1
        Xn = Xi
        Yn = Yi
        Thetan = 3.14 * Thetai / 180

        # Xi, Yi,Thetai: Input point's coordinates
        # Xs, Ys: Start point coordinates for plot function
        # Xn, Yn, Thetan: End point coordintes

        while t < 1:
            t = t + dt
            Xs = Xn
            Ys = Yn
            Xn += 0.5 * r * (UL + UR) * math.cos(Thetan) * dt
            Yn += 0.5 * r * (UL + UR) * math.sin(Thetan) * dt
            Thetan += (r / L) * (UR - UL) * dt
            plt.plot([Xs, Xn], [Ys, Yn], color="blue")

        Thetan = 180 * (Thetan) / 3.14
        return Xn, Yn, Thetan




if __name__ == '__main__':

    print("Enter start node information --> x, y and theta separated by commas")
    sx, sy, theta_s = list(map(float, input().split()))
    startx, starty = sx*10 + 50, sy*10 + 50

    print("---------------------------------------------------------------------")
    print("Enter goal node information --> x, y and theta separated by commas")
    gx, gy = list(map(float, input().split()))
    goalx, goaly = gx*10 + 50, gy*10 + 50

    print("---------------------------------------------------------------------")
    print("Enter: left wheel rpm | Right wheel rpm ")
    U1, U2 = list(map(float, input().split()))

    print("---------------------------------------------------------------------")
    step_size = 1
    clearance = int(input("Enter the clearance: "))
    robot_radius = 3.54/2

    grid_size = 1
    print("---------------------------------------------------------------------")

    t = 0
    r = 0.38
    L = 3.54
    dt = 0.1




    obstaclex, obstacley = [], []


    if ((startx - 51) ** 2 + (starty - 51) ** 2 <= ((10 + clearance) ** 2)) or \
            ((goalx - 51) ** 2 + (goaly - 51) ** 2 <= ((10 + clearance) ** 2)) or \
            ((startx - 71) ** 2 + (starty - 81) ** 2 <= ((10 + clearance) ** 2)) or \
            ((goalx - 71) ** 2 + (goaly - 81) ** 2 <= ((10+ clearance) ** 2)) or \
            ((startx - 31) ** 2 + (starty - 21) ** 2 <= ((10+ clearance) ** 2)) or \
            ((goalx - 31) ** 2 + (goaly - 21) ** 2 <= ((10+ clearance) ** 2)) or \
            ((startx - 71) ** 2 + (starty - 21) ** 2 <= ((10+ clearance) ** 2)) or \
            ((goalx - 71) ** 2 + (goaly - 21) ** 2 <= ((10+ clearance) ** 2)):
        print("Error!!! Input node in the obstacle space")
        sys.exit()

    if (((startx - (3.5 + clearance)) >= 0) and (
            (startx - (18.5 - clearance)) <= 0) and (
            (starty - (46 + clearance)) >= 0) and (
            (starty - (61 - clearance)) <= 0)):
        print("ERROR! Input node in the obstacle space")
        sys.exit()
    # Rectangle
    if (((startx - (23.5 + clearance)) >= 0) and (
            (startx - (38.5 - clearance)) <= 0) and (
            (starty - (73.5 + clearance)) >= 0) and (
            (starty - (88.5 - clearance)) <= 0)):
        print("ERROR! Input node in the obstacle space")
        sys.exit()
    # Rectangle
    if (((startx - (81.5 + clearance)) >= 0) and (
            (startx - (96.5 - clearance)) <= 0) and (
            (starty - (46.0 + clearance)) >= 0) and (
            (starty - (61.0 - clearance)) <= 0)):
        print("ERROR! Input node in the obstacle space")
        sys.exit()

    if (((goalx - (3.5+ clearance)) >= 0) and (
            (goalx - (18.5 - clearance)) <= 0) and (
            (goaly - (46 + clearance)) >= 0) and (
            (goaly - (61 - clearance)) <= 0)):
        print("ERROR! Input node in the obstacle space")
        sys.exit()
    # Rectangle
    if (((goalx - (23.5 + clearance)) >= 0) and (
            (goalx - (38.5 - clearance)) <= 0) and (
            (goaly - (73.5 + clearance)) >= 0) and (
            (goaly - (88.5 - clearance)) <= 0)):
        print("ERROR! Input node in the obstacle space")
        sys.exit()
    # Rectangle
    if (((goalx - (81.5 + clearance)) >= 0) and (
            (goalx - (96.5 - clearance)) <= 0) and (
            (goaly - (46.0 + clearance)) >= 0) and (
            (goaly - (61.0 - clearance)) <= 0)):
        print("ERROR! Input node in the obstacle space")
        sys.exit()

    print("Searching...")
    for i in range(101):
        for j in range(101):

            if (((i - (-1)) >= 0) and ((i - (0 + (robot_radius + clearance))) <= 0)):
                obstaclex.append(i)
                obstacley.append(j)

            if (((i - (100 - (robot_radius + clearance))) >= 0) and ((i - (101)) <= 0)):
                obstaclex.append(i)
                obstacley.append(j)

            if (((j - (-1)) >= 0) and ((j - (0 + (robot_radius + clearance))) <= 0)):
                obstaclex.append(i)
                obstacley.append(j)

            if (((j - (100 - (robot_radius + clearance))) >= 0) and ((i - (101)) <= 0)):
                obstaclex.append(i)
                obstacley.append(j)

            if ((i - 51) ** 2 + (j - 51) ** 2 <= ((10 + robot_radius + clearance) ** 2)):
                obstaclex.append(i)
                obstacley.append(j)

            if ((i - 71) ** 2 + (j - 81) ** 2 <= ((10 + robot_radius + clearance) ** 2)):
                obstaclex.append(i)
                obstacley.append(j)

            # Circle
            if ((i - 31) ** 2 + (j - 21) ** 2 <= ((10 + robot_radius + clearance) ** 2)):
                obstaclex.append(i)
                obstacley.append(j)
            # Circle
            if ((i - 71) ** 2 + (j - 21) ** 2 <= ((10 + robot_radius + clearance) ** 2)):
                obstaclex.append(i)
                obstacley.append(j)

            if (((i - (3.5 - (robot_radius + clearance))) >= 0) and (
                    (i - (18.5 + (robot_radius + clearance))) <= 0) and (
                    (j - (46 - (robot_radius + clearance))) >= 0) and (
                    (j - (61 + (robot_radius + clearance))) <= 0)):
                obstaclex.append(i)
                obstacley.append(j)

            # Rectangle
            if (((i - (23.5 - (robot_radius + clearance))) >= 0) and (
                    (i - (38.5 + (robot_radius + clearance))) <= 0) and (
                    (j - (73.5 - (robot_radius + clearance))) >= 0) and (
                    (j - (88.5 + (robot_radius + clearance))) <= 0)):
                obstaclex.append(i)
                obstacley.append(j)

            # Rectangle
            if (((i - (81.5 - (robot_radius + clearance))) >= 0) and (
                    (i - (96.5 + (robot_radius + clearance))) <= 0) and (
                    (j - (46.0 - (robot_radius + clearance))) >= 0) and (
                    (j - (61.0 + (robot_radius + clearance))) <= 0)):
                obstaclex.append(i)
                obstacley.append(j)


    StartTime = time.time()
    if show_animation:  # pragma: no cover
        plt.plot(obstaclex, obstacley, ".k")
        plt.plot(startx, starty, "or")
        plt.plot(goalx, goaly, "og")
        plt.grid(False)
        plt.axis("equal")

    solver = AStarAlgo(obstaclex, obstacley, grid_size, robot_radius, theta_s, step_size, startx, starty)
    resultx, resulty = solver.planner(startx, starty, goalx, goaly)


    angles = []
    #############################
    ###### Calculate theta ######
    for i in range(len(resultx)):
        xdiff = resultx[i-1] - resultx[i]
        ydiff = resulty[i-1] - resulty[i]
        theta = math.degrees(math.atan2(ydiff, xdiff))
        angles.append(theta)

    EndTime = time.time()
    if os.path.exists("Path_file.txt"):
        os.remove("Path_file.txt")

    f = open("Path_file.txt", "a")
    for i in range(len(resultx)):
        f.write(str(resultx[i]/10) + " " + str(resulty[i]/10) + " " + str(angles[i]) + '\n')
    print("Node path file saved")
    f.close()

    X1 = []
    X2 = []
    actions = [[U1, 0], [0,U1], [U2, 0], [0, U2], [U1, U2], [U2, U1], [U1, U1], [U2,U2]]
    # for action in actions:
    #
    #     X1 = plot_curve(resultx[0], resulty[0], theta_s, action[0], action[1])  # (0,0,45) hypothetical start configuration
    #
    #     for action in actions:
    #         X2 = plot_curve(X1[0], X1[1], X1[2], action[0], action[1])

    print("Solved in:", EndTime - StartTime)
    if show_animation:
        plt.plot(resultx, resulty, "-r")
        # plt.plot([Xs, Xn], [Ys, Yn], color="blue")

        plt.show()


