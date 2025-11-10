import math

import matplotlib.pyplot as plt

show_animation = True


class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr, fc_x, fc_y, tc_x, tc_y):
        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

        self.fc_x = fc_x
        self.fc_y = fc_y
        self.tc_x = tc_x
        self.tc_y = tc_y

        self.Delta_C1 = 0.3      # extra cost in tc-area
        self.Delta_C2 = 0.15     # extra cost in fc-area

        self.costPerGrid = 1

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return f"{self.x},{self.y},{self.cost},{self.parent_index}"

    # ------------------------------------------------------------------ #
    #  Core A* – unchanged except a bug-fix in heuristic call
    # ------------------------------------------------------------------ #
    def planning(self, sx, sy, gx, gy):
        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while open_set:
            # ----- heuristic fix -------------------------------------------------
            c_id = min(open_set,
                       key=lambda o: open_set[o].cost +
                       self.calc_heuristic(goal_node, open_set[o]))
            # ---------------------------------------------------------------------

            current = open_set[c_id]

            if show_animation:
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                if len(closed_set) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Goal reached – cost:", current.cost)
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            del open_set[c_id]
            closed_set[c_id] = current

            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2] * self.costPerGrid,
                                 c_id)

                # ----- extra cost for special areas --------------------------------
                grid_x = self.calc_grid_position(node.x, self.min_x)
                grid_y = self.calc_grid_position(node.y, self.min_y)

                if grid_x in self.tc_x and grid_y in self.tc_y:
                    node.cost += self.Delta_C1 * self.motion[i][2]
                if grid_x in self.fc_x and grid_y in self.fc_y:
                    node.cost += self.Delta_C2 * self.motion[i][2]
                # -------------------------------------------------------------------

                n_id = self.calc_grid_index(node)

                if not self.verify_node(node):
                    continue
                if n_id in closed_set:
                    continue
                if n_id not in open_set:
                    open_set[n_id] = node
                elif open_set[n_id].cost > node.cost:
                    open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)
        return rx, ry

    # ------------------------------------------------------------------ #
    #  Helper methods (unchanged)
    # ------------------------------------------------------------------ #
    def calc_final_path(self, goal_node, closed_set):
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index
        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    @staticmethod
    def calc_heuristic_maldis(n1, n2):
        w = 1.0
        return w * (abs(n1.x - n2.x) + abs(n1.y - n2.y))

    def calc_grid_position(self, index, min_position):
        return index * self.resolution + min_position

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x or py < self.min_y or \
           px >= self.max_x or py >= self.max_y:
            return False
        if self.obstacle_map[node.x][node.y]:
            return False
        return True

    def calc_obstacle_map(self, ox, oy):
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)

        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]

        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    if math.hypot(iox - x, ioy - y) <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        return [[1, 0, 1], [0, 1, 1], [-1, 0, 1], [0, -1, 1],
                [-1, -1, math.sqrt(2)], [-1, 1, math.sqrt(2)],
                [1, -1, math.sqrt(2)], [1, 1, math.sqrt(2)]]


# ---------------------------------------------------------------------- #
#  NEW: plan a path that *must* go through the two checkpoints
# ---------------------------------------------------------------------- #
def plan_via_checkpoints(planner, sx, sy, cp1_x, cp1_y, cp2_x, cp2_y, gx, gy):
    """Run A* three times and concatenate the results."""
    # start → CP1
    rx1, ry1 = planner.planning(sx, sy, cp1_x, cp1_y)
    # CP1 → CP2
    rx2, ry2 = planner.planning(cp1_x, cp1_y, cp2_x, cp2_y)
    # CP2 → goal
    rx3, ry3 = planner.planning(cp2_x, cp2_y, gx, gy)

    # remove duplicated points at the junctions
    path_x = rx1[:-1] + rx2[:-1] + rx3
    path_y = ry1[:-1] + ry2[:-1] + ry3
    return path_x, path_y


# ---------------------------------------------------------------------- #
def main():
    print(__file__ + " start the A star algorithm demo !!")

    sx = 0.0
    sy = 0.0
    gx = 50.0
    gy = 50.0
    grid_size = 1
    robot_radius = 1.0

    # ----- obstacles (same as original) ---------------------------------
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i);   oy.append(-10.0)
    for i in range(-10, 60):
        ox.append(60.0); oy.append(i)
    for i in range(-10, 61):
        ox.append(i);   oy.append(60.0)
    for i in range(-10, 61):
        ox.append(-10.0); oy.append(i)

    # three diagonal obstacle lines
    def add_line(start_x, start_y, end_x, end_y, steps):
        for i in range(steps + 1):
            t = i / steps
            ox.append(start_x + t * (end_x - start_x))
            oy.append(start_y + t * (end_y - start_y))

    add_line(20, 0, 25, 20, 425)
    add_line(10, 55, 25, 45, 325)
    add_line(30, 0, 45, 10, 325)

    # ----- cost-intensive areas -----------------------------------------
    tc_x, tc_y = [], []                     # slow area
    for i in range(10, 26):
        for j in range(20, 46):
            tc_x.append(i); tc_y.append(j)

    fc_x, fc_y = [], []                     # fast area
    for i in range(30, 46):
        for j in range(10, 36):
            fc_x.append(i); fc_y.append(j)

    # ----- ONE CHECKPOINT PER COST AREA ---------------------------------
    # centre of the rectangles (any point inside works – centre is clear)
    cp1_x = (min(fc_x) + max(fc_x)) / 2.0   # fast-cost checkpoint
    cp1_y = (min(fc_y) + max(fc_y)) / 2.0
    cp2_x = (min(tc_x) + max(tc_x)) / 2.0   # slow-cost checkpoint
    cp2_y = (min(tc_y) + max(tc_y)) / 2.0

    # --------------------------------------------------------------------
    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og", label="start")
        plt.plot(gx, gy, "xb", label="goal")
        plt.plot(fc_x, fc_y, "oy", alpha=0.3, label="fast-cost area")
        plt.plot(tc_x, tc_y, "or", alpha=0.3, label="slow-cost area")

        # checkpoints
        plt.plot(cp1_x, cp1_y, "sm", markersize=10, label="CP1 (fast)")
        plt.plot(cp2_x, cp2_y, "sm", markersize=10, label="CP2 (slow)")

        plt.grid(True)
        plt.axis("equal")
        plt.legend()

    a_star = AStarPlanner(ox, oy, grid_size, robot_radius,
                          fc_x, fc_y, tc_x, tc_y)

    # ----- PLAN WITH MANDATORY CHECKPOINTS ------------------------------
    rx, ry = plan_via_checkpoints(a_star,
                                  sx, sy,
                                  cp1_x, cp1_y,
                                  cp2_x, cp2_y,
                                  gx, gy)

    if show_animation:
        plt.plot(rx, ry, "-r", linewidth=2, label="path")
        plt.pause(0.001)
        plt.show()


if __name__ == '__main__':
    main()
