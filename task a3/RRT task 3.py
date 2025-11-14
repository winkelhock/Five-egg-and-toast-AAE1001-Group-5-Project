import math
import random
import matplotlib.pyplot as plt

show_animation = True


class RRTPlanner:

    def __init__(self, ox, oy, resolution, rr, fc_x, fc_y, tc_x, tc_y):
        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        
        self.fc_x = fc_x
        self.fc_y = fc_y
        self.tc_x = tc_x
        self.tc_y = tc_y
        
        self.Delta_C1 = 0.3
        self.Delta_C2 = 0.15
        
        self.calc_obstacle_map(ox, oy)
        
        self.expand_dis = 1.0
        self.path_resolution = 0.5
        self.goal_sample_rate = 0.1
        self.max_iter = 5000

    class Node:
        def __init__(self, x, y, cost=0.0, parent_index=-1):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        start_node = self.Node(sx, sy, 0.0, -1)
        goal_node = self.Node(gx, gy, 0.0, -1)
        
        node_list = [start_node]
        
        for i in range(self.max_iter):
            # Sample random point
            rnd_node = self.get_random_node(goal_node)
            
            # Find nearest node
            nearest_ind = self.get_nearest_node_index(node_list, rnd_node)
            nearest_node = node_list[nearest_ind]
            
            # Expand towards random point
            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)
            
            # Check collision
            if self.check_collision(nearest_node, new_node):
                new_node.parent_index = nearest_ind
                node_list.append(new_node)
                
                if show_animation and i % 5 == 0:
                    plt.plot([nearest_node.x, new_node.x],
                            [nearest_node.y, new_node.y], "-c")
                    plt.gcf().canvas.mpl_connect('key_release_event',
                                                lambda event: [exit(0) if event.key == 'escape' else None])
                    plt.pause(0.001)
                
                # Check if goal is reached
                if self.calc_dist_to_goal(new_node, goal_node) <= self.expand_dis:
                    goal_node.parent_index = len(node_list) - 1
                    goal_node.cost = new_node.cost + self.calc_cost(new_node, goal_node)
                    print("Total Trip time required -> ", goal_node.cost)
                    
                    node_list.append(goal_node)
                    break
        
        rx, ry = self.calc_final_path(goal_node, node_list)
        return rx, ry

    def steer(self, from_node, to_node, expand_dis):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        dist = math.hypot(dx, dy)
        
        if dist < expand_dis:
            new_node = self.Node(to_node.x, to_node.y)
        else:
            theta = math.atan2(dy, dx)
            new_node = self.Node(
                from_node.x + expand_dis * math.cos(theta),
                from_node.y + expand_dis * math.sin(theta)
            )
        
        new_node.cost = from_node.cost + self.calc_cost(from_node, new_node)
        return new_node

    def calc_cost(self, from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        dist = math.hypot(dx, dy)
        
        # Check if in cost zones
        mid_x = (from_node.x + to_node.x) / 2
        mid_y = (from_node.y + to_node.y) / 2
        
        cost = dist
        
        if mid_x in self.tc_x and mid_y in self.tc_y:
            cost += self.Delta_C1 * dist
        
        if mid_x in self.fc_x and mid_y in self.fc_y:
            cost += self.Delta_C2 * dist
        
        return cost

    def get_random_node(self, goal_node):
        if random.random() < self.goal_sample_rate:
            return self.Node(goal_node.x, goal_node.y)
        
        rnd_x = random.uniform(self.min_x, self.max_x)
        rnd_y = random.uniform(self.min_y, self.max_y)
        return self.Node(rnd_x, rnd_y)

    def get_nearest_node_index(self, node_list, rnd_node):
        dlist = [math.hypot(node.x - rnd_node.x, node.y - rnd_node.y) for node in node_list]
        return dlist.index(min(dlist))

    def calc_dist_to_goal(self, node, goal_node):
        return math.hypot(node.x - goal_node.x, node.y - goal_node.y)

    def check_collision(self, from_node, to_node):
        x1, y1 = from_node.x, from_node.y
        x2, y2 = to_node.x, to_node.y
        
        num_checks = int(math.hypot(x2 - x1, y2 - y1) / self.path_resolution)
        
        for i in range(num_checks + 1):
            t = i / (num_checks + 1) if num_checks > 0 else 0
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            
            if not self.verify_position(x, y):
                return False
        
        return True

    def verify_position(self, x, y):
        if x < self.min_x or y < self.min_y or x >= self.max_x or y >= self.max_y:
            return False
        
        ix = self.calc_xy_index(x, self.min_x)
        iy = self.calc_xy_index(y, self.min_y)
        
        if 0 <= ix < self.x_width and 0 <= iy < self.y_width:
            if self.obstacle_map[ix][iy]:
                return False
        
        return True

    def calc_final_path(self, goal_node, node_list):
        rx, ry = [goal_node.x], [goal_node.y]
        parent_index = goal_node.parent_index
        
        while parent_index != -1:
            node = node_list[parent_index]
            rx.append(node.x)
            ry.append(node.y)
            parent_index = node.parent_index
        
        return rx, ry

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_position(self, index, min_position):
        pos = index * self.resolution + min_position
        return pos

    def calc_obstacle_map(self, ox, oy):
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True


def main():
    print(__file__ + " start the RRT algorithm demo !!")

    sx = 0.0
    sy = 0.0
    gx = 50.0
    gy = 50.0
    grid_size = 1
    robot_radius = 1.0

    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(-10.0)
    for i in range(-10, 60):
        ox.append(60.0)
        oy.append(i)
    for i in range(-10, 61):
        ox.append(i)
        oy.append(60.0)
    for i in range(-10, 61):
        ox.append(-10.0)
        oy.append(i)
    start_x, start_y = 20, 0
    end_x, end_y = 25, 20
    steps = 425
    for i in range(steps + 1):
        t = i / steps
        x = start_x + t * (end_x - start_x)
        y = start_y + t * (end_y - start_y)
        ox.append(x)
        oy.append(y)
    start_x, start_y = 10, 55
    end_x, end_y = 25, 45
    steps = 325
    for i in range(steps + 1):
        t = i / steps
        x = start_x + t * (end_x - start_x)
        y = start_y + t * (end_y - start_y)
        ox.append(x)
        oy.append(y)
    start_x, start_y = 30, 0
    end_x, end_y = 45, 10
    steps = 325
    for i in range(steps + 1):
        t = i / steps
        x = start_x + t * (end_x - start_x)
        y = start_y + t * (end_y - start_y)
        ox.append(x)
        oy.append(y)

    tc_x, tc_y = [], []
    for i in range(10, 26):
        for j in range(20, 46):
            tc_x.append(i)
            tc_y.append(j)
    fc_x, fc_y = [], []
    for i in range(30, 46):
        for j in range(10, 36):
            fc_x.append(i)
            fc_y.append(j)

    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.plot(fc_x, fc_y, "oy")
        plt.plot(tc_x, tc_y, "or")
        plt.grid(True)
        plt.axis("equal")
    
    rrt = RRTPlanner(ox, oy, grid_size, robot_radius, fc_x, fc_y, tc_x, tc_y)
    rx, ry = rrt.planning(sx, sy, gx, gy)

    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.pause(0.001)
        plt.show()


if __name__ == '__main__':
    main()
