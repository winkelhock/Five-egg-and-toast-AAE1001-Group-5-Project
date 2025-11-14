import math

import matplotlib.pyplot as plt

import random 

show_animation = True


class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr, fc_x, fc_y):
        
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
        
        self.Delta_C2 = 0.15

        self.costPerGrid = 1 


    class Node: 
        def __init__(self, x, y, cost, parent_index):
            self.x = x  
            self.y = y  
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        
        start_node = self.Node(self.calc_xy_index(sx, self.min_x), 
                               self.calc_xy_index(sy, self.min_y), 0.0, -1) 
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x), 
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict() 

        open_set[self.calc_grid_index(start_node)] = start_node 

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(self, goal_node,
                                                                     open_set[
                                                                         o])) 
            current = open_set[c_id]


            if show_animation: 
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
               
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

          
            if current.x == goal_node.x and current.y == goal_node.y:
                print("Total Trip time required -> ",current.cost )
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            
            del open_set[c_id]

          
            closed_set[c_id] = current

           
            for i, _ in enumerate(self.motion): 
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2] * self.costPerGrid, c_id)
                
               
               
                if self.calc_grid_position(node.x, self.min_x) in self.fc_x:
                    if self.calc_grid_position(node.y, self.min_y) in self.fc_y:
                        
                        node.cost = node.cost + self.Delta_C2 * self.motion[i][2]
                   
                
                n_id = self.calc_grid_index(node)

                
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)
        

        return rx, ry

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
    def calc_heuristic(self, n1, n2):
        w = 1.0  
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        d = d * self.costPerGrid
        return d
    
    def calc_heuristic_maldis(n1, n2):
        w = 1.0  # weight of heuristic
        dx = w * math.abs(n1.x - n2.x)
        dy = w *math.abs(n1.y - n2.y)
        return dx + dy

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x) 

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

    
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

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

        # obstacle map generation
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
                        break

    @staticmethod
    def get_motion_model(): 
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
            ]

        return motion


def main():
    print(__file__ + " start the A star algorithm demo !!") 
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
    map_min_x, map_max_x = -10, 60
    map_min_y, map_max_y = -10, 60

    obstacle_density = 0.15
    obstacle_count = int((map_max_x - map_min_x) * (map_max_y - map_min_y) * obstacle_density)
    for _ in range(obstacle_count):
        obs_x = random.uniform(map_min_x, map_max_x)
        obs_y = random.uniform(map_min_y, map_max_y)
       
        ox.append(obs_x)
        oy.append(obs_y)

    fc_start_x = random.randint(10, 20)
    fc_start_y = random.randint(10, 20)
    fc_x, fc_y = [], []
    for i in range(fc_start_x, fc_start_x + 40):
        for j in range(fc_start_y, fc_start_y + 40):
            fc_x.append(i)
            fc_y.append(j)

    filtered_ox, filtered_oy = [], []
    for obs_x, obs_y in zip(ox, oy):
        in_fc = (fc_start_x <= obs_x < fc_start_x + 40) and (fc_start_y <= obs_y < fc_start_y + 40)
        if not in_fc:
            filtered_ox.append(obs_x)
            filtered_oy.append(obs_y)
    ox, oy = filtered_ox, filtered_oy

    sx, sy, gx, gy = 0.0, 0.0, 0.0, 0.0
    while True:
        sx = random.uniform(map_min_x, map_max_x)
        sy = random.uniform(map_min_y, map_max_y)
        gx = random.uniform(map_min_x, map_max_x)
        gy = random.uniform(map_min_y, map_max_y)
        
        if math.hypot(gx - sx, gy - sy) < 40:
            continue
       
        start_in_fc = (fc_start_x <= sx < fc_start_x + 40) and (fc_start_y <= sy < fc_start_y + 40)
        goal_in_fc = (fc_start_x <= gx < fc_start_x + 40) and (fc_start_y <= gy < fc_start_y + 40)
        if start_in_fc or goal_in_fc:
            continue
        start_near_obs = any(math.hypot(sx - ox_, sy - oy_) < 3 for ox_, oy_ in zip(ox, oy))
        goal_near_obs = any(math.hypot(gx - ox_, gy - oy_) < 3 for ox_, oy_ in zip(ox, oy))
        if not start_near_obs and not goal_near_obs:
            break
    
    final_ox, final_oy = [], []
    for ox_, oy_ in zip(ox, oy):
        if math.hypot(ox_ - sx, oy_ - sy) >= 3 and math.hypot(ox_ - gx, oy_ - gy) >= 3:
            final_ox.append(ox_)
            final_oy.append(oy_)
    ox, oy = final_ox, final_oy
   
    if show_animation: 
        plt.plot(ox, oy, ".k", label="Obstacle")
        plt.plot(sx, sy, "og", label="Start")
        plt.plot(gx, gy, "xb", label="Goal")
        plt.scatter(fc_x, fc_y, c="y", alpha=0.3, label="Fuel-consuming Area")
        plt.grid(True)
        plt.axis("equal")
        plt.legend()

    a_star = AStarPlanner(ox, oy, grid_size, robot_radius, fc_x, fc_y)
    rx, ry = a_star.planning(sx, sy, gx, gy)

    if show_animation: 
        plt.plot(rx, ry, "-r") 
        plt.pause(0.001) 
        plt.show() 


if __name__ == '__main__':
    main()

