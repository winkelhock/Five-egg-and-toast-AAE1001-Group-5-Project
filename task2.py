import math
import matplotlib.pyplot as plt
import pandas as pd
import io

# Keep the original show_animation setting
show_animation = True


class AStarPlanner:
    """
    A* planner customized for flight path planning with variable costs,
    including high-cost areas (fc_x, tc_x) and a reward zone (rc_x) representing 
    a beneficial jet stream.
    """

    def __init__(self, ox, oy, resolution, rr, fc_x, fc_y, tc_x, tc_y,
                 rc_x=None, rc_y=None, jet_vx=1.0, jet_vy=1.0,
                 J_max_discount=0.5, J_counter_penalty=0.0):

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

        # Use sets for faster 'in' checks for cost zones
        self.fc_x = set(fc_x) # Fuel consumption cost zone X
        self.fc_y = set(fc_y) # Fuel consumption cost zone Y
        self.tc_x = set(tc_x) # Time-related cost zone X
        self.tc_y = set(tc_y) # Time-related cost zone Y

        # Jet stream reward zone
        self.rc_x = set(rc_x) if rc_x is not None else set()
        self.rc_y = set(rc_y) if rc_y is not None else set()

        # Jet stream direction and strength (used for cost modification)
        self.jet_vx = jet_vx
        self.jet_vy = jet_vy
        self.J_max_discount = J_max_discount      # Max cost reduction (e.g., 0.05 for 5%)
        self.J_counter_penalty = J_counter_penalty  # Penalty for moving against the flow

        # Fixed cost penalties for high-cost zones
        self.Delta_C1 = 0.3 # Time cost zone multiplier
        self.Delta_C2 = 0.15 # Fuel cost zone multiplier
        self.costPerGrid = 1 # Base cost per step

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return f"{self.x},{self.y},{self.cost},{self.parent_index}"

    def planning(self, sx, sy, gx, gy):

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while len(open_set) > 0:
            
            # Get the node with the lowest f = g + h score
            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(open_set[o], goal_node))
            current = open_set[c_id]

            # Reached goal
            if current.x == goal_node.x and current.y == goal_node.y:
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            del open_set[c_id]
            closed_set[c_id] = current

            # Explore neighbor nodes
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2] * self.costPerGrid, c_id)

                # --- Cost calculation logic with variable zones ---
                gx_pos = self.calc_grid_position(node.x, self.min_x)
                gy_pos = self.calc_grid_position(node.y, self.min_y)
                step_cost = self.motion[i][2] * self.costPerGrid

                # 1. High-Cost Area Checks (Penalties)
                if gx_pos in self.tc_x and gy_pos in self.tc_y:
                    node.cost += self.Delta_C1 * step_cost

                if gx_pos in self.fc_x and gy_pos in self.fc_y:
                    node.cost += self.Delta_C2 * step_cost

                # 2. Jet Stream Reward Zone Check (Discount)
                if gx_pos in self.rc_x and gy_pos in self.rc_y:
                    # motion direction vector (normalized)
                    mvx, mvy = float(self.motion[i][0]), float(self.motion[i][1])
                    mv_norm = math.hypot(mvx, mvy)
                    
                    # jet flow vector (normalized)
                    jvx, jvy = float(self.jet_vx), float(self.jet_vy)
                    jv_norm = math.hypot(jvx, jvy)

                    if mv_norm > 0 and jv_norm > 0:
                        # Cosine of angle: 1 for perfect alignment, -1 for counter-flow
                        cos_align = (mvx * jvx + mvy * jvy) / (mv_norm * jv_norm)

                        # align_pos: Max discount for alignment (cos_align=1)
                        align_pos = max(0.0, cos_align) 
                        # align_neg: Max penalty for counter-flow (cos_align=-1)
                        align_neg = max(0.0, -cos_align) 

                        # Apply discount for aligned motion (max J_max_discount)
                        node.cost -= align_pos * self.J_max_discount * step_cost
                        
                        # Apply penalty for counter-flow motion (J_counter_penalty)
                        node.cost += align_neg * self.J_counter_penalty * step_cost

                n_id = self.calc_grid_index(node)

                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node
                else:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id] = node
        
        if len(open_set) == 0 and not (current.x == goal_node.x and current.y == goal_node.y):
             # Path not found
             return [], [], float('inf')


        rx, ry = self.calc_final_path(goal_node, closed_set)
        return rx, ry, goal_node.cost

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

    # Note: Heuristic uses the lowest possible cost (base cost - max discount)
    def calc_heuristic(self, n1, n2):
        w = 1.0
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        min_cost = self.costPerGrid * (1.0 - self.J_max_discount)
        return d * min_cost

    def calc_grid_position(self, index, min_position):
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        # Check map boundaries
        if not (self.min_x <= px < self.max_x and self.min_y <= py < self.max_y):
            return False

        # Check for out of bounds index before accessing obstacle_map (based on grid indices)
        if not (0 <= node.x < self.x_width and 0 <= node.y < self.y_width):
            return False
            
        # Check obstacle map
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
        
        # Ensure x_width and y_width are at least 1 for map creation
        if self.x_width <= 0: self.x_width = 1
        if self.y_width <= 0: self.y_width = 1

        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        
        # Populate obstacle map based on radius
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
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion

# --- COST ANALYSIS SECTION ---

def perform_cost_analysis(trip_time):
    """
    Performs cost analysis for different aircraft based on the optimal trip time.
    (This function uses the fixed Scenario 1 parameters for the analysis report)
    """
    if trip_time == float('inf'):
        print("\nSkipping cost analysis: No optimal path found.")
        return

    # 1. Define Aircraft Specifications (Scenario 1 context uses specific costs)
    aircraft_data_csv = """Aircraft,Fuel_Consumption_Rate,Passenger_Capacity,Time_Cost_Low,Time_Cost_Medium,Time_Cost_High,Fixed_Cost
A321neo,54,200,10,15,20,1800
A330-900neo,84,300,15,21,27,2000
A350-900,90,350,20,27,34,2500
"""
    aircraft_df = pd.read_csv(io.StringIO(aircraft_data_csv))
    aircraft_df.set_index('Aircraft', inplace=True)
    
    # Map time cost levels to dataframe columns
    time_cost_map = {
        'low': 'Time_Cost_Low',
        'medium': 'Time_Cost_Medium',
        'high': 'Time_Cost_High'
    }

    # 2. Define Scenario 1 parameters
    scenario = {
        "name": "Scenario 1 (Optimization Target)", 
        "passengers": 3300, 
        "max_flights": 13, 
        "time_cost_level": "medium", 
        "fuel_cost": 0.85
    }

    print("\n" + "="*70)
    print("                 AIRCRAFT COST & SCENARIO ANALYSIS")
    print("="*70)
    print(f"Analysis for: {scenario['name']}")
    print(f"Based on OPTIMAL flight time (T_best): {trip_time:.2f} minutes\n")

    results = []
    
    for aircraft_name, specs in aircraft_df.iterrows():
        # Calculate number of flights needed
        flights_needed = math.ceil(scenario["passengers"] / specs["Passenger_Capacity"])
        
        # Check if the number of flights is feasible
        if flights_needed > scenario["max_flights"]:
            results.append({
                "aircraft": aircraft_name,
                "flights_needed": flights_needed,
                "total_cost": float('inf'), # Infeasible
                "feasible": "No"
            })
            continue
        
        # Get parameters for the cost formula
        C_F = scenario["fuel_cost"]           # Cost of Fuel per kg
        delta_F = specs["Fuel_Consumption_Rate"]  # Fuel consumption rate (kg/min)
        T_best = trip_time
        
        time_cost_column = time_cost_map[scenario["time_cost_level"]]
        C_T = specs[time_cost_column]         # Time-related cost per minute
        
        C_c = specs["Fixed_Cost"]             # Fixed cost per flight
        
        # C = C_F * ΔF * T_best + C_T * T_best + C_c
        cost_per_flight = (C_F * delta_F * T_best) + (C_T * T_best) + C_c
        
        # Calculate total cost for the scenario
        total_cost = cost_per_flight * flights_needed
        
        results.append({
            "aircraft": aircraft_name,
            "flights_needed": flights_needed,
            "total_cost": total_cost,
            "feasible": "Yes"
        })
        
    # 4. Analyze results and print the recommendation
    best_choice = None
    min_cost = float('inf')
    
    print(f"{'Aircraft':<15} | {'Flights Needed':<15} | {'Feasible?':<10} | {'Total Cost (USD)':<20}")
    print("-"*70)
    
    for res in results:
        if res["feasible"] == "Yes":
            cost_str = f"${res['total_cost']:,.2f}"
            if res["total_cost"] < min_cost:
                min_cost = res["total_cost"]
                best_choice = res["aircraft"]
        else:
            cost_str = "Exceeds limit"
            
        print(f"{res['aircraft']:<15} | {res['flights_needed']:<15} | {res['feasible']:<10} | {cost_str:<20}")
        
    print("-" * 70)
    if best_choice:
        print(f"CONCLUSION: The best aircraft for the optimal route is the {best_choice} with a total cost of ${min_cost:,.2f}.")
    else:
        print(f"CONCLUSION: No single aircraft model can meet the passenger requirements within the flight limit.")
    print("="*70)


def main():
    print("Starting A* optimization for optimal jet stream placement...")

    # --- SETUP MAP AND BOUNDARIES ---
    sx, sy = 0.0, 0.0
    gx, gy = 50.0, 50.0
    grid_size = 1.0
    robot_radius = 1.0

    # Obstacles (ox, oy)
    ox, oy = [], []
    for i in range(-10, 61):
        ox.append(i); oy.append(-10.0) # Bottom wall
        ox.append(i); oy.append(60.0)  # Top wall
    for i in range(-10, 61):
        ox.append(60.0); oy.append(i) # Right wall
        ox.append(-10.0); oy.append(i)# Left wall
    
    # Complex Obstacles
    start_x, start_y = 20, 0; end_x, end_y = 25, 20; steps = 425
    for i in range(steps + 1):
        t = i / steps; x = start_x + t * (end_x - start_x); y = start_y + t * (end_y - start_y)
        ox.append(x); oy.append(y)
    start_x, start_y = 10, 55; end_x, end_y = 25, 45; steps = 325
    for i in range(steps + 1):
        t = i / steps; x = start_x + t * (end_x - start_x); y = start_y + t * (end_y - start_y)
        ox.append(x); oy.append(y)
    start_x, start_y = 30, 0; end_x, end_y = 45, 10; steps = 325
    for i in range(steps + 1):
        t = i / steps; x = start_x + t * (end_x - start_x); y = start_y + t * (end_y - start_y)
        ox.append(x); oy.append(y)

    # Time-related Cost Zone (tc_x, tc_y)
    tc_x, tc_y = [], []
    for i in range(10, 26):
        for j in range(20, 46):
            tc_x.append(i); tc_y.append(j)
    
    # Fuel Consumption Cost Zone (fc_x, fc_y)
    fc_x, fc_y = [], []
    for i in range(30, 46):
        for j in range(10, 36):
            fc_x.append(i); fc_y.append(j)

    # --- TASK 2 OPTIMIZATION LOOP ---
    best_time = float('inf')
    best_y_start = -1
    best_path_rx, best_path_ry = [], []
    
    # CORRECTED: Loop through all possible vertical start positions for the 5-unit jet stream
    # The map goes from y=-10 to y=60. 
    # If y_start is 56, the range is 56, 57, 58, 59, 60 (5 units, up to the max boundary).
    # So we loop from -10 up to 57 (exclusive) to include 56 (inclusive).
    for y_start in range(-10, 57, 1):
        
        # 1. Dynamically define the Jet Stream Area (rc_x, rc_y)
        # Requirement: Span across the map laterally (x=-10 to x=60) and 5-unit length vertically
        rc_x, rc_y = [], []
        # X range from -10 to 60 (full lateral span)
        for i in range(-10, 61): 
            # Y range from y_start (inclusive) to y_start + 5 (exclusive) = 5 units total
            # e.g., range(10, 15) gives 10, 11, 12, 13, 14 (5 points)
            for j in range(y_start, y_start + 5): 
                rc_x.append(i); rc_y.append(j)
        
        # 2. Configure the A* Planner
        # J_max_discount set to 0.05 (5% reduction as per task)
        a_star = AStarPlanner(ox, oy, grid_size, robot_radius,
                              fc_x, fc_y, tc_x, tc_y,
                              rc_x, rc_y,
                              jet_vx=1.0, jet_vy=1.0, # Jet stream direction (e.g., 45 degrees)
                              J_max_discount=0.05,  
                              J_counter_penalty=0.0) # No counter penalty specified
        
        # 3. Run the planner
        rx, ry, current_time = a_star.planning(sx, sy, gx, gy)
        
        # 4. Check for the best result
        if current_time < best_time:
            best_time = current_time
            best_y_start = y_start
            best_path_rx = rx
            best_path_ry = ry

    # --- FINAL RESULTS AND ANALYSIS ---
    print("\n" + "#"*70)
    print("      TASK 2 OPTIMIZATION AND COST ANALYSIS COMPLETE")
    print("#"*70)
    
    if best_y_start != -1:
        # Redefine the optimal jet stream area for accurate plotting and reporting
        # This uses the best_y_start found in the loop
        optimal_rc_x, optimal_rc_y = [], []
        for i in range(-10, 61):
            for j in range(best_y_start, best_y_start + 5):
                optimal_rc_x.append(i); optimal_rc_y.append(j)

        print(f"**OPTIMAL JET STREAM PLACEMENT FOUND**")
        print(f"Vertical start position (y-start): {best_y_start:.0f}")
        print(f"Jet Stream Vertical Span: Y={best_y_start} to Y={best_y_start + 4} (5 units)")
        print(f"Optimal total trip time (T_best): {best_time:.2f} minutes")
        
        # Run the full cost analysis for the optimal time
        perform_cost_analysis(best_time)
        
        # --- PLOT THE FINAL OPTIMAL PATH AND JET STREAM ---
        if show_animation:
            plt.figure(figsize=(10, 8))
            plt.plot(ox, oy, ".k", label="Obstacles")
            plt.plot(sx, sy, "og", markersize=8, label="Start (0, 0)")
            plt.plot(gx, gy, "xb", markersize=8, label="Goal (50, 50)")
            
            # Plot high-cost zones
            plt.plot(fc_x, fc_y, "oy", markersize=2, alpha=0.5, label="Fuel Cost Zone")
            plt.plot(tc_x, tc_y, "or", markersize=2, alpha=0.5, label="Time Cost Zone")
            
            # Plot optimal jet stream in a light color
            plt.plot(optimal_rc_x, optimal_rc_y, "c", markersize=2, alpha=0.4, label="Optimal Jet Stream") 
            
            # Plot the resulting optimal path
            plt.plot(best_path_rx, best_path_ry, "-r", linewidth=3, label="Optimal Path")
            
            plt.title(f"Optimal Path with Jet Stream at Y={best_y_start} | Time: {best_time:.2f} min")
            plt.xlabel("X Coordinate")
            plt.ylabel("Y Coordinate")
            plt.grid(True)
            plt.axis("equal")
            plt.legend()
            plt.show()

    else:
        print("No path found for any jet stream placement. Check map setup.")

if __name__ == '__main__':
    main()
