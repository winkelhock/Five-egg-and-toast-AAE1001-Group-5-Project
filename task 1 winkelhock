import math
import heapq
from typing import List, Tuple, Dict

class PathPlanner:
    def __init__(self, width=60, height=60):
        self.width = width
        self.height = height
        self.obstacles = set()
        self.fuel_areas = set()
        self.time_areas = set()
        
    def setup_group5_environment(self):
        """Set up the environment for Group 5 based on the provided map"""
        # Start and goal nodes for Group 5
        self.start = (0, 0)
        self.goal = (50, 50)
        
        # Obstacles for Group 5 (from the map)
        obstacles = [
            (30, 0), (10, 55), (30, 35), (25, 45), (10, 20), 
            (20, 0), (25, 20), (10, 45), (45, 10), (30, 20)
        ]
        
        # Fuel-consuming areas (you need to identify from the map)
        self.fuel_areas = {(30, 35), (25, 45), (10, 45)}
        
        # Time-consuming areas (you need to identify from the map)
        self.time_areas = {(30, 0), (20, 0), (25, 20)}
        
        self.obstacles = set(obstacles)
    
    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Euclidean distance heuristic"""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def get_neighbors(self, node: Tuple[int, int]) -> List[Tuple[Tuple[int, int], float]]:
        """Get neighboring nodes with movement costs"""
        x, y = node
        neighbors = []
        
        # All possible movements (8 directions)
        directions = [
            (1, 0, 1.0),   # right
            (-1, 0, 1.0),  # left
            (0, 1, 1.0),   # up
            (0, -1, 1.0),  # down
            (1, 1, math.sqrt(2)),   # up-right
            (1, -1, math.sqrt(2)),  # down-right
            (-1, 1, math.sqrt(2)),  # up-left
            (-1, -1, math.sqrt(2))  # down-left
        ]
        
        for dx, dy, cost in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx <= self.width and 0 <= ny <= self.height:
                if (nx, ny) not in self.obstacles:
                    # Apply penalties for special areas
                    final_cost = cost
                    if (nx, ny) in self.fuel_areas:
                        final_cost *= 1.15  # 15% penalty for fuel areas
                    if (nx, ny) in self.time_areas:
                        final_cost *= 1.30  # 30% penalty for time areas
                    
                    neighbors.append(((nx, ny), final_cost))
        
        return neighbors
    
    def a_star_search(self) -> Tuple[List[Tuple[int, int]], float]:
        """A* search algorithm to find the shortest path"""
        open_set = []
        heapq.heappush(open_set, (0, self.start))
        
        came_from = {}
        g_score = {self.start: 0}
        f_score = {self.start: self.heuristic(self.start, self.goal)}
        
        while open_set:
            current_f, current = heapq.heappop(open_set)
            
            if current == self.goal:
                return self.reconstruct_path(came_from, current), g_score[current]
            
            for neighbor, cost in self.get_neighbors(current):
                tentative_g = g_score[current] + cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, self.goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        return [], float('inf')  # No path found
    
    def reconstruct_path(self, came_from: Dict, current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Reconstruct the path from start to goal"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

class AircraftCostCalculator:
    def __init__(self):
        self.aircraft_data = {
            'A321neo': {
                'fuel_rate': 54,
                'capacity': 200,
                'time_cost_low': 10,
                'time_cost_medium': 15,
                'time_cost_high': 20,
                'fixed_cost': 1800
            },
            'A330-900neo': {
                'fuel_rate': 84,
                'capacity': 300,
                'time_cost_low': 15,
                'time_cost_medium': 21,
                'time_cost_high': 27,
                'fixed_cost': 2000
            },
            'A350-900': {
                'fuel_rate': 90,
                'capacity': 350,
                'time_cost_low': 20,
                'time_cost_medium': 27,
                'time_cost_high': 34,
                'fixed_cost': 2500
            }
        }
    
    def calculate_trip_cost(self, aircraft: str, trip_time: float, time_cost_level: str, 
                          fuel_cost: float, num_flights: int) -> float:
        """Calculate total trip cost for an aircraft"""
        data = self.aircraft_data[aircraft]
        
        # Get time cost based on level
        if time_cost_level == 'low':
            time_cost = data['time_cost_low']
        elif time_cost_level == 'medium':
            time_cost = data['time_cost_medium']
        else:  # high
            time_cost = data['time_cost_high']
        
        # Calculate cost using the formula: C = C_F * ΔF * T_best + C_T * T_best + C_c
        trip_fuel = data['fuel_rate'] * trip_time
        cost_per_flight = (fuel_cost * trip_fuel * trip_time + 
                          time_cost * trip_time + 
                          data['fixed_cost'])
        
        return cost_per_flight * num_flights

def task1_group5():
    """Main function for Task 1 - Group 5"""
    print("=" * 70)
    print("TASK 1 - FLIGHT PATH PLANNING & AIRCRAFT SELECTION - GROUP 5")
    print("=" * 70)
    
    # Step 1: Find shortest path
    print("\nSTEP 1: Finding shortest path...")
    planner = PathPlanner()
    planner.setup_group5_environment()
    
    path, total_time = planner.a_star_search()
    
    if not path:
        print("No path found!")
        return
    
    print(f"Shortest path found with {len(path)} nodes")
    print(f"Total travel time: {total_time:.2f} minutes")
    print(f"Path: {path[:3]} ... {path[-3:]}")
    
    # Step 2: Calculate costs for different scenarios
    calculator = AircraftCostCalculator()
    
    scenarios = [
        {
            'name': 'Scenario 1',
            'passengers': 3300,
            'max_flights': 13,
            'time_cost': 'medium',
            'fuel_cost': 0.85
        },
        {
            'name': 'Scenario 2', 
            'passengers': 1500,
            'max_flights': 7,
            'time_cost': 'high',
            'fuel_cost': 0.96
        },
        {
            'name': 'Scenario 3',
            'passengers': 2250,
            'max_flights': 25,
            'time_cost': 'low',
            'fuel_cost': 0.78
        }
    ]
    
    print("\n" + "=" * 70)
    print("AIRCRAFT COST ANALYSIS")
    print("=" * 70)
    
    for scenario in scenarios:
        print(f"\n{scenario['name']}:")
        print(f"  Passengers: {scenario['passengers']}")
        print(f"  Max flights: {scenario['max_flights']}")
        print(f"  Time cost: {scenario['time_cost']}")
        print(f"  Fuel cost: ${scenario['fuel_cost']}/kg")
        
        best_aircraft = None
        best_cost = float('inf')
        
        for aircraft in ['A321neo', 'A330-900neo', 'A350-900']:
            capacity = calculator.aircraft_data[aircraft]['capacity']
            num_flights = math.ceil(scenario['passengers'] / capacity)
            
            if num_flights > scenario['max_flights']:
                print(f"  {aircraft}: Not feasible (requires {num_flights} flights)")
                continue
            
            total_cost = calculator.calculate_trip_cost(
                aircraft, total_time, scenario['time_cost'],
                scenario['fuel_cost'], num_flights
            )
            
            print(f"  {aircraft}: {num_flights} flights, Total cost: ${total_cost:,.2f}")
            
            if total_cost < best_cost:
                best_cost = total_cost
                best_aircraft = aircraft
        
        if best_aircraft:
            capacity = calculator.aircraft_data[best_aircraft]['capacity']
            num_flights = math.ceil(scenario['passengers'] / capacity)
            print(f"  → RECOMMENDATION: {num_flights} flights of {best_aircraft}")
        else:
            print("  → No feasible aircraft found!")

if __name__ == "__main__":
    task1_group5()
