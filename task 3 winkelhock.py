class AdvancedAircraftDesigner:
    def __init__(self):
        # Scenario 1 parameters
        self.passengers_needed = 3300
        self.max_flights_per_week = 13
        self.time_cost_level = "medium"
        self.fuel_cost_per_kg = 0.85
        
        # Base parameters
        self.base_time_cost = 12  # $/min
        self.time_cost_increment = 2  # $/min per 50 passengers
        self.engine_fuel_consumption = 20  # kg/min per engine
        self.twin_engine_fixed_cost = 2000  # $
        self.four_engine_fixed_cost = 2500  # $
        
    def design_aircraft_family(self, trip_time: float) -> List[Dict]:
        """Design multiple aircraft options and return the best ones"""
        designs = []
        
        # Test different passenger capacities
        for capacity in range(100, 451, 50):
            num_flights = math.ceil(self.passengers_needed / capacity)
            
            if num_flights > self.max_flights_per_week:
                continue
                
            # Determine engine configuration
            if capacity >= 300:
                engine_count = 4
                fixed_cost = self.four_engine_fixed_cost
            else:
                engine_count = 2
                fixed_cost = self.twin_engine_fixed_cost
            
            # Calculate time cost based on capacity
            capacity_increments = (capacity - 100) // 50
            time_cost = self.base_time_cost + (capacity_increments * self.time_cost_increment)
            
            # Calculate fuel consumption
            fuel_consumption_rate = engine_count * self.engine_fuel_consumption
            
            # Calculate costs
            trip_fuel = fuel_consumption_rate * trip_time
            cost_per_flight = (self.fuel_cost_per_kg * trip_fuel * trip_time + 
                             time_cost * trip_time + fixed_cost)
            total_cost = cost_per_flight * num_flights
            
            # Calculate cost per passenger
            cost_per_passenger = total_cost / self.passengers_needed
            
            design = {
                'name': f"PolyU-A{capacity}",
                'passenger_capacity': capacity,
                'engine_count': engine_count,
                'num_flights': num_flights,
                'fuel_consumption_rate': fuel_consumption_rate,
                'time_cost_per_min': time_cost,
                'fixed_cost': fixed_cost,
                'cost_per_flight': cost_per_flight,
                'total_cost': total_cost,
                'cost_per_passenger': cost_per_passenger
            }
            
            designs.append(design)
        
        # Sort by total cost
        return sorted(designs, key=lambda x: x['total_cost'])
    
    def print_design_comparison(self, designs: List[Dict], trip_time: float):
        """Print comparison of different aircraft designs"""
        print("=" * 80)
        print("AIRCRAFT DESIGN COMPARISON - GROUP 5")
        print("=" * 80)
        print(f"Scenario: {self.passengers_needed} passengers, {self.max_flights_per_week} max flights")
        print(f"Trip time: {trip_time} minutes, Fuel cost: ${self.fuel_cost_per_kg}/kg")
        print()
        
        print(f"{'Aircraft':<15} {'Capacity':<10} {'Engines':<8} {'Flights':<8} {'Total Cost':<12} {'Cost/Passenger':<15}")
        print("-" * 80)
        
        for design in designs[:5]:  # Show top 5 designs
            print(f"{design['name']:<15} {design['passenger_capacity']:<10} {design['engine_count']:<8} "
                  f"{design['num_flights']:<8} ${design['total_cost']:>9,.0f} ${design['cost_per_passenger']:>7.2f}")
        
        # Show best design details
        best = designs[0]
        print("\n" + "=" * 80)
        print("OPTIMAL AIRCRAFT DESIGN")
        print("=" * 80)
        self.print_design_details(best, trip_time)
        
        # Compare with existing aircraft
        print("\n" + "=" * 80)
        print("COMPARISON WITH EXISTING AIRCRAFT")
        print("=" + "=" * 80)
        self.compare_with_existing(best, trip_time)
    
    def print_design_details(self, design: Dict, trip_time: float):
        """Print detailed specifications of a design"""
        print(f"Aircraft Name: {design['name']}")
        print(f"Passenger Capacity: {design['passenger_capacity']}")
        print(f"Engine Configuration: {design['engine_count']}-engine")
        print(f"Number of Flights Required: {design['num_flights']}")
        print(f"Fuel Consumption Rate: {design['fuel_consumption_rate']} kg/min")
        print(f"Time Cost: ${design['time_cost_per_min']}/min")
        print(f"Fixed Cost per Flight: ${design['fixed_cost']}")
        print()
        
        # Cost breakdown
        fuel_cost = self.fuel_cost_per_kg * design['fuel_consumption_rate'] * trip_time * trip_time
        time_cost = design['time_cost_per_min'] * trip_time
        fixed_cost = design['fixed_cost']
        
        print("COST BREAKDOWN PER FLIGHT:")
        print(f"  Fuel Cost: ${fuel_cost:,.2f}")
        print(f"  Time Cost: ${time_cost:,.2f}")
        print(f"  Fixed Cost: ${fixed_cost:,.2f}")
        print(f"  Total per Flight: ${design['cost_per_flight']:,.2f}")
        print(f"  Weekly Total: ${design['total_cost']:,.2f}")
        print(f"  Cost per Passenger: ${design['cost_per_passenger']:.2f}")
    
    def compare_with_existing(self, best_design: Dict, trip_time: float):
        """Compare designed aircraft with existing models"""
        existing_costs = {
            'A321neo': 81840,  # Example costs from Task 1
            'A330-900neo': 83048,
            'A350-900': 81240
        }
        
        print(f"Our designed aircraft: {best_design['name']}")
        print(f"Total weekly cost: ${best_design['total_cost']:,.2f}")
        print()
        
        for aircraft, cost in existing_costs.items():
            savings = cost - best_design['total_cost']
            percentage = (savings / cost) * 100
            print(f"vs {aircraft}: Savings of ${savings:,.2f} ({percentage:.1f}%)")

def task3_group5():
    """Main function for Task 3 - Aircraft Design"""
    print("\n" + "=" * 80)
    print("TASK 3 - AIRCRAFT DESIGN OPTIMIZATION - GROUP 5")
    print("=" + "=" * 80)
    
    designer = AdvancedAircraftDesigner()
    
    # Use the trip time calculated in Task 1 (you need to replace this with actual value)
    # For demonstration, using 120 minutes
    trip_time = 120  # Replace with your actual calculated trip time
    
    # Design aircraft family
    designs = designer.design_aircraft_family(trip_time)
    
    if designs:
        designer.print_design_comparison(designs, trip_time)
    else:
        print("No feasible aircraft designs found within constraints!")

# Main execution function for all tasks
def main():
    """Run all tasks for Group 5"""
    print("GROUP 5 - AAE1001 PROJECT TASKS 1-3")
    print("=" * 50)
    
    # Run Task 1
    task1_group5()
    
    # Run Task 2  
    task2_group5()
    
    # Run Task 3
    task3_group5()

if __name__ == "__main__":
    main()
