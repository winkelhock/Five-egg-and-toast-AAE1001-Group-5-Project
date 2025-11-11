import math
import matplotlib.pyplot as plt

# === INPUT PARAMETERS (from your assignment) ===
total_passengers = 3300
max_flights = 13
fuel_cost_per_kg = 0.85          # C_F
base_time_cost = 12              # Base C_T
extra_cost_per_50 = 2
fuel_per_engine = 20             # kg/min per engine
fixed_cost_twin = 2000
fixed_cost_four = 2500

# Possible seat counts
capacities = [250, 300, 350, 400, 450]

# === USER INPUT: Cruise time in minutes ===
print("HK Thunder-450 Aircraft Cost Optimizer")
print("="*50)
T_best = float(input("Enter cruise time T_best (minutes), e.g. 600 for 10-hour flight: "))

# === CALCULATION ===
results = []

print("\n{:<8} {:<7} {:<8} {:<10} {:<12} {:<15}".format(
    "Seats", "Engines", "Flights", "C_T($/min)", "VarRate", "Total Cost($)"))
print("-"*68)

for seats in capacities:
    # Number of flights needed (round up)
    flights = math.ceil(total_passengers / seats)
    
    # Engine rule
    if seats >= 300:
        engines = 4
        fixed_cost = fixed_cost_four
    else:
        engines = 2
        fixed_cost = fixed_cost_twin
    
    # Skip if exceeds 13 flights
    if flights > max_flights:
        print(f"{seats:<8} {engines:<7} {flights:<8} EXCEEDS 13 FLIGHTS → INVALID")
        continue
    
    # Time cost C_T
    extra_blocks = max(0, (seats - 100) // 50)
    C_T = base_time_cost + extra_cost_per_50 * extra_blocks
    
    # Fuel burn ΔF
    delta_F = engines * fuel_per_engine
    
    # Variable cost per minute
    var_rate = fuel_cost_per_kg * delta_F + C_T
    
    # Total weekly cost
    total_cost = flights * (var_rate * T_best + fixed_cost)
    
    # Save result
    results.append({
        'seats': seats,
        'engines': engines,
        'flights': flights,
        'C_T': C_T,
        'var_rate': var_rate,
        'total_cost': total_cost
    })
    
    print(f"{seats:<8} {engines:<7} {flights:<8} {C_T:<10.1f} {var_rate:<12.1f} {total_cost:,.2f}")

# === FIND WINNER ===
winner = min(results, key=lambda x: x['total_cost'])

print("\n" + "="*50)
print("OPTIMAL DESIGN (LOWEST COST)")
print("="*50)
print(f"Aircraft Name     : HK Thunder-{winner['seats']}")
print(f"Passenger Capacity: {winner['seats']} seats")
print(f"Engine Count      : {winner['engines']} engines")
print(f"Flights needed    : {winner['flights']} per week")
print(f"Total Weekly Cost : ${winner['total_cost']:,.2f}")
print(f"Cruise time used  : {T_best} minutes")

# === BONUS: Plot ===
seats_list = [r['seats'] for r in results]
costs = [r['total_cost'] for r in results]

plt.figure(figsize=(10, 6))
plt.bar(seats_list, costs, color=['red' if s == winner['seats'] else 'skyblue' for s in seats_list])
plt.xlabel('Passenger Capacity')
plt.ylabel('Total Weekly Operating Cost ($)')
plt.title(f'Cost vs Capacity (T_best = {T_best} min) → Winner: {winner["seats"]} seats')
plt.grid(axis='y', alpha=0.3)

# Add cost labels on bars
for i, cost in enumerate(costs):
    plt.text(seats_list[i], cost + 10000, f"${cost:,.0f}", ha='center', fontsize=10)

plt.tight_layout()
plt.show()
