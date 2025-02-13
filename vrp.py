import math
from gurobipy import Model, GRB, quicksum

def read_vrp_data(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()

        # Find vehicle capacity
        for i, line in enumerate(lines):
            if "VEHICLE" in line:
                vehicle_info = lines[i + 2].strip().split()
                num_vehicles = int(vehicle_info[0])
                vehicle_capacity = int(vehicle_info[1])
                break
        else:
            raise ValueError("Vehicle information not found.")

        # Find customer data
        customers = []
        for i, line in enumerate(lines):
            if "CUST NO." in line:
                for line in lines[i + 1:]:
                    data = line.strip().split()
                    if len(data) == 7:
                        customers.append({
                            'cust_no': int(data[0]),
                            'xcoord': float(data[1]),
                            'ycoord': float(data[2]),
                            'demand': float(data[3]),
                            'ready_time': float(data[4]),
                            'due_date': float(data[5]),
                            'service_time': float(data[6])
                        })
                break

    return num_vehicles, vehicle_capacity, customers

def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def solve_vrp_with_time_windows(file_path):
    num_vehicles, vehicle_capacity, customers = read_vrp_data(file_path)

    if not customers:
        raise ValueError("No customer data was loaded. Check the dataset format.")

    num_customers = len(customers) - 1  # Exclude depot
    num_nodes = num_customers + 2  # Depot + Customers + Auxiliary Depot
    depot = customers[0]  # First node is the depot
    customers = customers[1:]  # Remaining are customers
    auxiliary_depot = num_nodes - 1  # Last index is auxiliary depot

    print(f"Depot: {depot}")
    print(f"Number of customers: {num_customers}")

    # Distance matrix with (num_nodes) size
    distance = [[0.0] * num_nodes for _ in range(num_nodes)]

    for i in range(num_nodes):
        for j in range(num_nodes):
            # Assign coordinates for i
            if i == 0 or i == auxiliary_depot:  # First depot or auxiliary depot
                x1, y1 = depot['xcoord'], depot['ycoord']
            else:  # Customers
                x1, y1 = customers[i - 1]['xcoord'], customers[i - 1]['ycoord']

            # Assign coordinates for j
            if j == 0 or j == auxiliary_depot:  # First depot or auxiliary depot
                x2, y2 = depot['xcoord'], depot['ycoord']
            else:  # Customers
                x2, y2 = customers[j - 1]['xcoord'], customers[j - 1]['ycoord']

            distance[i][j] = calculate_distance(x1, y1, x2, y2)

    print("Distance matrix computed successfully.")

    model = Model("VRP_with_Time_Windows")

    x = model.addVars(num_nodes, num_nodes, num_vehicles, vtype=GRB.BINARY, name="x")
    t = model.addVars(num_nodes, num_vehicles, vtype=GRB.CONTINUOUS, name="t")

    model.setObjective(
        quicksum(distance[i][j] * x[i, j, k] 
                 for i in range(num_nodes) 
                 for j in range(num_nodes) 
                 for k in range(num_vehicles) if i != j), 
        GRB.MINIMIZE)

    # Each customer must be visited exactly once
    for i in range(1, num_nodes - 1): 
        model.addConstr(quicksum(x[i, j, k] for j in range(num_nodes) for k in range(num_vehicles) if i != j) == 1)

    # Capacity constraint
    for k in range(num_vehicles):
        model.addConstr(quicksum(customers[i - 1]['demand'] * x[i, j, k] 
                                 for i in range(1, num_nodes - 1) for j in range(num_nodes) if i != j) <= vehicle_capacity)

    # Each vehicle starts at the depot
    for k in range(num_vehicles):
        model.addConstr(quicksum(x[0, j, k] for j in range(1, num_nodes)) == 1)

    # Each vehicle ends at the auxiliary depot
    for k in range(num_vehicles):
        model.addConstr(quicksum(x[i, auxiliary_depot, k] for i in range(num_nodes - 1)) == 1)

    # Flow constraints to ensure continuity
    for j in range(1, num_nodes - 1):
        for k in range(num_vehicles):
            model.addConstr(quicksum(x[i, j, k] for i in range(num_nodes) if i != j) == quicksum(x[j, i, k] for i in range(num_nodes) if i != j))

    # Time constraints
    for i in range(num_nodes):
        for j in range(1, num_nodes):
            if i != j:
                for k in range(num_vehicles):
                    service_time = customers[i - 1]['service_time'] if (1 <= i < num_nodes - 1) else 0
                    model.addConstr(t[j, k] >= t[i, k] + service_time - (1 - x[i, j, k]) * 1e6)

    for i in range(1, num_nodes - 1):
        for k in range(num_vehicles):
            model.addConstr(t[i, k] >= customers[i - 1]['ready_time'])
            model.addConstr(t[i, k] <= customers[i - 1]['due_date'])

    model.optimize()

    if model.status == GRB.OPTIMAL:
        print("Optimal solution found!")
        print(f"Total travel distance: {model.objVal}")
        for k in range(num_vehicles):
            print(f"Route for vehicle {k}:")
            route = []
            current = 0  # Start at depot
            while True:
                next_node = None
                for j in range(num_nodes):
                    if j != current and x[current, j, k].X > 0.5:
                        next_node = j
                        break
                if next_node is None:
                    break
                route.append(next_node)
                current = next_node
            print(f"Vehicle {k} route: {route}")

    else:
        print("No optimal solution found.")

file_path = './rc1/rc101.txt'  
solve_vrp_with_time_windows(file_path)
