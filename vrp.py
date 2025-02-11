from gurobipy import Model, GRB, quicksum
import math

def read_vrp_data(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
        
        vehicle_line_index = -1
        for i, line in enumerate(lines):
            if "VEHICLE" in line:
                vehicle_line_index = i + 2
                break
        
        if vehicle_line_index == -1:
            raise ValueError("Vehicle information not found in the input file.")
        
        vehicle_info = lines[vehicle_line_index].strip().split()
        num_vehicles = int(vehicle_info[0])
        vehicle_capacity = int(vehicle_info[1])

        customer_line_index = -1
        for i, line in enumerate(lines):
            if "CUST NO." in line:
                customer_line_index = i + 1
                break

        if customer_line_index == -1:
            raise ValueError("Customer information not found in the input file.")

        customers = []
        for line in lines[customer_line_index:]:
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

    return num_vehicles, vehicle_capacity, customers

def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def solve_vrp_with_time_windows(file_path):
    num_vehicles, vehicle_capacity, customers = read_vrp_data(file_path)
    
    if not customers:
        raise ValueError("No customer data was loaded. Check the dataset format.")

    depot = customers[0]  # Ensure depot exists
    customers = customers[1:]  # Remove depot
    num_customers = len(customers)

    print(f"Depot: {depot}")
    print(f"Number of customers: {num_customers}")

    # Fix the distance matrix calculation to ensure index safety
    distance = [[0.0] * (num_customers + 1) for _ in range(num_customers + 1)]
    for i in range(num_customers + 1):
        for j in range(num_customers + 1):
            if i == 0:  # Depot to customer
                x1, y1 = depot['xcoord'], depot['ycoord']
            else:  # Customer to customer
                x1, y1 = customers[i - 1]['xcoord'], customers[i - 1]['ycoord']

            if j == 0:
                x2, y2 = depot['xcoord'], depot['ycoord']
            else:
                x2, y2 = customers[j - 1]['xcoord'], customers[j - 1]['ycoord']

            distance[i][j] = calculate_distance(x1, y1, x2, y2)

    print("Distance matrix computed successfully.")

    
    model = Model("VRP_with_Time_Windows")
    
    x = model.addVars(num_customers + 1, num_customers + 1, num_vehicles, vtype=GRB.BINARY, name="x")
    t = model.addVars(num_customers + 1, num_vehicles, vtype=GRB.CONTINUOUS, name="t")
    
    model.setObjective(quicksum(distance[i][j] * x[i, j, k] 
                                for i in range(num_customers + 1) 
                                for j in range(num_customers + 1) 
                                for k in range(num_vehicles) if i != j), GRB.MINIMIZE)
    
    for j in range(1, num_customers + 1):
        model.addConstr(quicksum(x[i, j, k] for i in range(num_customers + 1) for k in range(num_vehicles) if i != j) == 1)
    
    for k in range(num_vehicles):
        model.addConstr(quicksum(x[0, j, k] for j in range(1, num_customers + 1)) == 1)

    for j in range(num_customers + 1):
        for k in range(num_vehicles):
            model.addConstr(quicksum(x[i, j, k] for i in range(num_customers + 1) if i != j) == quicksum(x[j, i, k] for i in range(num_customers + 1) if i != j))
    
    for k in range(num_vehicles):
        model.addConstr(quicksum(customers[j - 1]['demand'] * x[i, j, k] for i in range(num_customers + 1) for j in range(1, num_customers + 1) if i != j) <= vehicle_capacity)
    
    for i in range(num_customers + 1):
        for j in range(num_customers + 1):
            if i != j:
                for k in range(num_vehicles):
                    model.addConstr(t[j, k] >= t[i, k] + (customers[i - 1]['service_time'] if i != 0 else 0) - (1 - x[i, j, k]) * 1e6)
    
    for j in range(1, num_customers + 1):
        for k in range(num_vehicles):
            model.addConstr(t[j, k] >= customers[j - 1]['ready_time'])
            model.addConstr(t[j, k] <= customers[j - 1]['due_date'])
    
    model.optimize()
    
    if model.status == GRB.OPTIMAL:
        print("Optimal solution found!")
        print(f"Total travel distance: {model.objVal}")
        for k in range(num_vehicles):
            print(f"Route for vehicle {k}:")
            for i in range(num_customers + 1):
                for j in range(num_customers + 1):
                    if i != j and x[i, j, k].X > 0.5: 
                        print(f"Vehicle {k} travels from {i} to {j}")
            route = []
            current = 0  # Start at the depot
            while True:
                next_node = None
                for j in range(num_customers + 1):
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
