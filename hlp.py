from gurobipy import Model, GRB, quicksum
import math
import random
import numpy as np

def read_hlp_data(file_path):
    with open(file_path, 'r') as file:
        n = int(file.readline().strip())
        
        x = []
        y = []
        for _ in range(n):
            coords = file.readline().strip().split()
            x.append(float(coords[0]))
            y.append(float(coords[1]))
        
        w = []
        for _ in range(n):
            row = file.readline().strip().split()
            w.append([float(val) for val in row])
        
        p = int(file.readline().strip())
        delta = float(file.readline().strip())
        alpha = float(file.readline().strip())
        chi = float(file.readline().strip())
    
    return n, x, y, w, delta, alpha, chi

def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def read_fix_data(n,fix_path,cap_path):
    with open(fix_path, 'r') as file:
        f = []
        for _ in range(n):
            cost = file.readline().strip().split()
            f.append(float(cost[0]))

    with open(cap_path, 'r') as file:
        u = []
        for _ in range(n):
            cap = file.readline().strip().split()
            u.append(float(cap[0]))

    return f, u

def solve_hlp(file_path, fixCost_path, cap_path):
    n, x_coord, y_coord, w, delta, alpha, chi = read_hlp_data(file_path)
    d = [[calculate_distance(x_coord[i], y_coord[i], x_coord[j], y_coord[j]) for j in range(n)] for i in range(n)]
    f, u = read_fix_data(n, fixCost_path, cap_path)
    
    model = Model("HLP")
    
    z = model.addVars(n, n, vtype=GRB.BINARY, name="z")
    x = model.addVars(n, n, n, n, vtype=GRB.CONTINUOUS, name="x")
    
    model.setObjective(
        quicksum(f[k] * z[k,k] for k in range(n)) + 
        quicksum(w[i][j] * (delta*d[i][k] + alpha*d[k][l] + chi*d[l][j]) * x[i,j,k,l] for i in range(n) for j in range(n) for k in range(n) for l in range(n)),
        GRB.MINIMIZE
    )

    for i in range(n):
        model.addConstr(
            quicksum(z[i,k] for k in range(n)) == 1,
            name=f"hubs_or_assigned_{i}"
        )

    for i in range(n):
        for k in range(n):
            model.addConstr(
                z[i,k] <= z[k,k],
                name=f"nonhub_to_hub_{i}_{k}"
            )

    for k in range(n):
        model.addConstr(
            quicksum(quicksum(w[i][j] for j in range(n)) * z[i,k] for i in range(n)) <= u[k]*z[k,k],
            name=f"hubs_capacity_{k}"
        )

    for i in range(n):
        for j in range(n):
            for k in range(n):
                model.addConstr(
                        quicksum(x[i,j,k,l] for l in range(n)) == z[i,k],
                        name=f"hub_{i}_{k}_utilization"
                        )

    for i in range(n):
        for j in range(n):
            for l in range(n):
                model.addConstr(
                        quicksum(x[i,j,k,l] for k in range(n)) == z[j,l],
                        name=f"hub_{j}_{l}_utilization"
                        )

    model.optimize()
    
    if model.status == GRB.OPTIMAL:
        print("Optimal solution found!")
        print(f"Total cost: {model.objVal}")
        print("Selected hubs:")
        for i in range(n):
            for k in range(n):
                if z[i,k].X > 0.5 and i == k:
                    print(f"Node {i},{k} is a hub.")
    else:
        print("No optimal solution found.")

file_path = './AP/10.3.txt'
fixCost_path = './AP/FcostL.10.txt'
cap_path = './AP/CapT.10.txt'
solve_hlp(file_path, fixCost_path, cap_path)
