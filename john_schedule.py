import cplex
import numpy as np

# Create a new CPLEX problem instance
problem = cplex.Cplex()
problem.conflict.refine(problem.conflict.all_constraints())

# Define the number of courses and hours
num_courses = 10
num_hours = 9
req = 8

# Define the preferences for each section (i, k)
preferences = np.random.uniform(5, 15, (num_courses, num_hours))

# Create lists to store the binary variables x, y
x_types = [problem.variables.type.binary] * (num_hours * num_courses)
t_types = [problem.variables.type.integer] * (num_hours * num_courses)
y_types = [problem.variables.type.binary] * (num_hours * num_hours * num_courses * num_courses)

# Set the objective function to maximize the sum of John's preferences
x_coefficients = [preferences[k][i] for k in range(num_courses) for i in range(num_hours)]

# Set Lower and Upper bound
t_lb = [0] * (num_hours * num_courses)
t_ub = [num_hours-1] * (num_hours * num_courses)

# Add the binary variables to the problem
problem.variables.add(obj=x_coefficients, names=["x_{}_{}".format(i, k) for i in range(num_hours) for k in range(num_courses)], types=x_types)
problem.variables.add(lb = t_lb, ub = t_ub, names=["t_{}_{}".format(i, k) for i in range(num_hours) for k in range(num_courses)], types=t_types)
problem.variables.add(names=["y_{}_{}_{}_{}".format(i,j,k,l) for i in range(num_hours) for j  in range(num_hours) for k in range(num_courses) for l in range(num_courses)], types=y_types)

# Set the objective function to maximize the sum of John's preferences
problem.objective.set_sense(problem.objective.sense.maximize)

# Add constraints to ensure that each course is taken exactly once per day
for k in range(num_courses):
    constraint = cplex.SparsePair(ind=["x_{}_{}".format(i, k) for i in range(num_hours)], val=[1] * num_hours)
    problem.linear_constraints.add(names=["course_{}_constraint".format(k)], lin_expr=[constraint], senses='L', rhs=[1])

for i in range(num_hours):
    section_assign = cplex.SparsePair(ind=["x_{}_{}".format(i, k) for k in range(num_courses)], val=[1] * num_courses)
    problem.linear_constraints.add(names=["hour_{}_constraint".format(i)], lin_expr=[section_assign], senses='L', rhs=[1])
    for k in range(num_courses):
        ind = [
                "y_{}_{}_{}_{}".format(j, i, l, k)
                for j in range(num_hours)
                for l in range(num_courses)
                if i != j and k != l  
                ]
        val = [1] * len(ind)
        class_assign = cplex.SparsePair(ind=ind, val=val)
        problem.linear_constraints.add(names=["class_assign_{}_{}".format(i,k)], lin_expr=[class_assign], senses='L', rhs=[1])
        ind = [
                "y_{}_{}_{}_{}".format(i, j, k, l)
                for j in range(num_hours)
                for l in range(num_courses)
                if i != j and k != l  
                ]
        val = [1] * len(ind)
        class_assign = cplex.SparsePair(ind=ind, val=val)
        problem.linear_constraints.add(names=["class_assign_{}_{}".format(i,k)], lin_expr=[class_assign], senses='L', rhs=[1])
        for j in range(num_hours):
            for l in range(num_courses):
                if i != j and k != l:
                    subtour = cplex.SparsePair(
                            ind=["t_{}_{}".format(j, l), "t_{}_{}".format(i, k), "y_{}_{}_{}_{}".format(i, j, k, l)],
                            val=[1, -1, -num_hours]
                            )
                    problem.linear_constraints.add(
                            names=["subtour_{}_{}_{}_{}".format(i,j,k,l)],
                            lin_expr=[subtour],
                            senses='G',
                            rhs=[1-num_hours]
                            )
                    lr1 = cplex.SparsePair(
                            ind=["y_{}_{}_{}_{}".format(i, j, k, l), "x_{}_{}".format(i,k)],
                            val=[1, -1]
                            )
                    problem.linear_constraints.add(
                            names=["lr1_{}_{}_{}_{}".format(i,j,k,l)],
                            lin_expr=[lr1],
                            senses=["L"],
                            rhs=[0]
                            )
                    lr2 = cplex.SparsePair(
                            ind=["y_{}_{}_{}_{}".format(i, j, k, l), "x_{}_{}".format(j,l)],
                            val=[1, -1]
                            )
                    problem.linear_constraints.add(
                            names=["lr2_{}_{}_{}_{}".format(i,j,k,l)],
                            lin_expr=[lr2],
                            senses=["L"],
                            rhs=[0]
                            )

# Add constraints to ensure that John takes four courses per day
constraint_expr = cplex.SparsePair(ind=["x_{}_{}".format(i,k) for i in range(num_hours) for k in range(num_courses)],val=[1] * (num_hours * num_courses))
problem.linear_constraints.add(names=["sum_const"], lin_expr=[constraint_expr], senses='E', rhs=[req])

ind = ["y_{}_{}_{}_{}".format(i,j,k,l) for i in range(num_hours) for j in range(num_hours) for k in range(num_courses) for l in range(num_courses) if i != j and k != l]
val = [1] * len(ind)
constraint = cplex.SparsePair(ind=ind,val=val)
problem.linear_constraints.add(names=["sum_consty"], lin_expr=[constraint], senses='E', rhs=[req-1])

problem.write("john.lp")
# Solve the problem
problem.solve()

# Get the solution
solution = problem.solution.get_values()
problem.solution.write("optimal.txt")
# Print the solution
print("Solution:")
for i in range(num_hours):
    for k in range(num_courses):
        variable_index = i * num_courses + k  # Calculate the index based on i and k
        if solution[variable_index] > 0.5:
            print("John chooses section ({}, {})".format(i, k))


# Print the objective value
objective_value = problem.solution.get_objective_value()
print("Objective value:", objective_value)

