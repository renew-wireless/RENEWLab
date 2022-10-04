import gurobipy
from gurobipy import Model
from gurobipy import GRB
from gurobipy import quicksum

def mlSolver(csi, y, x):
    results = []
    status = []
    m = csi.shape[2] # number of TX antennas
    n = csi.shape[1] # number of RX antennas
    for idx_, Y in enumerate(y):

        H = csi[idx_]
        model = Model('mimo')
        k = len(x) # size of modulation
        Z = model.addVars(m, k, vtype=GRB.BINARY, name='Z')
        S = model.addVars(m, ub=max(x)+.1, lb=min(x)-0.1,  name='S')
        E = model.addVars(n, ub=200.0, lb=-200.0, vtype=GRB.CONTINUOUS, name='E')
        model.update() 

        # Defining S[i]
        for i in range(m):
            model.addConstr(S[i] == quicksum(Z[i,j] * x[j] for j in range(k)))

        # Constraint on Z[i,j]
        model.addConstrs((Z.sum(j,'*') == 1
                         for j in range(m)), name='Const1')


        # Defining E
        for i in range(n):
            E[i] = quicksum(H[i][j] * S[j] for j in range(m)) - Y[i]

        # Defining the objective function
        obj = E.prod(E)  
        model.setObjective(obj, GRB.MINIMIZE)
        model.Params.logToConsole = 0
        model.setParam('TimeLimit', 100)
        model.update()

        model.optimize()

        # Retrieve optimization result
        solution = model.getAttr('X', S)
        status.append(model.getAttr(GRB.Attr.Status) == GRB.OPTIMAL)
        x_est = []
        for nnn in solution:
             x_est.append(solution[nnn])
        results.append(x_est)
    return results



