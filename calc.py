import sympy as sp

vx, vy, cvx, cvy, omega, omegaCurr, rx, ry, dt = sp.symbols('vx vy cvx cvy omega omegaCurr rx ry dt')

vel = sp.Matrix([vx, vy])
currVel = sp.Matrix([cvx, cvy])

alpha = (omega - omegaCurr) / dt

constr = ((vel - currVel) / dt + sp.Matrix([-alpha * ry, alpha * rx])).dot(
    (vel - currVel) / dt + sp.Matrix([-alpha * ry, alpha * rx])
)

gradient = sp.simplify(sp.Matrix([sp.diff(constr, var) for var in [vx, vy, omega]]))
hessian = sp.simplify(sp.Matrix([[sp.diff(gradient[i], var) for var in [vx, vy, omega]] for i in range(3)]))

print("Gradient:")
sp.pprint(gradient)
print("\nHessian:")
sp.pprint(hessian)
