import numpy as np
import sympy as sp

cos = np.vectorize(sp.cos)
sin = np.vectorize(sp.sin)
tan = np.vectorize(sp.tan)
acos = np.vectorize(sp.acos)
atan2 = np.vectorize(sp.atan2)

l1, l2, d, gamma, b, L, dx = sp.symbols('l1, l2, d, gamma, b, L, dx')

s = np.array([1, -1])
phi = -s * gamma

v1 = l1 * cos(phi)
h1 = -l1 * sin(phi)

v2 = d - v1
h2 = s*(l2*l2 - v2*v2)**.5

x0 = h1 + h2

x = s*dx + x0
l3 = (x*x + d*d)**0.5
alpha = (atan2(d, x) + acos((l1*l1 + l3*l3 - l2*l2)/(2*l1*l3)) + gamma - sp.pi/2) * s

r = L/tan(alpha) + s*b/2
kappa = 1/r

# r_approx, kappa_approx = sp.symbols('r_approx, kappa_approx')
# eq_kappa = kappa.mean() - kappa_approx
# eq_r = r.mean() - r_approx
