import numpy as np
from math import *

def cam2world(m):
    xc = 9.890661037168528e+02
    yc = 1.240887736413883e+03
    c = 1.000095457005999
    d = 1.348821585691861e-04
    e = -2.766447240107194e-05
    ss = 100.0*np.array([-5.506324725325824, 0, 0.000004199699653, -0.000000000365763, 0.000000000002906])
    ss = np.flip(ss, 0)

    scaling_factor = 13.5

    A_inv = np.array([[1., -d], [-e, c]])
    
    T = np.array([[xc, yc]]).T

    m_dash = A_inv@(scaling_factor*m - T)

    rho = np.linalg.norm(m_dash, axis=0)

    z = np.polyval(ss, rho)

    M = np.vstack((m_dash, z))

    M_norm = np.linalg.norm(M, axis=0)

    M = M / M_norm[None, :]

    return M

def compute_global_coordinates(m):
    M = cam2world(m)

    p0 = M[0, 0]
    q0 = M[1, 0]
    r0 = M[2, 0]

    p1 = M[0, 1]
    q1 = M[1, 1]
    r1 = M[2, 1]

    delta = 6

    a = p1**2 - p0**2
    b = 2*delta*p0*(p1**2 - 1)
    c = (delta**2)*(p1**2 - 1)

    d0_plus = (-b + sqrt(b**2 - 4*a*c)) / (2*a)
    d0_minus = (-b - sqrt(b**2 - 4*a*c)) / (2*a)

    d0 = d0_plus
    x = p0*d0
    d1 = (x + delta)/p1
    diff_y = abs(d0*q0 - d1*q1)
    diff_z = abs(d0*r0 - d1*r1)
    diff_plus = diff_y + diff_z

    d0 = d0_minus
    x = p0*d0
    d1 = (x + delta)/p1
    diff_y = abs(d0*q0 - d1*q1)
    diff_z = abs(d0*r0 - d1*r1)
    diff_minus = diff_y + diff_z

    if (diff_plus < diff_minus):
        result = d0_plus*np.array([[p0, q0, r0]]).T
    else:
        result = d0_minus*np.array([[p0, q0, r0]]).T

    print(d0_plus)
    print(d0_minus)

    # print(diff_plus)
    # print(diff_minus)

    return result


if __name__ == "__main__":
    m = np.array([[36.8, 40.6], [94.6, 94.6]])
    # m = np.array([[40.6, 36.8], [94.6, 94.6]])
    print(m)

    global_coords = compute_global_coordinates(m)
    print(global_coords)

    print(np.linalg.norm(global_coords))

