#--- AIM: Develop a program to compute the inverse kinematics of a 
# 3R planar serial robot by finding the optimal set of joint 
# parameters closer to a given 2D position of the end-effector, and show the simulation. ---#


import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def forward_kinematics(theta, links):
    t1, t2, t3 = theta
    l1, l2, l3 = links

    x1 = l1*np.cos(t1)
    y1 = l1*np.sin(t1)

    x2 = x1 + l2 * np.cos(t1 + t2)
    y2 = y1 + l2 * np.sin(t1 + t2)

    x3 = x2 + l3 * np.cos(t1 + t2 + t3)
    y3 = y2 + l3 * np.sin(t1 + t2 + t3)

    return np.array([[0,0],[x1,y1],[x2,y2],[x3,y3]])


def analytical_ik(target, links, phi_samples = 300):
    x, y = target
    l1, l2, l3 = links

    solutions = []

    for phi in np.linspace(-np.pi, np.pi, phi_samples):

    
        xw = x - l3 * np.cos(phi)
        yw = y - l3 * np.sin(phi)

        r2 = xw**2 + yw**2

        c2 = (r2 - l1**2 - l2**2)/(2*l1*l2)

        if abs(c2) > 1:
            continue

        for sign in [1, -1]:

            s2 = sign * np.sqrt(1-c2**2)
            theta2 = np.arctan2(s2, c2)

            theta1 = np.arctan2(yw, xw) - \
                     np.arctan2(l2 * s2, l1 + l2 * c2)

            theta3 = phi - theta1 - theta2

            solutions.append(np.array([theta1,theta2,theta3]))

    return solutions


def choose_optimal(solutions, theta_prev):

    best = None
    best_cost = 1e9

    for s in solutions:
        cost = np.linalg.norm(s - theta_prev)

        if cost < best_cost:
            best_cost = cost
            best = s

    return best

def simulate(theta, links, target):

    fig, ax = plt.subplots()

    L = sum(links) + 1
    ax.set_xlim(-L, L)
    ax.set_ylim(-L, L)
    ax.set_aspect('equal')

    line, = ax.plot([], [], 'o-', lw=4)
    ax.plot(target[0], target[1], 'rx', markersize = 12)

    def update(frame):
        t = theta * frame
        pts = forward_kinematics(t, links)
        line.set_data(pts[:,0], pts[:,1])
        return line,

    ani = FuncAnimation(
        fig,
        update,
        frames = np.linspace(0, 1, 60),
        interval = 40
    )

    plt.show()


def main():

    print("\n 3R Planar Robot Inverse Kinematics\n")

    l1 = float(input("Enter link1 length:"))
    l2 = float(input("Enter link2 length:"))
    l3 = float(input("Enter link3 length:"))

    x = float(input("Enter target x:"))
    y = float(input("Enter target y:"))

    links = [l1, l2, l3]
    target = np.array([x, y])

    theta_prev = np.array([0.0,0.0,0.0])

    solutions = analytical_ik(target, links)

    if len(solutions) == 0:
        print("\n Target not reached")
        return

    theta_opt = choose_optimal(solutions, theta_prev)

    print("\n Optimal Solution (radians)")
    print(theta_opt)

    print("\n Optimal Solution (degrees)")
    print(np.degrees(theta_opt))

    print("\n Number of solutions found:", len(solutions))

    simulate(theta_opt, links, target)

if __name__ == "__main__":
    main()