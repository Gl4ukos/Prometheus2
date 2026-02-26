#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from Prometheus2_config import *
import matplotlib as mpl


# -------- Robot Parameters --------
class Prometheus2_state:
    def __init__(self):
        self.l1 = config["l1"]
        self.l2a = config["l2a"]
        self.l2b = config["l2b"]
        self.l3 = config["l3"]
        self.theta0 = config["theta0"]
        self.theta1 = config["theta1"]
        self.theta2a = config["theta2a"]
        self.theta2b = config["theta2b"]
        self.theta3 = config["theta3"]
        self.theta0_bounds = config["theta0_bounds"]
        self.theta1_bounds = config["theta1_bounds"]
        self.theta2a_bounds = config["theta2a_bounds"]
        self.theta3_bounds = config["theta3_bounds"]
        self.T01 = self.trans_x(1)
        self.T02 = self.trans_x(1)
        self.T03a = self.trans_x(1)
        self.T03b = self.trans_x(1)
        self.T04 = self.trans_x(1)

        self.angle_step = np.deg2rad(5)
        self.ee_step = 0.1

        self.l1_pos = np.array([[0],
                                [0],
                                [0],
                                [1]])
        self.l2a_pos = np.array([[0],
                                 [0],
                                 [0],
                                 [1]])
        self.l2b_pos = np.array([[0],
                                 [0],
                                 [0],
                                 [1]])
        self.ee_pos =  np.array([[0],
                                 [0],
                                 [0],
                                 [1]])
        self.compute_fk()
    
    def print_positions(self):
        print("EE POSITION: ", [round(float(x),4) for x in self.ee_pos])

    def print_angles(self):
        print("Theta0: ", self.theta0)
        print("Theta1: ", self.theta1)
        print("Theta2: ", self.theta2a)
        print("Theta3: ", self.theta3)

    def rot_x(self, theta):
        rot = np.array([[1,0,0,0],
                       [0, np.cos(theta), -np.sin(theta),0],
                       [0, np.sin(theta), np.cos(theta),0],
                       [0, 0, 0, 1]])
        return rot

    def rot_y(self, theta):
        rot = np.array([[np.cos(theta),0,np.sin(theta), 0],
                       [0, 1, 0, 0],
                       [-np.sin(theta), 0, np.cos(theta), 0],
                       [0 ,0 ,0 ,1]])
        return rot

    def rot_z(self, theta):
        rot = np.array([[np.cos(theta), -np.sin(theta), 0, 0],
                        [np.sin(theta), np.cos(theta), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
        return rot

    def trans_x(self, l):
        t = np.array([[1, 0, 0, l],
                      [0, 1, 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        return t
    
    def trans_y(self, l):
        t = np.array([[1, 0, 0, 0],
                      [0, 1, 0, l],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        return t
    
    def trans_z(self, l):
        t = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, l],
                      [0, 0, 0, 1]])
        return t

    def compute_fk(self):
        # transformation matrices
        self.T01 = self.rot_y(self.theta0)
        T12 = self.rot_z(self.theta1) @ self.trans_x(self.l1)
        T23a = self.rot_z(self.theta2a) @ self.trans_x(self.l2a)
        T23b = self.rot_z(self.theta2b) @ self.trans_x(self.l2b)
        T34 = self.rot_z(self.theta3) @ self.trans_x(self.l3)

        self.T02 = self.T01 @ T12
        self.T03a = self.T02 @ T23a
        self.T03b = self.T03a @ T23b
        self.T04 = self.T03b @ T34

        self.l1_pos = self.T02 @ [0,0,0,1]
        self.l2a_pos = self.T03a @ [0,0,0,1]
        self.l2b_pos = self.T03b @ [0,0,0,1]
        self.ee_pos = self.T04 @ [0,0,0,1]


    def compute_jacobian(self):

        # Make sure FK is current
        self.compute_fk()

        # End effector position
        p_e = self.ee_pos[0:3].flatten()

        # Joint origins
        p0 = np.array([0,0,0])
        p1 = self.T01[0:3, 3]
        p2 = self.T02[0:3, 3]
        p3 = self.T03a[0:3, 3]

        # Joint axes in world frame
        # Joint 0 rotates about X axis
        z0 = self.T01[0:3, 0]

        # Others rotate about Z axis
        z1 = self.T02[0:3, 2]
        z2 = self.T03a[0:3, 2]
        z3 = self.T03b[0:3, 2]

        # Build Jacobian
        J = np.zeros((3,4))

        J[:,0] = np.cross(z0, p_e - p0)
        J[:,1] = np.cross(z1, p_e - p1)
        J[:,2] = np.cross(z2, p_e - p2)
        J[:,3] = np.cross(z3, p_e - p3)

        return J

    def move_ee(self, target:np.array, threshold = 1e-4):

        for i in range(20):
            self.compute_fk()
            error = target[0:3] - self.ee_pos[0:3]
            if abs(np.sum(error)) < threshold:
                break
            J = self.compute_jacobian()
            dtheta = np.linalg.pinv(J) @ error

            step = 0.3
            self.theta0 += step * dtheta[0]
            self.theta1 += step * dtheta[1]
            self.theta2a += step * dtheta[2]
            self.theta3 += step * dtheta[3]
            


arm_state = Prometheus2_state()

# -------- Plot Setup --------
mpl.rcParams['toolbar'] = 'None'
for key in mpl.rcParams:
    if key.startswith('keymap.'):
        mpl.rcParams[key] = []
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-4, 4)
ax.set_ylim(-4, 4)
ax.set_zlim(-4, 4)
ax.set_aspect('equal')
ax.set_xscale("linear")
ax.set_yscale("linear")
ax.grid()

# Initial plot lines
line1, = ax.plot([], [], [], 'o-',color = "black")
line2a, = ax.plot([], [], [], 'o-', color = "purple")
line2b, = ax.plot([], [], [], 'o-', color = "purple")
line3, = ax.plot([], [], [],'o-', color = "black")
ee_point = ax.plot([], [], [],'ro')[0]

def update_plot():
    global arm_state

    # Link 1
    line1.set_data(
        [0, arm_state.l1_pos[0]],
        [0, arm_state.l1_pos[1]]
    )
    line1.set_3d_properties(
        [0, arm_state.l1_pos[2]]
    )

    # Link 2a
    line2a.set_data(
        [arm_state.l1_pos[0], arm_state.l2a_pos[0]],
        [arm_state.l1_pos[1], arm_state.l2a_pos[1]]
    )
    line2a.set_3d_properties(
        [arm_state.l1_pos[2], arm_state.l2a_pos[2]]
    )

    # Link 2b
    line2b.set_data(
        [arm_state.l2a_pos[0], arm_state.l2b_pos[0]],
        [arm_state.l2a_pos[1], arm_state.l2b_pos[1]]
    )
    line2b.set_3d_properties(
        [arm_state.l2a_pos[2], arm_state.l2b_pos[2]]
    )

    # Link 3
    line3.set_data(
        [arm_state.l2b_pos[0], arm_state.ee_pos[0]],
        [arm_state.l2b_pos[1], arm_state.ee_pos[1]]
    )
    line3.set_3d_properties(
        [arm_state.l2b_pos[2], arm_state.ee_pos[2]]
    )

    # End effector
    ee_point.set_data(
        [arm_state.ee_pos[0]],
        [arm_state.ee_pos[1]]
    )
    ee_point.set_3d_properties(
        [arm_state.ee_pos[2]]
    )

    fig.canvas.draw_idle()

# -------- Keyboard Controls --------
def on_key(event):
    global arm_state


    if event.key == 'f':
        arm_state.theta0 += arm_state.angle_step
        arm_state.compute_fk()
        arm_state.print_angles()
    elif event.key == 'v':
        arm_state.theta0 -= arm_state.angle_step
        arm_state.compute_fk()
        arm_state.print_angles()
    elif event.key == 'g':
        arm_state.theta1 += arm_state.angle_step
        arm_state.compute_fk()
        arm_state.print_angles()
    elif event.key == 'b':
        arm_state.theta1 -= arm_state.angle_step
        arm_state.compute_fk()
        arm_state.print_angles()
    elif event.key == 'h':
        arm_state.theta2a += arm_state.angle_step
        arm_state.compute_fk()
        arm_state.print_angles()
    elif event.key == 'n':
        arm_state.theta2a -= arm_state.angle_step
        arm_state.compute_fk()
        arm_state.print_angles()
    elif event.key == 'j':
        arm_state.theta3 += arm_state.angle_step
        arm_state.compute_fk()
        arm_state.print_angles()
    elif event.key == 'm':
        arm_state.theta3 -= arm_state.angle_step
        arm_state.compute_fk()
        arm_state.print_angles()

    elif event.key == 'o':
        target = np.round(arm_state.ee_pos + np.array([0, arm_state.ee_step, 0, 0]), 4)
        arm_state.move_ee(target)
    elif event.key == 'l':
        target = np.round(arm_state.ee_pos + np.array([0, -arm_state.ee_step, 0, 0]), 4)
        arm_state.move_ee(target)
    elif event.key == ';':
        target = np.round(arm_state.ee_pos + np.array([arm_state.ee_step, 0, 0, 0]), 4)
        arm_state.move_ee(target)
    elif event.key == 'k':
        target = np.round(arm_state.ee_pos + np.array([-arm_state.ee_step, 0, 0, 0]), 4)
        arm_state.move_ee(target)
    elif event.key == 'p':
        target = np.round(arm_state.ee_pos + np.array([0, 0, arm_state.ee_step, 0]), 4)
        arm_state.move_ee(target)
    elif event.key == 'i':
        target = np.round(arm_state.ee_pos + np.array([0, 0, -arm_state.ee_step, 0]), 4)
        arm_state.move_ee(target)


    update_plot()
    arm_state.print_positions()
    

fig.canvas.mpl_connect('key_press_event', on_key)
update_plot()
arm_state.print_positions()
plt.title("Prometheus2 movement simulator")
plt.show()