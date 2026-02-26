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
        T01 = self.rot_x(self.theta0)
        T12 = self.rot_z(self.theta1) @ self.trans_y(self.l1)
        T23a = self.rot_z(self.theta2a) @ self.trans_y(self.l2a)
        T23b = self.rot_z(self.theta2b) @ self.trans_y(self.l2b)
        T34 = self.rot_z(self.theta3) @ self.trans_y(self.l3)

        T02 = T01 @ T12
        T03a = T02 @ T23a
        T03b = T03a @ T23b
        T04 = T03b @ T34

        self.l1_pos = T02 @ [0,0,0,1]
        self.l2a_pos = T03a @ [0,0,0,1]
        self.l2b_pos = T03b @ [0,0,0,1]
        self.ee_pos = T04 @ [0,0,0,1]


    def compute_jacobian(self, eps=1e-6):
        pass
    

    def move_ee(self, target_x, target_y):
        pass


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
ax.set_zlim(-4,4)
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


    if event.key == 'zf':
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
        arm_state.move_ee(arm_state.ee_pos[0], arm_state.ee_pos[1] + arm_state.ee_step)
    elif event.key == 'l':
        arm_state.move_ee(arm_state.ee_pos[0], arm_state.ee_pos[1] - arm_state.ee_step)
    elif event.key == ';':
        arm_state.move_ee(arm_state.ee_pos[0] + arm_state.ee_step, arm_state.ee_pos[1])
    elif event.key == 'k':
        arm_state.move_ee(arm_state.ee_pos[0] - arm_state.ee_step, arm_state.ee_pos[1])
    
    update_plot()
    arm_state.print_positions()
    

fig.canvas.mpl_connect('key_press_event', on_key)
update_plot()
plt.title("Prometheus2 movement simulator")
plt.show()