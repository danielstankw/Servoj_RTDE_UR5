import numpy as np
import time
import matplotlib.pyplot as plt


class PathPlanTranslation(object):
    def __init__(self, pose_init, pose_desired, total_time):
        
        self.position_init = pose_init[:3]
        self.position_des = pose_desired[:3]

        self.tfinal = total_time

    def trajectory_planning(self, t):
        X_init = self.position_init[0]
        Y_init = self.position_init[1]
        Z_init = self.position_init[2]

        X_final = self.position_des[0]
        Y_final = self.position_des[1]
        Z_final = self.position_des[2]

        # position
        x_traj = (X_final - X_init) / (self.tfinal ** 3) * (
                6 * (t ** 5) / (self.tfinal ** 2) - 15 * (t ** 4) / self.tfinal + 10 * (t ** 3)) + X_init
        y_traj = (Y_final - Y_init) / (self.tfinal ** 3) * (
                6 * (t ** 5) / (self.tfinal ** 2) - 15 * (t ** 4) / self.tfinal + 10 * (t ** 3)) + Y_init
        z_traj = (Z_final - Z_init) / (self.tfinal ** 3) * (
                6 * (t ** 5) / (self.tfinal ** 2) - 15 * (t ** 4) / self.tfinal + 10 * (t ** 3)) + Z_init
        position = np.array([x_traj, y_traj, z_traj])

        # velocities
        vx = (X_final - X_init) / (self.tfinal ** 3) * (
                30 * (t ** 4) / (self.tfinal ** 2) - 60 * (t ** 3) / self.tfinal + 30 * (t ** 2))
        vy = (Y_final - Y_init) / (self.tfinal ** 3) * (
                30 * (t ** 4) / (self.tfinal ** 2) - 60 * (t ** 3) / self.tfinal + 30 * (t ** 2))
        vz = (Z_final - Z_init) / (self.tfinal ** 3) * (
                30 * (t ** 4) / (self.tfinal ** 2) - 60 * (t ** 3) / self.tfinal + 30 * (t ** 2))
        velocity = np.array([vx, vy, vz])

        # acceleration
        ax = (X_final - X_init) / (self.tfinal ** 3) * (
                120 * (t ** 3) / (self.tfinal ** 2) - 180 * (t ** 2) / self.tfinal + 60 * t)
        ay = (Y_final - Y_init) / (self.tfinal ** 3) * (
                120 * (t ** 3) / (self.tfinal ** 2) - 180 * (t ** 2) / self.tfinal + 60 * t)
        az = (Z_final - Z_init) / (self.tfinal ** 3) * (
                120 * (t ** 3) / (self.tfinal ** 2) - 180 * (t ** 2) / self.tfinal + 60 * t)
        acceleration = np.array([ax, ay, az])

        return [position, velocity, acceleration]

    # ******* EXAMPLE **********


if __name__ == "__main__":
    pose_init = np.array([0.334010, -0.515243, 0.2760, 2.41052252419476, 0.9219877505542503, 0.22313174746991263])
    pose_des = np.array([0.953948, -0.015090, 0.6758, 2.41052252419476, 0.9219877505542503, 0.22313174746991263])

    tfinal = 5

    trajectory = PathPlanTranslation(pose_init, pose_des, tfinal)

    posx = []
    posy = []
    posz = []
    v_x = []
    v_y = []
    v_z = []
    a_x = []
    a_y = []
    a_z = []
    time_range = []

    t_start = time.time()
    while time.time() - t_start < tfinal:
        t_current = time.time() - t_start
        [position, velocity, acceleration] = trajectory.trajectory_planning(t_current)

        # position
        posx.append(position[0])
        posy.append(position[1])
        posz.append(position[2])
        # # velocity
        v_x.append(velocity[0])
        v_y.append(velocity[1])
        v_z.append(velocity[2])
        # # acceleration
        a_x.append(acceleration[0])
        a_y.append(acceleration[1])
        a_z.append(acceleration[2])

        time_range.append(t_current)

    # plotting using pyplot
    plt.figure()
    plt.plot(time_range, posx, label='X position')
    plt.plot(time_range, posy, label='Y position')
    plt.plot(time_range, posz, label='Z position')
    plt.legend()
    plt.grid()
    plt.ylabel('Position [m]')
    plt.xlabel('Time [s]')

    plt.figure()
    plt.plot(time_range, v_x, label='X velocity')
    plt.plot(time_range, v_y, label='Y velocity')
    plt.plot(time_range, v_z, label='Z velocity')
    plt.legend()
    plt.grid()
    plt.ylabel('Velocity[m/s]')
    plt.xlabel('Time [s]')

    plt.figure()
    plt.plot(time_range, a_x, label='X acc')
    plt.plot(time_range, a_y, label='Y acc')
    plt.plot(time_range, a_z, label='Z acc')
    plt.legend()
    plt.grid()
    plt.ylabel('Acceleration [m/s^2]')
    plt.xlabel('Time [s]')
    plt.show()
