
import numpy as np
import math
import matplotlib.pyplot as plt
from bezier_path import path

k  = 1.0 #control gain
Kp = 1.0 #speed proportional gain
dt = 0.1 #[s] time tick
WB = 2.9 #[m] wheel base of vehicle
max_steer = np.radians(30.0)  # [rad] max steering angle


class State(object):
    """
    Class representing the state of a vehicle.

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        super(State, self).__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, acceleration, delta):
        """
        Update the state of the vehicle.

        Stanley Control uses bicycle model.

        :param acceleration: (float) Acceleration
        :param delta: (float) Steering
        """
        delta = np.clip(delta, -max_steer, max_steer)

        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += self.v / WB * np.tan(delta) * dt
        self.yaw = normalize_angle(self.yaw)
        self.v += acceleration * dt


def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


def pid_speed_control(target, current,yaw):
    """
    Proportional control for the speed.

    :param target: (float)
    :param current: (float)
    :return: (float)
    """
    if yaw>0.5:
        target+=0.5       
    return Kp * (target - current)


def stanley_control(state, cx, cy, cyaw, last_target_idx):
    """
    Stanley steering control.

    :param state: (State object)
    :param cx: ([float])
    :param cy: ([float])
    :param cyaw: ([float])
    :param last_target_idx: (int)
    :return: (float, int)
    """
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)
    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, state.v)
    # Steering control
    delta = theta_e + theta_d

    return delta, current_target_idx


def direksiyon(state_yaw):
    heading=[]
    heading.append(state_yaw)
    for i in range(len(heading)-1):
        pass


def calc_target_index(state, cx, cy):
    """
    Compute index in the trajectory list of the target.

    :param state: (State object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """
    # Calc front axle position
    fx = state.x + WB * np.cos(state.yaw)
    fy = state.y + WB * np.sin(state.yaw)

    # Search nearest point index
    dx = [fx - i for i in cx]
    dy = [fy - i for i in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                      -np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle




def calc_cyaw_matris(cx,cy):
    cyaw=[]
    for i in range(len(cx)-1):
        dx = cx[i+1] - cx[i]
        dy = cy[i+1] - cy[i]
        cyaw.append(np.arctan2(dy,dx))
        print("current=",cx[i],cy[i],"\nnext=",cx[i+1],cy[i+1],"\t\tyaw=",cyaw[i],"\n")
    #son indexte düz devam et
    ddx=cx[len(cx)-1] - cx[len(cx)-2]
    ddy=cx[len(cy)-1] - cx[len(cy)-2] 
    cyaw.append(np.arctan2(ddy,ddx))
    return cyaw



def near_point_index_in_path(state,cx,cy):
    dx = [state.x - i for i in cx]
    dy = [state.y - i for i in cy]
    d = np.hypot(dx, dy)
    ind = np.argmin(d)
    return ind

def error_calc(state,cx,cy,nearest_point_ind):
    dx=cx[nearest_point_ind]- state.x
    dy=cy[nearest_point_ind]- state.y
    hip=math.sqrt(dx**2+dy**2) 
    return hip



def main():
    """Plot an example of Stanley steering control on a bezier path."""
 
    #  target course
    cx,cy=path()
    cyaw=calc_cyaw_matris(cx,cy)

    target_speed = 30.0 / 3.6  # [m/s]
    max_simulation_time = 100.0

    # Initial state
    state = State(x=-0.0, y=0.0, yaw=0.0, v=0.0)

    last_idx = len(cx) - 1
    time = 0.0

    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_idx, _ = calc_target_index(state, cx, cy)


    error=0
    show_animation=True
    while max_simulation_time >= time and last_idx > target_idx:
        
        time += dt

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        
        
        nearest_point_ind=near_point_index_in_path(state,cx,cy)
        instant_error=error_calc(state,cx,cy,nearest_point_ind)
        error += instant_error
        
        print(f"current==>{state.x},{state.y}\tnearest point==>{cx[nearest_point_ind]},{cy[nearest_point_ind]}\tinstant error==>{instant_error}\ttotal error==>{error}")

                
        ai = pid_speed_control(target_speed, state.v,state.yaw)
        di, target_idx = stanley_control(state, cx, cy, cyaw, target_idx)
        state.update(ai, di)

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(cx, cy, ".r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.plot(cx[target_idx], cy[target_idx], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.001)

    # Test
    assert last_idx >= target_idx, "Cannot reach goal"

    if show_animation:  # pragma: no cover
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        plt.subplots(1)
        plt.plot(t, [i * 3.6 for i in v], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()


if __name__ == '__main__':
    main()
