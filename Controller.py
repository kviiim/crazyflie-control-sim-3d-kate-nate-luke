import math
import numpy as np
from math import sin, cos

class Controller3D():
    """
    This class computes the commanded thrusts (N) to be applied to the quadrotor plant.

    You are to implement the "compute_commands" method.
    """
    def __init__(self, cfparams, pid_gains, dt):
        """
        Inputs:
        - cfparams (CrazyflieParams dataclass):     model parameter class for the crazyflie
        - pid_gains (dict):                         pid gains

        N.B. pid_gains is a dictionary structure where the keys are 'kp_x', 'kd_z', etc.
        """
        self.params = cfparams
        self.pid_gains = pid_gains

        self.x_error = 0
        self.y_error = 0
        self.z_error = 0

        self.p_error = 0
        self.q_error = 0
        self.r_error = 0
        # set control gains here


    def compute_commands(self, setpoint, state):
        """
        Inputs:
        - setpoint (TrajPoint dataclass):   the desired control setpoint
        - state (State dataclass):          the current state of the system
        Returns:
        - U (np.array):     array of control inputs {u1-u4}

        N.B. TrajPoint is a new dataclass. Please check it out from the utils.py script
        """
        print(f"State X/Y:{state.x_pos} {state.y_pos} \nSetpoint:{setpoint.x_pos} {setpoint.y_pos}")
        # your code here
        z_error = setpoint.z_pos-state.z_pos
        z_error_dot = z_error-self.z_error

        z_dot_scaling = 0
        z_des_dot_dot = z_dot_scaling * (setpoint.z_vel - state.z_vel) # = self.trajectory.acl_z
        U1 =  self.params.mass * (z_des_dot_dot +
                                 self.pid_gains["kd_z"] * z_error_dot +
                                 self.pid_gains["kp_z"] * z_error +
                                 self.params.g)
        

        x_error = float(setpoint.x_pos)-float(state.x_pos)
        x_error_dot = x_error-self.x_error
        x_des_dot_dot = 0 # = self.trajectory.acl_x

        x_control_dot_dot = (x_des_dot_dot +
                                 self.pid_gains["kd_x"] * x_error_dot +
                                 self.pid_gains["kp_x"] * x_error)
        print(f"X_error: {x_error},x_error_dot: {x_error_dot},x_control_dot_dot: {x_control_dot_dot}")

        y_error = float(setpoint.y_pos)-float(state.y_pos)
        y_error_dot = y_error-self.y_error
        y_des_dot_dot = 0  # = self.trajectory.acl_y

        y_control_dot_dot = (y_des_dot_dot +
                            self.pid_gains["kd_y"] * y_error_dot +
                            self.pid_gains["kp_y"] * y_error)
        print(f"Y_error: {x_error},Y_error_dot: {y_error_dot},y_control_dot_dot: {y_control_dot_dot}")


        phi_des = (x_control_dot_dot*math.sin(setpoint.psi) - y_control_dot_dot*math.cos(setpoint.psi))/self.params.g
        theta_des = (x_control_dot_dot*math.cos(setpoint.psi) + y_control_dot_dot*math.sin(setpoint.psi))/self.params.g
        psi_des = setpoint.psi

        print(f"Phi_des: {phi_des},State_phi:{state.phi} theta_des:{theta_des}, psi_des{psi_des}")
        p_des_dot = 0  # = self.trajectory.acl_p
        p_error = phi_des-state.phi
        p_error_dot = p_error-self.p_error # Is this just equal to state.p
        print(f"P_error:{p_error}, p_error_dot:{p_error_dot}")
        U2 =  (p_des_dot +
                self.pid_gains["kd_p"] * p_error_dot +
                self.pid_gains["kp_phi"] * p_error)

        q_des_dot = 0  # = self.trajectory.acl_p
        q_error = theta_des-state.theta
        q_error_dot = q_error-self.q_error # Is this just equal to state.p
        U3 =  (q_des_dot +
                self.pid_gains["kd_q"] * q_error_dot +
                self.pid_gains["kp_theta"] * q_error)


        r_des_dot = 0  # = self.trajectory.acl_p
        r_error = psi_des-state.psi
        r_error_dot = r_error-self.r_error # Is this just equal to state.p
        U4 =  (r_des_dot +
                self.pid_gains["kd_r"] * r_error_dot +
                self.pid_gains["kp_psi"] * r_error)



        self.x_error = x_error
        self.y_error = y_error
        self.z_error = z_error

        self.p_error = p_error
        self.q_error = q_error
        self.r_error = r_error

        print(f"U1: {U1},U2: {U2},U3: {U3},U4: {U4}")
        
        return np.array([float(U1),float(U2), float(U3), float(U4)])# float(U3),float(U4)])
