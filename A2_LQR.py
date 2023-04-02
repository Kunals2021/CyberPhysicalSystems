import matplotlib.pyplot as plt
import numpy as np
import math
import matplotlib.markers as mmarkers

class LQR_DifferentialRobot:
    def __init__(self, current_state, desired_state):
        self.current_x = current_state[0]
        self.current_y = current_state[1]
        self.current_theta = current_state[2]
        self.desired_x = desired_state[0]
        self.desired_y = desired_state[1]
        self.desired_theta = desired_state[2]
        
        # Initialize linear and angular velocities
        self.velocity = 1
        self.angular_vel = 0.5
    
        # Set the State Cost Matrix "Q"
        #self.Q = "TO DO -- 0.5pt"
        self.Q = [[100,0,0],
                  [0,100,0],
                  [0,0,100]]
        
        # Set the Control Input Cost Matrix "R"
        #self.R = "TO DO -- 0.5pt"
        self.R = [[50,0],
                  [0,50]]
        
        # Set the time-step (time interval) in [second]
        #self.delta_t = "TO DO -- 0.5pt"
        self.delta_t = 0.1

        # Set the maximum value of yaw (theta) angle and linear velocity
        self.max_theta = math.pi * 2
        #self.max_vel = "TO DO -- 0.5pt"
        self.max_vel = -10

        self.beam_current_x = [0,0]
        self.beam_current_y = [0,0]
        
        self.beam_desired_x = [0,0]
        self.beam_desired_y = [0,0]
        
        self.t1 = 0
        self.t2 = 0
        
        # For saving the trajectory followed by the robot
        self.x_trajectory = [self.current_x]
        self.y_trajectory = [self.current_y]
        
    ''' @brief UpdateStates: for updating the current states, as well as the trajectoy points (x,y) '''
    def UpdateStates(self, velocity, angular_velocity):
        #self.current_x, self.current_y, self.current_theta = self.UpdateKinematics("TO DO -- 0.5pt", "TO DO -- 0.5pt")
        self.current_x, self.current_y, self.current_theta = self.UpdateKinematics(velocity,angular_velocity)
        self.x_trajectory.append(self.current_x)
        self.y_trajectory.append(self.current_y)
        return self.current_x, self.current_y, self.current_theta
    
    ''' @brief UpdateKinematics: for updating the robot's states using X(t+1) = A * X(t) + B* U(t)'''
    def UpdateKinematics(self, velocity, angular_velocity):
        curr_state = np.array([self.current_x, self.current_y, self.current_theta])
        curr_control = np.array([self.velocity, self.angular_vel])
        self.velocity = velocity
        self.angular_vel = angular_velocity

        # Following 2 lines can be used to limit max velocity and max angular velocity
        # self.angular_vel = np.clip(angular_velocity, -self.max_theta, self.max_theta)
        # self.velocity = np.clip(velocity,-self.max_vel, self.max_vel)
        print('-------------------------------------------------------')
        print('Linear Vel.: %.3f, Angular Vel.: %.3f'% (self.velocity, self.angular_vel))
        print('Current State: ', curr_state)
        
        #return ((self.A @ "TO DO -- 0.5pt") + (self.B @ "TO DO -- 0.5pt"))
        return self.A @ curr_state + self.B @ curr_control

    '''@brief SetRobotHeading: for visualizing the current and desired heading of the robot '''
    def SetRobotHeading(self):
        beam_len = 2
        self.beam_current_x[0] = self.current_x
        self.beam_current_x[1] = self.current_x + beam_len * math.cos(self.current_theta) 
        self.beam_current_y[0] = self.current_y 
        self.beam_current_y[1] = self.current_y + beam_len * math.sin(self.current_theta) 

        self.beam_desired_x[0] = self.desired_x
        self.beam_desired_x[1] = self.desired_x + beam_len * math.cos(self.desired_theta)  
        self.beam_desired_y[0] = self.desired_y  
        self.beam_desired_y[1] = self.desired_y + beam_len * math.sin(self.desired_theta) 
   
    
    ''' @brief render: for robot plotting, considering the robot as  a square marker'''
    def render(self, marker_symbol):
        plt.cla()
        # Rotate robot marker with respect to theta
        self.SetRobotHeading()
        deg_curr = math.degrees(self.current_theta)
        deg_end = math.degrees(self.desired_theta)
        self.t1 = mmarkers.MarkerStyle(marker='{m}'.format(m=marker_symbol))  
        self.t1._transform = self.t1.get_transform().rotate_deg(deg_curr)
        self.t2 = mmarkers.MarkerStyle(marker='{m}'.format(m=marker_symbol)) 
        self.t2._transform = self.t2.get_transform().rotate_deg(deg_end)
        
        plt.scatter(self.current_x, self.current_y, marker = self.t1, s=100, color='b')
        plt.scatter(self.desired_x, self.desired_y, marker = self.t2, s=100, color='r')

        plt.plot(self.beam_current_x, self.beam_current_y, color='b')
        plt.plot(self.beam_desired_x, self.beam_desired_y, color="r")

        plt.plot(self.x_trajectory, self.y_trajectory, '--', alpha=0.8)
 
        plt.xlim(-100, 100)
        plt.ylim(-100, 100)
        plt.pause(0.01)    

    def getA(self, velocity, theta, dt):
        #self.A = "TO DO -- 1.5pt"
        self.A = [[1,0,-velocity * math.sin(theta)*dt],
                  [0,1,velocity * math.cos(theta)*dt],
                  [0,0,1]]
        #A= np.array([self.A])

    def getB(self, theta, dt):
        #self.B = "TO DO -- 1pt"
        self.B = [[math.cos(theta)*dt,0],
                  [math.sin(theta)*dt,0],
                  [0,dt]]
         #B = np.array([self.B])
    
    def ComputeLQR(self):
        N = 10
        P = [None] * (N+1)
        Qf = [[100,0,0],
              [0,100,0],
              [0,0,100]]
        P[N] = Qf

        #for i in range("TO DO -- 0.5pt"):
            #P[i-1] = "TO DO -- 1pt"
        
        A = np.array(self.A)
        B = np.array(self.B)
            
        for i in range(N):

            #P[N-1-i] = self.Q + np.linalg.inv(self.A)@P[N-i]@self.A-np.linalg.inv(self.A)@P[N-i]@self.B/(self.R + np.linalg.inv(self.B)@P[N-i]@self.B)@np.linalg.inv(self.B)@P[i]@self.A
            P[N-i-1] = self.Q + (A.T@P[N-i]@self.A) - (A.T@P[N-i]@self.B) @ ((np.linalg.inv(self.R + B.T@P[N-i]@self.B))) @ (B.T@P[N-i]@self.A)
        
        K = [None] * N
        u = [None] * N

        #for i in range ("TO DO -- 0.5pt"):
        for i in range (10):
            #K[i] = "TO DO -- 1pt"
            #u[i] = "TO DO -- 1pt"
            #K[i] = -(self.R + np.linalg.inv((self.B)@P[i+1]@self.B))
            K[i] = -(np.linalg.inv(self.R+(B.T@P[i+1]@self.B)) @ (B.T@P[i+1]@self.A))
            #u[i] = K[i]@self.state_error
            u[i] = K[i]@(self.desired_state - self.curr_state)

        # Optimal control is u_star
        #u_star = "TO DO -- 0.5pt"
        u_star = u[-1]
        return u_star
    
    ''' @brief LQRpath: for computing the optimal control inputs using LQR, updat the current states, then show the generated path'''    
    def LQRpath(self):
        for i in range(250):
            self.curr_state = np.array([self.current_x, self.current_y, self.current_theta])
            self.desired_state = np.array([self.desired_x, self.desired_y, self.desired_theta])
            #self.state_error = "TO DO -- 1pt"
            #mag_error = "TO DO -- 0.5pt"
            self.state_error = (self.desired_state - self.curr_state)
            #mag_error = math.sqrt(self.state_error[0]**2 + self.state_error[1]**2 + self.state_error[2]**2)
            mag_error = np.linalg.norm(self.state_error)

            
            #print('Mag. Error: ', mag_error)
           
            self.getA(self.velocity, self.current_theta, self.delta_t)
            self.getB(self.current_theta, self.delta_t)
            
            optimal_control_input = self.ComputeLQR()
            # Update the states given current control inputs
            #self.UpdateStates("TO DO -- 0.5pt", "TO DO -- 0.5pt")
            self.UpdateStates(optimal_control_input[0], optimal_control_input[1])
            if mag_error < .15:
                print('\n Ending states with error magnitued less than %.3f', mag_error)
                break
        
            # marker_symbol='4' for tri_right, ">"	￼	triangle_right, "s" for square
            self.render(marker_symbol='>')

if __name__ == "__main__":
    # Set the inital and desired pose of the robot 
    initial_state = np.array([0,0,0])
    desired_state = np.array([20,20, math.pi/4])
    robot = LQR_DifferentialRobot(initial_state, desired_state)
    robot.LQRpath()
