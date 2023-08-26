import numpy as np
import copy

# # hand model pass through via-point. No spring interaction.
# class Hand_Model:
#     def __init__(self, params, vp_cross_timestep):
#         self.params = params
#         self.h = params['h']
#         self.kh = params['k_h'] # spring constant for the spring connecting hands
#         self.kvp = params['k_vp'] # spring constant for the spring connecting hands
#         self.c = params['c'] # spring constant for the spring connecting object to the hand
#         self.m = params['m'] # mass of the object, mass of the hand
#         self.tau = params['tau'] # time constants of the second order muscle filter
#         self.vp_t = vp_cross_timestep

#         self.setup_model()
#         self.set_costmatrices()
#         self.set_noisevariables()

#     def setup_model(self):
#         """
#         States
#         [0-x, 1-y, 
#         2-vx, 3-vy, 
#         4-fx, 5-gx, 
#         6-fy, 7-gy,
#         8-targ_x, 9-targ_y,
#         10-vp_x, 11-vp_y]
#         """
#         self.A = np.array([[1, 0, self.h, 0, 0, 0, 0, 0], [0, 1, 0, self.h, 0, 0, 0, 0],
#                     [0, 0, (1 - self.h * self.c / self.m), 0, self.h / self.m, 0, 0, 0], 
#                     [0, 0, 0, (1 - self.h * self.c / self.m), 0, 0, self.h / self.m, 0],
#                     [0, 0, 0, 0, (1 -self.h / self.tau), self.h / self.tau, 0, 0], 
#                     [0, 0, 0, 0, 0, (1 - self.h / self.tau), 0, 0],
#                     [0, 0, 0, 0, 0, 0, (1 - self.h / self.tau), self.h / self.tau], 
#                     [0, 0, 0, 0, 0, 0, 0, (1 - self.h / self.tau)]])
        
#         self.A = np.block([[self.A, np.zeros((self.A.shape[0], 4))], 
#                         [np.zeros((4, self.A.shape[0])), np.eye(4)]])
        
#         self.B = np.block([[np.zeros((5, 2))], 
#                         [np.array([[self.h / self.tau, 0], 
#                         [0, 0], 
#                         [0, self.h / self.tau]])], 
#                         [np.zeros((4, 2))]])
        
#         self.state_len = self.A.shape[0]
#         self.control_len = self.B.shape[1]
        
#         return self.A, self.B
    
#     def set_costmatrices(self):
#         self.r = self.params['r'] # weighting factor for the control cost matrix
#         self.wp = self.params['wp'] # weighting factor for position cost
#         self.wv = self.params['wv'] # weighting factor for velocity cost
#         self.wf = self.params['wf'] # weighting factor for force cost
#         self.R = self.r / (self.params['N'] - 1) * np.identity(self.control_len) # Control cost matrix
        
#         self.p_N = np.block([[np.concatenate(([-self.wp], np.zeros(7), [self.wp, 0, 0, 0]))],
#                             [np.concatenate(([0, -self.wp], np.zeros(7), [self.wp, 0, 0]))],
#                            [np.zeros((6, 2)), np.diag([self.wv, self.wv, self.wv, self.wf, self.wf, self.wf]), 
#                            np.zeros((6, 4))]])
        
#         self.Q_N = 1 / 2 * self.p_N.T @ self.p_N # State Cost matrix at last step

#         self.p_t1 = np.block([[np.concatenate(([-self.wp], np.zeros(9), [self.wp, 0]))],
#                             [np.concatenate(([0, -self.wp], np.zeros(9), [self.wp]))]])        
#         self.Q_T1 = 1 / 2 * self.p_t1.T @ self.p_t1
        
#         self.Q = np.zeros_like(self.Q_N)
    
#     def compute_controlfeedbackgains(self):
#         self.L = np.zeros((self.params['N'] + 1, self.B.shape[1], self.B.shape[0])) # sequence of control feedback gains
        
#         # Control feedback gain backward recursion
#         S = copy.copy(self.Q_N)
#         for step in np.arange(self.params['N']-1, -1, -1):
#             self.L[step, :, :] = np.linalg.inv(self.B.transpose() @ S @ self.B + self.R) @ \
#                             self.B.transpose() @ S @ self.A

#             if step > self.params['N'] - self.params['settle_steps']:
#                 S = self.Q_N + self.A.transpose() @ S @ (self.A - self.B @ self.L[step, :, :])
#             elif step == self.vp_t:
#                 S = self.Q_T1 + self.A.transpose() @ S @ (self.A - self.B @ self.L[step, :, :])
#             else:
#                 S = self.Q + self.A.transpose() @ S @ (self.A - self.B @ self.L[step, :, :])
    
#     def set_noisevariables(self):
#         pass

#     def estimate_state(self, x_true_next, x_est_curr, u_curr):        
#         pass

#     # Computing trajectory cost
#     def compute_trajectory_cost(self, x, u):
#         cost = 0
#         for step in range(u.shape[1]):
#             Q = copy.copy(self.Q_T1) if step == round(self.params['N'] / 2) else np.zeros_like(self.Q_N)
                
#             cost += x[:, step].T @ Q @ x[:, step] + \
#                 u[:, step].transpose() @ self.R @ u[:, step]

#         cost += x[:, -1].T @ self.Q_N @ x[:, -1] # terminal state cost

#         return cost


# # hand model. spring between hands only in x-dimension.
# class Hand_Model2:
#     def __init__(self, params, vp_cross_timestep):
#         self.params = params
#         self.h = params['h']
#         self.kh = params['k_h'] # spring constant for the spring connecting hands
#         self.kvp = params['k_vp'] # spring constant for the spring connecting hands
#         self.c = params['c'] # spring constant for the spring connecting object to the hand
#         self.m = params['m'] # mass of the object, mass of the hand
#         self.tau = params['tau'] # time constants of the second order muscle filter
#         self.vp_t = vp_cross_timestep
        
#         self.setup_model()
#         self.set_costmatrices()
#         self.set_noisevariables()

#     def setup_model(self):
#         """
#         States
#         [0-x, 1-y, 
#         2-vx, 3-vy, 
#         4-fx, 5-gx, 
#         6-fy, 7-gy,
#         8-x_other, 9-y_other,
#         10-vx_other, 11-vy_other,
#         12-targ_x, 13-targ_y,
#         14-vp_x, 15-vp_y]
#         """
#         self.A = np.array([[1, 0, self.h, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
#                         [0, 1, 0, self.h, 0, 0, 0, 0, 0, 0, 0, 0],
#                         [-self.h*self.kh/self.m, 0, (1 - self.h * self.c / self.m), 0, self.h / self.m, 0, 0, 0, self.h*self.kh/self.m, 0, 0, 0], 
#                         [0, 0, 0, (1 - self.h * self.c / self.m), 0, 0, self.h / self.m, 0, 0, 0, 0, 0],
#                         [0, 0, 0, 0, (1 -self.h / self.tau), self.h / self.tau, 0, 0, 0, 0, 0, 0], 
#                         [0, 0, 0, 0, 0, (1 - self.h / self.tau), 0, 0, 0, 0, 0, 0],
#                         [0, 0, 0, 0, 0, 0, (1 - self.h / self.tau), self.h / self.tau, 0, 0, 0, 0], 
#                         [0, 0, 0, 0, 0, 0, 0, (1 - self.h / self.tau), 0, 0, 0, 0],
#                         [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, self.h, 0],
#                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, self.h],
#                         [self.h*self.kh/self.m, 0, 0, 0, 0, 0, 0, 0, -self.h*self.kh/self.m, 0, 1 - self.h * self.c / self.m, 0],
#                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 - self.h * self.c / self.m]])
            
#         self.A = np.block([[self.A, np.zeros((self.A.shape[0], 4))], 
#                         [np.zeros((4, self.A.shape[0])), np.eye(4)]])
        
#         self.B = np.block([[np.zeros((5, 2))], 
#                         [np.array([[self.h / self.tau, 0], 
#                         [0, 0], 
#                         [0, self.h / self.tau]])], 
#                         [np.zeros((8, 2))]])
        
#         self.state_len = self.A.shape[0]
#         self.control_len = self.B.shape[1]
        
#         return self.A, self.B
    
#     def set_costmatrices(self):
#         self.r = self.params['r'] # weighting factor for the control cost matrix
#         self.wp = self.params['wp'] # weighting factor for position cost
#         self.wv = self.params['wv'] # weighting factor for velocity cost
#         self.wf = self.params['wf'] # weighting factor for force cost
#         self.R = self.r / (self.params['N'] - 1) * np.identity(self.control_len) # Control cost matrix
        
#         self.p_N = np.block([[np.concatenate(([-self.wp], np.zeros(11), [self.wp, 0, 0, 0]))],
#                             [np.concatenate(([0, -self.wp], np.zeros(11), [self.wp, 0, 0]))],
#                            [np.zeros((10, 2)), np.diag([self.wv, self.wv, self.wv, self.wf, self.wf, self.wf, 0, 0, 0, 0]), 
#                            np.zeros((10, 4))]])
        
#         self.Q_N = 1 / 2 * self.p_N.T @ self.p_N # State Cost matrix at last step

#         self.p_t1 = np.block([[np.concatenate(([-self.wp], np.zeros(13), [self.wp, 0]))],
#                             [np.concatenate(([0, -self.wp], np.zeros(13), [self.wp]))]])        
#         self.Q_T1 = 1 / 2 * self.p_t1.T @ self.p_t1
        
#         self.Q = np.zeros_like(self.Q_N)
    
#     def compute_controlfeedbackgains(self):
#         self.L = np.zeros((self.params['N'] + 1, self.B.shape[1], self.B.shape[0])) # sequence of control feedback gains
        
#         # Control feedback gain backward recursion
#         S = copy.copy(self.Q_N)
#         for step in np.arange(self.params['N']-1, -1, -1):
#             self.L[step, :, :] = np.linalg.inv(self.B.transpose() @ S @ self.B + self.R) @ \
#                             self.B.transpose() @ S @ self.A

#             if step > self.params['N'] - self.params['settle_steps']:
#                 S = self.Q_N + self.A.transpose() @ S @ (self.A - self.B @ self.L[step, :, :])
#             elif step == self.vp_t:
#                 S = self.Q_T1 + self.A.transpose() @ S @ (self.A - self.B @ self.L[step, :, :])
#             else:
#                 S = self.Q + self.A.transpose() @ S @ (self.A - self.B @ self.L[step, :, :])
    
#     def set_noisevariables(self):
#         pass

#     def estimate_state(self, x_true_next, x_est_curr, u_curr):        
#         pass

#     # Computing trajectory cost
#     def compute_trajectory_cost(self, x, u):
#         cost = 0
#         for step in range(u.shape[1]):
#             Q = copy.copy(self.Q_T1) if step == round(self.params['N'] / 2) else np.zeros_like(self.Q_N)
                
#             cost += x[:, step].T @ Q @ x[:, step] + \
#                 u[:, step].transpose() @ self.R @ u[:, step]

#         cost += x[:, -1].T @ self.Q_N @ x[:, -1] # terminal state cost

#         return cost


# # hand model. spring between hands only in x-dimension. Cost term to minimize distance from other's hand.
# class Hand_Model3:
#     def __init__(self, params, vp_cross_timestep):
#         self.params = params
#         self.h = params['h']
#         self.kh = params['k_h'] # spring constant for the spring connecting hands
#         self.kvp = params['k_vp'] # spring constant for the spring connecting hands
#         self.c = params['c'] # spring constant for the spring connecting object to the hand
#         self.m = params['m'] # mass of the object, mass of the hand
#         self.tau = params['tau'] # time constants of the second order muscle filter
#         self.vp_t = vp_cross_timestep
        
#         self.setup_model()
#         self.set_costmatrices()
#         self.set_noisevariables()

#     def setup_model(self):
#         """
#         States
#         [0-x, 1-y, 
#         2-vx, 3-vy, 
#         4-fx, 5-gx, 
#         6-fy, 7-gy,
#         8-x_other, 9-y_other,
#         10-vx_other, 11-vy_other,
#         12-targ_x, 13-targ_y,
#         14-vp_x, 15-vp_y]
#         """
#         self.A = np.array([[1, 0, self.h, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
#                         [0, 1, 0, self.h, 0, 0, 0, 0, 0, 0, 0, 0],
#                         [-self.h*self.kh/self.m, 0, (1 - self.h * self.c / self.m), 0, self.h / self.m, 0, 0, 0, self.h*self.kh/self.m, 0, 0, 0], 
#                         [0, 0, 0, (1 - self.h * self.c / self.m), 0, 0, self.h / self.m, 0, 0, 0, 0, 0],
#                         [0, 0, 0, 0, (1 -self.h / self.tau), self.h / self.tau, 0, 0, 0, 0, 0, 0], 
#                         [0, 0, 0, 0, 0, (1 - self.h / self.tau), 0, 0, 0, 0, 0, 0],
#                         [0, 0, 0, 0, 0, 0, (1 - self.h / self.tau), self.h / self.tau, 0, 0, 0, 0], 
#                         [0, 0, 0, 0, 0, 0, 0, (1 - self.h / self.tau), 0, 0, 0, 0],
#                         [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, self.h, 0],
#                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, self.h],
#                         [self.h*self.kh/self.m, 0, 0, 0, 0, 0, 0, 0, -self.h*self.kh/self.m, 0, 1 - self.h * self.c / self.m, 0],
#                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 - self.h * self.c / self.m]])
            
#         self.A = np.block([[self.A, np.zeros((self.A.shape[0], 4))], 
#                         [np.zeros((4, self.A.shape[0])), np.eye(4)]])
        
#         self.B = np.block([[np.zeros((5, 2))], 
#                         [np.array([[self.h / self.tau, 0], 
#                         [0, 0], 
#                         [0, self.h / self.tau]])], 
#                         [np.zeros((8, 2))]])
        
#         self.state_len = self.A.shape[0]
#         self.control_len = self.B.shape[1]
        
#         return self.A, self.B
    
#     def set_costmatrices(self):
#         self.r = self.params['r'] # weighting factor for the control cost matrix
#         self.wp = self.params['wp'] # weighting factor for position cost
#         self.wv = self.params['wv'] # weighting factor for velocity cost
#         self.wf = self.params['wf'] # weighting factor for force cost
#         self.R = self.r / (self.params['N'] - 1) * np.identity(self.control_len) # Control cost matrix
        
#         self.p_N = np.block([[np.concatenate(([-self.wp], np.zeros(11), [self.wp, 0, 0, 0]))],
#                             [np.concatenate(([0, -self.wp], np.zeros(11), [self.wp, 0, 0]))],
#                            [np.zeros((10, 2)), np.diag([self.wv, self.wv, self.wv, self.wf, self.wf, self.wf, 0, 0, 0, 0]), 
#                            np.zeros((10, 4))]])
        
#         self.Q_N = 1 / 2 * self.p_N.T @ self.p_N # State Cost matrix at last step

#         # minimizing distance from the viapoint
#         self.p_t1 = np.block([[np.concatenate(([-self.wp], np.zeros(13), [self.wp, 0]))],
#                             [np.concatenate(([0, -self.wp], np.zeros(13), [self.wp]))]])        
#         self.Q_T1 = 1 / 200 * self.p_t1.T @ self.p_t1
        
#         # minimizing distance from the others hand positions
#         self.p_t = np.block([[np.concatenate(([-self.wp], np.zeros(7), [self.wp, 0, 0, 0, 0, 0, 0, 0]))],
#                             [np.concatenate(([0, -self.wp], np.zeros(7), [self.wp, 0, 0, 0, 0, 0, 0]))]])
#         self.Q = 1 / 200000 * self.p_t.T @ self.p_t
    
#     def compute_controlfeedbackgains(self):
#         self.L = np.zeros((self.params['N'] + 1, self.B.shape[1], self.B.shape[0])) # sequence of control feedback gains
        
#         # Control feedback gain backward recursion
#         S = copy.copy(self.Q_N)
#         for step in np.arange(self.params['N']-1, -1, -1):
#             self.L[step, :, :] = np.linalg.inv(self.B.transpose() @ S @ self.B + self.R) @ \
#                             self.B.transpose() @ S @ self.A

#             if step > self.params['N'] - self.params['settle_steps']:
#                 S = self.Q + self.Q_N + self.A.transpose() @ S @ (self.A - self.B @ self.L[step, :, :])
#             elif step == self.vp_t:
#                 S = self.Q + self.Q_T1 + self.A.transpose() @ S @ (self.A - self.B @ self.L[step, :, :])
#             else:
#                 S = self.Q + self.A.transpose() @ S @ (self.A - self.B @ self.L[step, :, :])
    
#     def set_noisevariables(self):
#         pass

#     def estimate_state(self, x_true_next, x_est_curr, u_curr):        
#         pass

#     # Computing trajectory cost
#     def compute_trajectory_cost(self, x, u):
#         cost = 0
#         for step in range(u.shape[1]):
#             Q = copy.copy(self.Q_T1) if step == round(self.params['N'] / 2) else np.zeros_like(self.Q_N)
                
#             cost += x[:, step].T @ Q @ x[:, step] + \
#                 u[:, step].transpose() @ self.R @ u[:, step]

#         cost += x[:, -1].T @ self.Q_N @ x[:, -1] # terminal state cost

#         return cost


# # hand model. Spring between hands only in x-dimension. Spring to via-point only in x dimension.
# class Hand_Model4:
#     def __init__(self, params, vp_cross_timestep):
#         self.params = params
#         self.h = params['h']
#         self.kh = params['k_h'] # spring constant for the spring connecting hands
#         self.kvp = params['k_vp'] # spring constant for the spring connecting hands
#         self.c = params['c'] # spring constant for the spring connecting object to the hand
#         self.m = params['m'] # mass of the object, mass of the hand
#         self.tau = params['tau'] # time constants of the second order muscle filter
#         self.vp_t = vp_cross_timestep
        
#         self.setup_model()
#         self.set_costmatrices()
#         self.set_noisevariables()

#     def setup_model(self):
#         """
#         States
#         [0-x, 1-y, 
#         2-vx, 3-vy, 
#         4-fx, 5-gx, 
#         6-fy, 7-gy,
#         8-x_other, 9-y_other,
#         10-vx_other, 11-vy_other,
#         12-targ_x, 13-targ_y,
#         14-vp_x, 15-vp_y]
#         """
#         self.A = np.array([[1, 0, self.h, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
#                         [0, 1, 0, self.h, 0, 0, 0, 0, 0, 0, 0, 0],
#                         [-self.h*self.kh/self.m - self.h*self.kvp/self.m, 0, (1 - self.h * self.c / self.m), 0, self.h / self.m, 0, 0, 0, self.h*self.kh/self.m, 0, 0, 0], 
#                         [0, 0, 0, (1 - self.h * self.c / self.m), 0, 0, self.h / self.m, 0, 0, 0, 0, 0],
#                         [0, 0, 0, 0, (1 -self.h / self.tau), self.h / self.tau, 0, 0, 0, 0, 0, 0], 
#                         [0, 0, 0, 0, 0, (1 - self.h / self.tau), 0, 0, 0, 0, 0, 0],
#                         [0, 0, 0, 0, 0, 0, (1 - self.h / self.tau), self.h / self.tau, 0, 0, 0, 0], 
#                         [0, 0, 0, 0, 0, 0, 0, (1 - self.h / self.tau), 0, 0, 0, 0],
#                         [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, self.h, 0],
#                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, self.h],
#                         [self.h*self.kh/self.m, 0, 0, 0, 0, 0, 0, 0, -self.h*self.kh/self.m, 0, 1 - self.h * self.c / self.m, 0],
#                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 - self.h * self.c / self.m]])
            
#         self.A = np.block([[self.A, np.zeros((self.A.shape[0], 4))], 
#                         [np.zeros((4, self.A.shape[0])), np.eye(4)]])
#         self.A[2, 14] = self.h * self.kvp / self.m # x interaction force term from the spring to via-point
        
#         self.B = np.block([[np.zeros((5, 2))], 
#                         [np.array([[self.h / self.tau, 0], 
#                         [0, 0], 
#                         [0, self.h / self.tau]])], 
#                         [np.zeros((8, 2))]])
        
#         self.state_len = self.A.shape[0]
#         self.control_len = self.B.shape[1]
        
#         return self.A, self.B
    
#     def set_costmatrices(self):
#         self.r = self.params['r'] # weighting factor for the control cost matrix
#         self.wp = self.params['wp'] # weighting factor for position cost
#         self.wv = self.params['wv'] # weighting factor for velocity cost
#         self.wf = self.params['wf'] # weighting factor for force cost
#         self.R = self.r / (self.params['N'] - 1) * np.identity(self.control_len) # Control cost matrix
        
#         self.p_N = np.block([[np.concatenate(([-self.wp], np.zeros(11), [self.wp, 0, 0, 0]))],
#                             [np.concatenate(([0, -self.wp], np.zeros(11), [self.wp, 0, 0]))],
#                            [np.zeros((10, 2)), np.diag([self.wv, self.wv, self.wv, self.wf, self.wf, self.wf, 0, 0, 0, 0]), 
#                            np.zeros((10, 4))]])
        
#         self.Q_N = 1 / 2 * self.p_N.T @ self.p_N # State Cost matrix at last step

#         self.p_t1 = np.block([[np.concatenate(([-self.wp], np.zeros(13), [self.wp, 0]))],
#                             [np.concatenate(([0, -self.wp], np.zeros(13), [self.wp]))]])        
#         self.Q_T1 = 1 / 2 * self.p_t1.T @ self.p_t1
        
#         self.Q = np.zeros_like(self.Q_N)
    
#     def compute_controlfeedbackgains(self):
#         self.L = np.zeros((self.params['N'] + 1, self.B.shape[1], self.B.shape[0])) # sequence of control feedback gains
        
#         # Control feedback gain backward recursion
#         S = copy.copy(self.Q_N)
#         for step in np.arange(self.params['N']-1, -1, -1):
#             self.L[step, :, :] = np.linalg.inv(self.B.transpose() @ S @ self.B + self.R) @ \
#                             self.B.transpose() @ S @ self.A

#             if step > self.params['N'] - self.params['settle_steps']:
#                 S = self.Q_N + self.A.transpose() @ S @ (self.A - self.B @ self.L[step, :, :])
#             elif step == self.vp_t:
#                 S = self.Q_T1 + self.A.transpose() @ S @ (self.A - self.B @ self.L[step, :, :])
#             else:
#                 S = self.Q + self.A.transpose() @ S @ (self.A - self.B @ self.L[step, :, :])
    
#     def set_noisevariables(self):
#         pass

#     def estimate_state(self, x_true_next, x_est_curr, u_curr):        
#         pass

#     # Computing trajectory cost
#     def compute_trajectory_cost(self, x, u):
#         cost = 0
#         for step in range(u.shape[1]):
#             Q = copy.copy(self.Q_T1) if step == round(self.params['N'] / 2) else np.zeros_like(self.Q_N)
                
#             cost += x[:, step].T @ Q @ x[:, step] + \
#                 u[:, step].transpose() @ self.R @ u[:, step]

#         cost += x[:, -1].T @ self.Q_N @ x[:, -1] # terminal state cost

#         return cost


# # hand model. Spring between hands only in x-dimension. Spring to self via-point only in x dimension.
# class Hand_Model5:
#     def __init__(self, params, vp_cross_timestep):
#         self.params = params            
#         self.h = params['h']
#         self.kh = params['k_h'] # spring constant for the spring connecting hands
#         self.kvp = params['k_vp'] # spring constant for the spring connecting hands
#         self.c = params['c'] # spring constant for the spring connecting object to the hand
#         self.m = params['m'] # mass of the object, mass of the hand
#         self.tau = params['tau'] # time constants of the second order muscle filter
#         self.vp_t = vp_cross_timestep
        
#         self.setup_model()
#         self.set_costmatrices()
#         self.set_noisevariables()

#     def setup_model(self):
#         """
#         States
#         [0-x, 1-y, 
#         2-vx, 3-vy, 
#         4-fx, 5-gx, 
#         6-fy, 7-gy,
#         8-x_other, 9-y_other,
#         10-vx_other, 11-vy_other,
#         12-targ_x, 13-targ_y,
#         14-vp_x, 15-vp_y]
#         """
#         self.A = np.array([[1, 0, self.h, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
#                         [0, 1, 0, self.h, 0, 0, 0, 0, 0, 0, 0, 0],
#                         [-self.h*self.kh/self.m - self.h*self.kvp/self.m, 0, (1 - self.h * self.c / self.m), 0, self.h / self.m, 0, 0, 0, self.h*self.kh/self.m, 0, 0, 0], 
#                         [0, 0, 0, (1 - self.h * self.c / self.m), 0, 0, self.h / self.m, 0, 0, 0, 0, 0],
#                         [0, 0, 0, 0, (1 -self.h / self.tau), self.h / self.tau, 0, 0, 0, 0, 0, 0], 
#                         [0, 0, 0, 0, 0, (1 - self.h / self.tau), 0, 0, 0, 0, 0, 0],
#                         [0, 0, 0, 0, 0, 0, (1 - self.h / self.tau), self.h / self.tau, 0, 0, 0, 0], 
#                         [0, 0, 0, 0, 0, 0, 0, (1 - self.h / self.tau), 0, 0, 0, 0],
#                         [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, self.h, 0],
#                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, self.h],
#                         [self.h*self.kh/self.m, 0, 0, 0, 0, 0, 0, 0, -self.h*self.kh/self.m, 0, 1 - self.h * self.c / self.m, 0],
#                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 - self.h * self.c / self.m]])
            
#         self.A = np.block([[self.A, np.zeros((self.A.shape[0], 4))], 
#                         [np.zeros((4, self.A.shape[0])), np.eye(4)]])
#         self.A[2, 14] = self.h * self.kvp / self.m # x interaction force term from the spring to via-point
        
#         self.B = np.block([[np.zeros((5, 2))], 
#                         [np.array([[self.h / self.tau, 0], 
#                         [0, 0], 
#                         [0, self.h / self.tau]])], 
#                         [np.zeros((8, 2))]])
        
#         self.state_len = self.A.shape[0]
#         self.control_len = self.B.shape[1]
        
#         return self.A, self.B
    
#     def set_costmatrices(self):
#         self.r = self.params['r'] # weighting factor for the control cost matrix
#         self.wp = self.params['wp'] # weighting factor for position cost
#         self.wv = self.params['wv'] # weighting factor for velocity cost
#         self.wf = self.params['wf'] # weighting factor for force cost
#         self.R = self.r / (self.params['N'] - 1) * np.identity(self.control_len) # Control cost matrix
        
#         self.p_N = np.block([[np.concatenate(([-self.wp], np.zeros(11), [self.wp, 0, 0, 0]))],
#                             [np.concatenate(([0, -self.wp], np.zeros(11), [self.wp, 0, 0]))],
#                            [np.zeros((10, 2)), np.diag([self.wv, self.wv, self.wv, self.wf, self.wf, self.wf, 0, 0, 0, 0]), 
#                            np.zeros((10, 4))]])
        
#         self.Q_N = 1 / 2 * self.p_N.T @ self.p_N # State Cost matrix at last step

#         self.p_t1 = np.block([[np.concatenate(([-self.wp], np.zeros(13), [self.wp, 0]))],
#                             [np.concatenate(([0, -self.wp], np.zeros(13), [self.wp]))]])        
#         self.Q_T1 = 1 / 2 * self.p_t1.T @ self.p_t1
        
#         self.Q = np.zeros_like(self.Q_N)
    
#     def compute_controlfeedbackgains(self):
#         self.L = np.zeros((self.params['N'] + 1, self.B.shape[1], self.B.shape[0])) # sequence of control feedback gains
        
#         # Control feedback gain backward recursion
#         S = copy.copy(self.Q_N)
#         for step in np.arange(self.params['N']-1, -1, -1):
#             self.L[step, :, :] = np.linalg.inv(self.B.transpose() @ S @ self.B + self.R) @ \
#                             self.B.transpose() @ S @ self.A

#             if step > self.params['N'] - self.params['settle_steps']:
#                 S = self.Q_N + self.A.transpose() @ S @ (self.A - self.B @ self.L[step, :, :])
#             elif step == self.vp_t:
#                 S = self.Q_T1 + self.A.transpose() @ S @ (self.A - self.B @ self.L[step, :, :])
#             else:
#                 S = self.Q + self.A.transpose() @ S @ (self.A - self.B @ self.L[step, :, :])
    
#     def set_noisevariables(self):
#         pass

#     def estimate_state(self, x_true_next, x_est_curr, u_curr):        
#         pass

#     # Computing trajectory cost
#     def compute_trajectory_cost(self, x, u):
#         cost = 0
#         for step in range(u.shape[1]):
#             Q = copy.copy(self.Q_T1) if step == round(self.params['N'] / 2) else np.zeros_like(self.Q_N)
                
#             cost += x[:, step].T @ Q @ x[:, step] + \
#                 u[:, step].transpose() @ self.R @ u[:, step]

#         cost += x[:, -1].T @ self.Q_N @ x[:, -1] # terminal state cost

#         return cost


# # hand model. Spring between hands only in x-dimension. Spring to both via-points only in x dimension.
# class Hand_Model6:
#     def __init__(self, params, vp_cross_timestep):
#         self.params = params
#         self.h = params['h']
#         self.kh = params['k_h'] # spring constant for the spring connecting hands
#         self.kvp = params['k_vp'] # spring constant for the spring connecting hands
#         self.c = params['c'] # spring constant for the spring connecting object to the hand
#         self.m = params['m'] # mass of the object, mass of the hand
#         self.tau = params['tau'] # time constants of the second order muscle filter
#         self.vp_t = vp_cross_timestep
        
#         self.setup_model()
#         self.set_costmatrices()
#         self.set_noisevariables()

#     def setup_model(self):
#         """
#         States
#         [0-x, 1-y, 
#         2-vx, 3-vy, 
#         4-fx, 5-gx, 
#         6-fy, 7-gy,
#         8-x_other, 9-y_other,
#         10-vx_other, 11-vy_other,
#         12-targ_x, 13-targ_y,
#         14-vp_x, 15-vp_y,
#         16-vpother_x, 17-vpother_y]]
#         """
#         self.A = np.array([[1, 0, self.h, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
#                         [0, 1, 0, self.h, 0, 0, 0, 0, 0, 0, 0, 0],
#                         [-self.h*self.kh/self.m - 2 * self.h*self.kvp/self.m, 0, (1 - self.h * self.c / self.m), 0, self.h / self.m, 0, 0, 0, self.h*self.kh/self.m, 0, 0, 0], 
#                         [0, 0, 0, (1 - self.h * self.c / self.m), 0, 0, self.h / self.m, 0, 0, 0, 0, 0],
#                         [0, 0, 0, 0, (1 -self.h / self.tau), self.h / self.tau, 0, 0, 0, 0, 0, 0], 
#                         [0, 0, 0, 0, 0, (1 - self.h / self.tau), 0, 0, 0, 0, 0, 0],
#                         [0, 0, 0, 0, 0, 0, (1 - self.h / self.tau), self.h / self.tau, 0, 0, 0, 0], 
#                         [0, 0, 0, 0, 0, 0, 0, (1 - self.h / self.tau), 0, 0, 0, 0],
#                         [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, self.h, 0],
#                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, self.h],
#                         [self.h*self.kh/self.m, 0, 0, 0, 0, 0, 0, 0, -self.h*self.kh/self.m, 0, 1 - self.h * self.c / self.m, 0],
#                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 - self.h * self.c / self.m]])
            
#         self.A = np.block([[self.A, np.zeros((self.A.shape[0], 6))], 
#                         [np.zeros((6, self.A.shape[0])), np.eye(6)]])
#         self.A[2, 14] = self.h * self.kvp / self.m # x interaction force term from the spring to via-point
#         self.A[2, 16] = self.h * self.kvp / self.m # x interaction force term from the spring to via-point

#         self.B = np.block([[np.zeros((5, 2))], 
#                         [np.array([[self.h / self.tau, 0], 
#                         [0, 0], 
#                         [0, self.h / self.tau]])], 
#                         [np.zeros((10, 2))]])
        
#         self.state_len = self.A.shape[0]
#         self.control_len = self.B.shape[1]
        
#         return self.A, self.B
    
#     def set_costmatrices(self):
#         self.r = self.params['r'] # weighting factor for the control cost matrix
#         self.wp = self.params['wp'] # weighting factor for position cost
#         self.wv = self.params['wv'] # weighting factor for velocity cost
#         self.wf = self.params['wf'] # weighting factor for force cost
#         self.R = self.r / (self.params['N'] - 1) * np.identity(self.control_len) # Control cost matrix
        
#         self.p_N = np.block([[np.concatenate(([-self.wp], np.zeros(11), [self.wp, 0, 0, 0, 0, 0]))],
#                             [np.concatenate(([0, -self.wp], np.zeros(11), [self.wp, 0, 0, 0, 0]))],
#                            [np.zeros((12, 2)), np.diag([self.wv, self.wv, self.wv, self.wf, self.wf, self.wf, 0, 0, 0, 0, 0, 0]), 
#                            np.zeros((12, 4))]])
        
#         self.Q_N = 1 / 2 * self.p_N.T @ self.p_N # State Cost matrix at last step

#         self.p_t1 = np.block([[np.concatenate(([-self.wp], np.zeros(13), [self.wp, 0, 0, 0]))],
#                             [np.concatenate(([0, -self.wp], np.zeros(13), [self.wp, 0, 0]))]])        
#         self.Q_T1 = 1 / 2 * self.p_t1.T @ self.p_t1
        
#         self.Q = np.zeros_like(self.Q_N)
    
#     def compute_controlfeedbackgains(self):
#         self.L = np.zeros((self.params['N'] + 1, self.B.shape[1], self.B.shape[0])) # sequence of control feedback gains
        
#         # Control feedback gain backward recursion
#         S = copy.copy(self.Q_N)
#         for step in np.arange(self.params['N']-1, -1, -1):
#             self.L[step, :, :] = np.linalg.inv(self.B.transpose() @ S @ self.B + self.R) @ \
#                             self.B.transpose() @ S @ self.A

#             if step > self.params['N'] - self.params['settle_steps']:
#                 S = self.Q_N + self.A.transpose() @ S @ (self.A - self.B @ self.L[step, :, :])
#             elif step == self.vp_t:
#                 S = self.Q_T1 + self.A.transpose() @ S @ (self.A - self.B @ self.L[step, :, :])
#             else:
#                 S = self.Q + self.A.transpose() @ S @ (self.A - self.B @ self.L[step, :, :])
    
#     def set_noisevariables(self):
#         pass

#     def estimate_state(self, x_true_next, x_est_curr, u_curr):        
#         pass

#     # Computing trajectory cost
#     def compute_trajectory_cost(self, x, u):
#         cost = 0
#         for step in range(u.shape[1]):
#             Q = copy.copy(self.Q_T1) if step == round(self.params['N'] / 2) else np.zeros_like(self.Q_N)
                
#             cost += x[:, step].T @ Q @ x[:, step] + \
#                 u[:, step].transpose() @ self.R @ u[:, step]

#         cost += x[:, -1].T @ self.Q_N @ x[:, -1] # terminal state cost

#         return cost


# single LQG controlling both hands.
class TwoHands_Model:
    def __init__(self, params, vp_cross_timestep):
        self.params = params
        self.h = params['h']
        self.kh = params['k_h'] # spring constant for the spring connecting hands
        self.kvp = params['k_vp'] # spring constant for the spring connecting to self via point
        self.kvp_ = params['k_vp_'] # spring constant for the spring connecting to others via point
        self.c = params['c'] # damping constant capturing viscous properties of the muscles and joints
        self.m = params['m'] # mass of the hand
        self.tau = params['tau'] # time constants of the second order muscle filter
        self.vp_t = vp_cross_timestep
        
        self.setup_model()
        self.set_costmatrices()
        self.set_noisevariables()

    def setup_model(self):
        """
        Hand States
        [0-x1, 1-y1, 
        2-vx1, 3-vy1, 
        4-fx1, 5-gx1, 
        6-fy1, 7-gy1,
        8-x1, 9-y1, 
        10-vx1, 11-vy1, 
        12-fx1, 13-gx1, 
        14-fy1, 15-gy1,
        16-targ_x, 17-targ_y,
        18-vp1_x, 19-vp1_y,
        20-vp2_x, 21-vp2_y
        """
        self.Ah = np.array([[1, 0, self.h, 0, 0, 0, 0, 0], 
                        [0, 1, 0, self.h, 0, 0, 0, 0],
                        [-self.h * self.kh / self.m - self.h * self.kvp / self.m - self.h * self.kvp_ / self.m, 0, (1 - self.h * self.c / self.m), 0, self.h / self.m, 0, 0, 0], 
                        [0, 0, 0, (1 - self.h * self.c / self.m), 0, 0, self.h / self.m, 0],
                        [0, 0, 0, 0, (1 -self.h / self.tau), self.h / self.tau, 0, 0], 
                        [0, 0, 0, 0, 0, (1 - self.h / self.tau), 0, 0],
                        [0, 0, 0, 0, 0, 0, (1 - self.h / self.tau), self.h / self.tau], 
                        [0, 0, 0, 0, 0, 0, 0, (1 - self.h / self.tau)]])
        
        # Interaction force elements from other hand
        self.Ai = np.zeros_like(self.Ah)
        self.Ai[2, 0] = self.h * self.kh / self.m

        self.A = np.block([[self.Ah, self.Ai, np.zeros((self.Ah.shape[0], 6))],
                           [self.Ai, self.Ah, np.zeros((self.Ah.shape[0], 6))],
                           [np.zeros((6, 2 * self.Ah.shape[0])), np.eye(6)]])
        
        # Interaction force elements from via points
        self.A[2, 18] = self.h * self.kvp / self.m
        self.A[10, 20] = self.h * self.kvp / self.m
        self.A[2, 20] = self.h * self.kvp_ / self.m
        self.A[10, 18] = self.h * self.kvp_ / self.m
        
        self.Bh = np.block([[np.zeros((5, 2))], 
                            [np.array([[self.h / self.tau, 0], 
                            [0, 0], 
                            [0, self.h / self.tau]])]])
        self.B = np.block([[self.Bh, np.zeros((self.Bh.shape[0], 2))],
                           [np.zeros((self.Bh.shape[0], 2)), self.Bh],
                           [np.zeros((6, 4))]])
        
        self.state_len = self.A.shape[0]
        self.control_len = self.B.shape[1]

        self.C = np.eye(self.A.shape[0])
        
        return self.A, self.B
    
    def set_costmatrices(self):
        self.r = self.params['r'] # weighting factor for the control cost matrix
        self.wp = self.params['wp'] # weighting factor for position cost
        self.wv = self.params['wv'] # weighting factor for velocity cost
        self.wf = self.params['wf'] # weighting factor for force cost
        self.R = self.r / (self.params['N'] - 1) * np.identity(self.control_len) # Control cost matrix
        
        # cost matrix for p1 passing throug end target
        self.p_N1 = np.block([[np.concatenate(([-self.wp], np.zeros(15), [self.wp, 0, 0, 0, 0, 0]))],
                                [np.concatenate(([0, -self.wp], np.zeros(15), [self.wp, 0, 0, 0, 0]))],
                                [np.zeros((6, 2)), np.diag([self.wv, self.wv, self.wv, self.wf, self.wf, self.wf]),
                                np.zeros((6, 14))]])
        
        # cost matrix for p2 passing throug end target
        self.p_N2 = np.block([[np.concatenate((np.zeros(8), [-self.wp], np.zeros(7), [self.wp, 0, 0, 0, 0, 0]))],
                                [np.concatenate((np.zeros(9), [-self.wp], np.zeros(7), [self.wp, 0, 0, 0, 0]))],
                                [np.zeros((6, 10)), np.diag([self.wv, self.wv, self.wv, self.wf, self.wf, self.wf]), 
                                np.zeros((6, 6))]])
        
        self.Q_N = 1 / 2 * self.p_N1.T @ self.p_N1 + 1 / 2 * self.p_N2.T @ self.p_N2  # State Cost matrix at last step

        # cost matrix for p1 passing throug vp1
        self.p_t11 = np.block([[np.concatenate(([-self.wp], np.zeros(17), [self.wp, 0, 0, 0]))],
                                [np.concatenate(([0, -self.wp], np.zeros(17), [self.wp, 0, 0]))]])        
        self.Q_T11 = 1 / 2 * self.p_t11.T @ self.p_t11

        # cost matrix for p1 passing throug vp2
        self.p_t12 = np.block([[np.concatenate(([-self.wp], np.zeros(19), [self.wp, 0]))],
                                [np.concatenate(([0, -self.wp], np.zeros(19), [self.wp]))]])        
        self.Q_T12 = 1 / 2 * self.p_t12.T @ self.p_t12

        # cost matrix for p2 passing throug vp2
        self.p_t22 = np.block([[np.concatenate((np.zeros(8), [-self.wp], np.zeros(11), [self.wp, 0]))],
                                [np.concatenate((np.zeros(9), [-self.wp], np.zeros(11), [self.wp]))]])        
        self.Q_T22 = 1 / 2 * self.p_t22.T @ self.p_t22

        # cost matrix for p2 passing throug vp1
        self.p_t21 = np.block([[np.concatenate((np.zeros(8), [-self.wp], np.zeros(9), [self.wp, 0, 0, 0]))],
                                [np.concatenate((np.zeros(9), [-self.wp], np.zeros(9), [self.wp, 0, 0]))]])        
        self.Q_T21 = 1 / 2 * self.p_t21.T @ self.p_t21
        
        # cost matrix for general time 't'
        self.Q = np.zeros_like(self.Q_N)
    
    def compute_controlfeedbackgains(self):
        self.L = np.zeros((self.params['N'] + 1, self.B.shape[1], self.B.shape[0])) # sequence of control feedback gains
        
        # Control feedback gain backward recursion
        S = self.Q_N
        for step in np.arange(self.params['N']-1, -1, -1):
            self.L[step, :, :] = np.linalg.inv(self.B.T @ S @ self.B + self.R) @ \
                                    self.B.T @ S @ self.A

            if step == self.params['N'] - 1:
                S = self.Q_N + self.A.T @ S @ (self.A - self.B @ self.L[step, :, :])
            elif step == self.vp_t[0]:
                S = self.Q_T11 + self.A.T @ S @ (self.A - self.B @ self.L[step, :, :])
            elif step == self.vp_t[1]:
                S = self.Q_T22 + self.A.T @ S @ (self.A - self.B @ self.L[step, :, :])
            else:
                S = self.Q + self.A.T @ S @ (self.A - self.B @ self.L[step, :, :])
    
    def set_noisevariables(self):
        self.sig_self_scale = self.params['sig self noise']
        
        s_p = self.params['sig p']
        s_v = self.params['sig v']
        s_f = self.params['sig f']
        
        self.noise_s = self.sig_self_scale * np.array([s_p, s_p, s_v, s_v, s_f, s_f, s_f, s_f, 
                                                    s_p, s_p, s_v, s_v, s_f, s_f, s_f, s_f,
                                                    s_p / 10, s_p / 10, s_p / 10, s_p / 10, s_p / 10, s_p / 10])

        self.Ws = np.diag(self.noise_s ** 2) # sensory feedback additive noise covariance matrix
        
        self.E = 0.01 * np.eye(self.A.shape[0]) # state covariance matrix initial for estimation

        f_cov = self.params['force cov']
        v_cov = self.params['vel cov']
        p_cov = self.params['pos cov']
        self.noise_c = np.array([0, 0, 0, 0, 0, f_cov, 0, f_cov,
                                 0, 0, 0, 0, 0, f_cov, 0, f_cov,
                                 0, 0, 0, 0, 0, 0])
        self.Wc = np.diag(self.noise_c ** 2)

    def estimate_state(self, x_true_next, x_est_curr, u_curr):
        y = self.C @ x_true_next + np.random.normal(0, np.sqrt(np.diag(self.Ws)))
            
        # prediction steps
        x_pred = self.A @ x_est_curr + self.B @ u_curr
        E_pred = self.A @ self.E @ self.A.T + self.Wc
        
        # update steps
        Ko = E_pred @ self.C.T @ np.linalg.inv(self.C @ E_pred @ self.C.T + self.Ws)

        x_est_next = x_pred + Ko @ (y - self.C @ x_pred)
        self.E = (np.eye(self.C.shape[1]) - Ko @ self.C) @ E_pred
        
        return x_est_next   
    
    # Computing trajectory cost
    def compute_trajectory_cost(self, x, u):
        cost = 0
        for step in range(u.shape[1]):
            if step == self.vp_t[0]:
                cost += x[:, step].T @ (self.Q_T11 + self.Q) @ x[:, step] + u[:, step].transpose() @ self.R @ u[:, step]
            elif step == self.vp_t[1]:
                cost += x[:, step].T @ (self.Q_T22 + self.Q) @ x[:, step] + u[:, step].transpose() @ self.R @ u[:, step]
            else:
                cost += x[:, step].T @ self.Q @ x[:, step] + u[:, step].transpose() @ self.R @ u[:, step]
                
        cost += x[:, step].T @ self.Q_N @ x[:, step] 

        return cost


# two LQG's but each controller calculates feedback gains assuming the other controller is inactive. 
class SingleHandControl_FullModel:
    def __init__(self, params, hand_id, vp_cross_timestep):
        self.params = params
        self.h = params['h']
        self.kh = params['k_h'] # spring constant for the spring connecting hands
        self.kvp = params['k_vp'] # spring constant for the spring connecting to self via point
        self.kvp_ = params['k_vp_'] # spring constant for the spring connecting to others via point
        self.c = params['c'] # damping constant capturing viscous properties of the muscles and joints
        self.m = params['m'] # mass of the hand
        self.tau = params['tau'] # time constants of the second order muscle filter
        self.vp_t = vp_cross_timestep
        self.hand_id = hand_id
        
        self.setup_model()
        self.set_costmatrices()
        self.set_noisevariables()

    def setup_model(self):
        """
        Hand States
        [0-x1, 1-y1, 
        2-vx1, 3-vy1, 
        4-fx1, 5-gx1, 
        6-fy1, 7-gy1,
        8-x1, 9-y1, 
        10-vx1, 11-vy1, 
        12-fx1, 13-gx1, 
        14-fy1, 15-gy1,
        16-targ_x, 17-targ_y,
        18-vp1_x, 19-vp1_y,
        20-vp2_x, 21-vp2_y
        """
        self.Ah = np.array([[1, 0, self.h, 0, 0, 0, 0, 0], 
                        [0, 1, 0, self.h, 0, 0, 0, 0],
                        [-self.h * self.kh / self.m - self.h * self.kvp / self.m - self.h * self.kvp_ / self.m, 0, (1 - self.h * self.c / self.m), 0, self.h / self.m, 0, 0, 0], 
                        [0, 0, 0, (1 - self.h * self.c / self.m), 0, 0, self.h / self.m, 0],
                        [0, 0, 0, 0, (1 -self.h / self.tau), self.h / self.tau, 0, 0], 
                        [0, 0, 0, 0, 0, (1 - self.h / self.tau), 0, 0],
                        [0, 0, 0, 0, 0, 0, (1 - self.h / self.tau), self.h / self.tau], 
                        [0, 0, 0, 0, 0, 0, 0, (1 - self.h / self.tau)]])
        
        # Interaction force elements from other hand
        self.Ai = np.zeros_like(self.Ah)
        self.Ai[2, 0] = self.h * self.kh / self.m

        self.A = np.block([[self.Ah, self.Ai, np.zeros((self.Ah.shape[0], 6))],
                           [self.Ai, self.Ah, np.zeros((self.Ah.shape[0], 6))],
                           [np.zeros((6, 2 * self.Ah.shape[0])), np.eye(6)]])
        
        # Interaction force elements from via points
        self.A[2, 18] = self.h * self.kvp / self.m
        self.A[10, 20] = self.h * self.kvp / self.m
        self.A[2, 20] = self.h * self.kvp_ / self.m
        self.A[10, 18] = self.h * self.kvp_ / self.m
        
        self.Bh = np.block([[np.zeros((5, 2))], 
                            [np.array([[self.h / self.tau, 0], 
                            [0, 0], 
                            [0, self.h / self.tau]])]])
        
        if self.hand_id == 1:
            self.B = np.block([[self.Bh],
                                [np.zeros((self.Bh.shape[0], 2))],
                                [np.zeros((6, 2))]])
        else:
            self.B = np.block([[np.zeros((self.Bh.shape[0], 2))],
                                [self.Bh],
                                [np.zeros((6, 2))]])
        
        self.state_len = self.A.shape[0]
        self.control_len = self.B.shape[1]

        self.C = np.eye(self.A.shape[0])
        
        return self.A, self.B
    
    def set_costmatrices(self):
        self.r = self.params['r'] # weighting factor for the control cost matrix
        self.wp = self.params['wp'] # weighting factor for position cost
        self.wv = self.params['wv'] # weighting factor for velocity cost
        self.wf = self.params['wf'] # weighting factor for force cost
        self.R = self.r / (self.params['N'] - 1) * np.identity(self.control_len) # Control cost matrix
        
        if self.hand_id == 1:
            # cost matrix for p1 passing throug end target
            self.p_N = np.block([[np.concatenate(([-self.wp], np.zeros(15), [self.wp, 0, 0, 0, 0, 0]))],
                                    [np.concatenate(([0, -self.wp], np.zeros(15), [self.wp, 0, 0, 0, 0]))],
                                    [np.zeros((6, 2)), np.diag([self.wv, self.wv, self.wv, self.wf, self.wf, self.wf]),
                                    np.zeros((6, 14))]])
            
            # cost matrix for p1 passing throug vp1
            self.p_tvp = np.block([[np.concatenate(([-self.wp], np.zeros(17), [self.wp, 0, 0, 0]))],
                                    [np.concatenate(([0, -self.wp], np.zeros(17), [self.wp, 0, 0]))]])        

            # cost matrix for p1 passing throug vp2
            self.p_tvp_ = np.block([[np.concatenate(([-self.wp], np.zeros(19), [self.wp, 0]))],
                                    [np.concatenate(([0, -self.wp], np.zeros(19), [self.wp]))]])        
        else: 
            # cost matrix for p2 passing throug end target
            self.p_N = np.block([[np.concatenate((np.zeros(8), [-self.wp], np.zeros(7), [self.wp, 0, 0, 0, 0, 0]))],
                                    [np.concatenate((np.zeros(9), [-self.wp], np.zeros(7), [self.wp, 0, 0, 0, 0]))],
                                    [np.zeros((6, 10)), np.diag([self.wv, self.wv, self.wv, self.wf, self.wf, self.wf]), 
                                    np.zeros((6, 6))]])
            
            # cost matrix for p2 passing throug vp2
            self.p_tvp = np.block([[np.concatenate((np.zeros(8), [-self.wp], np.zeros(11), [self.wp, 0]))],
                                    [np.concatenate((np.zeros(9), [-self.wp], np.zeros(11), [self.wp]))]])        

            # cost matrix for p2 passing throug vp1
            self.p_tvp_ = np.block([[np.concatenate((np.zeros(8), [-self.wp], np.zeros(9), [self.wp, 0, 0, 0]))],
                                    [np.concatenate((np.zeros(9), [-self.wp], np.zeros(9), [self.wp, 0, 0]))]])        
        
        self.Q_N = 1 / 2 * self.p_N.T @ self.p_N  # State Cost matrix at last step
        self.Q_tvp = 1 / 2 * self.p_tvp.T @ self.p_tvp # State Cost matrix for passing through self via point
        self.Q_tvp_ = 1 / 2 * self.p_tvp_.T @ self.p_tvp_ # State Cost matrix for passing through others via point

        # cost matrix for general time 't'
        self.Q = np.zeros_like(self.Q_N) 
    
    def compute_controlfeedbackgains(self):
        self.L = np.zeros((self.params['N'] + 1, self.B.shape[1], self.B.shape[0])) # sequence of control feedback gains
        
        # Control feedback gain backward recursion
        S = self.Q_N
        for step in np.arange(self.params['N']-1, -1, -1):
            self.L[step, :, :] = np.linalg.inv(self.B.T @ S @ self.B + self.R) @ \
                                    self.B.T @ S @ self.A
            
            if step == self.params['N'] - 1:
                S = self.Q_N + self.A.T @ S @ (self.A - self.B @ self.L[step, :, :])
            elif step == self.vp_t:
                S = self.Q_tvp + self.A.T @ S @ (self.A - self.B @ self.L[step, :, :])
            else:
                S = self.Q + self.A.T @ S @ (self.A - self.B @ self.L[step, :, :])
    
    def set_noisevariables(self):
        self.sig_self_scale = self.params['sig self noise']
        
        s_p = self.params['sig p']
        s_v = self.params['sig v']
        s_f = self.params['sig f']
        
        self.noise_s = self.sig_self_scale * np.array([s_p, s_p, s_v, s_v, s_f, s_f, s_f, s_f, 
                                                    s_p, s_p, s_v, s_v, s_f, s_f, s_f, s_f,
                                                    s_p / 10, s_p / 10, s_p / 10, s_p / 10, s_p / 10, s_p / 10])

        self.Ws = np.diag(self.noise_s ** 2) # sensory feedback additive noise covariance matrix
        
        self.E = 0.01 * np.eye(self.A.shape[0]) # state covariance matrix initial for estimation

        f_cov = self.params['force cov']
        v_cov = self.params['vel cov']
        p_cov = self.params['pos cov']
        self.noise_c = np.array([0, 0, 0, 0, 0, f_cov, 0, f_cov,
                                 0, 0, 0, 0, 0, f_cov, 0, f_cov,
                                 0, 0, 0, 0, 0, 0])
        self.Wc = np.diag(self.noise_c ** 2)

    def estimate_state(self, x_true_next, x_est_curr, u_curr):
        y = self.C @ x_true_next + np.random.normal(0, np.sqrt(np.diag(self.Ws)))
            
        # prediction steps
        x_pred = self.A @ x_est_curr + self.B @ u_curr
        E_pred = self.A @ self.E @ self.A.T + self.Wc
        
        # update steps
        Ko = E_pred @ self.C.T @ np.linalg.inv(self.C @ E_pred @ self.C.T + self.Ws)

        x_est_next = x_pred + Ko @ (y - self.C @ x_pred)
        self.E = (np.eye(self.C.shape[1]) - Ko @ self.C) @ E_pred
        
        return x_est_next   
    
    # Computing trajectory cost
    def compute_trajectory_cost(self, x, u):
        cost = 0
        for step in range(u.shape[1]):
            if step == self.vp_t[0]:
                cost += x[:, step].T @ (self.Q_T11 + self.Q) @ x[:, step] + u[:, step].transpose() @ self.R @ u[:, step]
            elif step == self.vp_t[1]:
                cost += x[:, step].T @ (self.Q_T22 + self.Q) @ x[:, step] + u[:, step].transpose() @ self.R @ u[:, step]
            else:
                cost += x[:, step].T @ self.Q @ x[:, step] + u[:, step].transpose() @ self.R @ u[:, step]
                
        cost += x[:, step].T @ self.Q_N @ x[:, step] 

        return cost


# TEST MODEL: Differential game theoretic model. See Basar Olsder Chapter 6.
# Perfect state observation. No need for estimation all states directly available. Used to test different models.
# Option to apply individual costs or shared costs
class TwoHands_Model_GameTheoretic_Base:
    def __init__(self, params, shared_cost, vp_cross_timestep):
        self.params = params
        self.h = params['h']
        self.kh = params['k_h'] # spring constant for the spring connecting hands
        self.kvp = params['k_vp'] # spring constant for the spring connecting to self via point
        self.kvp_ = params['k_vp_'] # spring constant for the spring connecting to others via point
        self.c = params['c'] # spring constant for the spring connecting object to the hand
        self.m = params['m'] # mass of the object, mass of the hand
        self.tau = params['tau'] # time constants of the second order muscle filter
        self.vp_t = vp_cross_timestep
        self.shared_cost = shared_cost

        self.setup_model()
        self.set_costmatrices()

    def setup_model(self):
        """
        Hand States
        [0-x1, 1-y1, 
        2-vx1, 3-vy1, 
        4-fx1, 5-gx1, 
        6-fy1, 7-gy1,
        8-x1, 9-y1, 
        10-vx1, 11-vy1, 
        12-fx1, 13-gx1, 
        14-fy1, 15-gy1,
        16-targ_x, 17-targ_y,
        18-vp1_x, 19-vp1_y,
        20-vp2_x, 21-vp2_y
        """
        self.Ah = np.array([[1, 0, self.h, 0, 0, 0, 0, 0], 
                        [0, 1, 0, self.h, 0, 0, 0, 0],
                        [-self.h * self.kh / self.m - self.h * self.kvp / self.m - self.h * self.kvp_ / self.m, 0, (1 - self.h * self.c / self.m), 0, self.h / self.m, 0, 0, 0], 
                        [0, 0, 0, (1 - self.h * self.c / self.m), 0, 0, self.h / self.m, 0],
                        [0, 0, 0, 0, (1 -self.h / self.tau), self.h / self.tau, 0, 0], 
                        [0, 0, 0, 0, 0, (1 - self.h / self.tau), 0, 0],
                        [0, 0, 0, 0, 0, 0, (1 - self.h / self.tau), self.h / self.tau], 
                        [0, 0, 0, 0, 0, 0, 0, (1 - self.h / self.tau)]])
        
        # Interaction force elements from other hand
        self.Ai = np.zeros_like(self.Ah)
        self.Ai[2, 0] = self.h * self.kh / self.m
        # self.Ai[3, 1] = self.h * self.kh / self.m

        self.A = np.block([[self.Ah, self.Ai, np.zeros((self.Ah.shape[0], 6))],
                           [self.Ai, self.Ah, np.zeros((self.Ah.shape[0], 6))],
                           [np.zeros((6, 2 * self.Ah.shape[0])), np.eye(6)]])
        
        # Interaction force elements from via points
        self.A[2, 18] = self.h * self.kvp / self.m
        self.A[10, 20] = self.h * self.kvp / self.m
        self.A[2, 20] = self.h * self.kvp_ / self.m
        self.A[10, 18] = self.h * self.kvp_ / self.m

        self.Bh = np.block([[np.zeros((5, 2))], 
                            [np.array([[self.h / self.tau, 0], 
                                       [0, 0], 
                                       [0, self.h / self.tau]])]])
        self.B1 = np.block([[self.Bh],
                            [np.zeros((self.Bh.shape[0], 2))],
                            [np.zeros((6, 2))]])
        self.B2 = np.block([[np.zeros((self.Bh.shape[0], 2))],
                            [self.Bh],
                            [np.zeros((6, 2))]])
        
        self.state_len = self.A.shape[0]
        self.control_len = self.B1.shape[1]
        
        return self.A, self.B1, self.B2
    
    def set_costmatrices(self):
        self.r = self.params['r'] # weighting factor for the control cost matrix
        self.wp = self.params['wp'] # weighting factor for position cost
        self.wv = self.params['wv'] # weighting factor for velocity cost
        self.wf = self.params['wf'] # weighting factor for force cost
        self.R = self.r / (self.params['N'] - 1) * np.identity(self.control_len) # Control cost matrix for each of p1 and p2
        
        # cost matrix for p1 passing through end target
        self.p_N1 = np.block([[np.concatenate(([-self.wp], np.zeros(15), [self.wp, 0, 0, 0, 0, 0]))],
                            [np.concatenate(([0, -self.wp], np.zeros(15), [self.wp, 0, 0, 0, 0]))],
                            [np.zeros((6, 2)), np.diag([self.wv, self.wv, self.wv, self.wf, self.wf, self.wf]), 
                             np.zeros((6, 14))]])
        
        # cost matrix for p2 passing through end target
        self.p_N2 = np.block([[np.concatenate((np.zeros(8), [-self.wp], np.zeros(7), [self.wp, 0, 0, 0, 0, 0]))],
                              [np.concatenate((np.zeros(9), [-self.wp], np.zeros(7), [self.wp, 0, 0, 0, 0]))],
                              [np.zeros((6, 10)), np.diag([self.wv, self.wv, self.wv, self.wf, self.wf, self.wf]), 
                               np.zeros((6, 6))]])
        
        # State Cost matrices at last step
        self.Q_N1 = 1 / 2 * self.p_N1.T @ self.p_N1 
        self.Q_N2 = 1 / 2 * self.p_N2.T @ self.p_N2  

        # cost matrix for p1 passing throug vp1
        self.p_t11 = np.block([[np.concatenate(([-self.wp], np.zeros(17), [self.wp, 0, 0, 0]))],
                            [np.concatenate(([0, -self.wp], np.zeros(17), [self.wp, 0, 0]))]])        
        self.Q_T1 = 1 / 2 * self.p_t11.T @ self.p_t11

        # cost matrix for p2 passing throug vp2
        self.p_t22 = np.block([[np.concatenate((np.zeros(8), [-self.wp], np.zeros(11), [self.wp, 0]))],
                    [np.concatenate((np.zeros(9), [-self.wp], np.zeros(11), [self.wp]))]])        
        self.Q_T2 = 1 / 2 * self.p_t22.T @ self.p_t22

        # cost matrix for p1 passing throug vp2
        self.p_t12 = np.block([[np.concatenate(([-self.wp], np.zeros(17), [0, 0, self.wp, 0]))],
                            [np.concatenate(([0, -self.wp], np.zeros(17), [0, 0, self.wp]))]])        
        self.Q_T12 = 1 / 2 * self.p_t12.T @ self.p_t12

        # cost matrix for p2 passing throug vp1
        self.p_t21 = np.block([[np.concatenate((np.zeros(8), [-self.wp], np.zeros(9), [self.wp, 0, 0, 0]))],
                    [np.concatenate((np.zeros(9), [-self.wp], np.zeros(9), [self.wp, 0, 0]))]])        
        self.Q_T21 = 1 / 2 * self.p_t21.T @ self.p_t21

        # general cost matrix for time 't'
        self.Q1 = np.zeros_like(self.Q_N1)
        self.Q2 = np.zeros_like(self.Q_N1)

        # Cost on reducing distance between the hands
        # self.p_t = np.concatenate(([-self.wp], np.zeros(7), [self.wp], np.zeros(13)))
        # self.Q1 = 1 / 2000000 * self.p_t.reshape(len(self.p_t), 1) @ self.p_t.reshape(1, len(self.p_t))
        # self.Q2 = self.Q1.copy()
    
    def compute_controlfeedbackgains(self):
        # sequence of control feedback gains
        self.L1 = np.zeros((self.params['N'] + 1, self.B1.shape[1], self.B1.shape[0]))
        self.L2 = np.zeros((self.params['N'] + 1, self.B2.shape[1], self.B2.shape[0]))

        self.l1 = np.zeros((self.params['N'] + 1, self.B1.shape[1], 1))
        self.l2 = np.zeros((self.params['N'] + 1, self.B2.shape[1], 1))

        # Control feedback gain backward recursion
        S1 = copy.copy(self.Q_N1)
        S2 = copy.copy(self.Q_N2)
        E1 = np.zeros((self.B1.shape[0], 1))
        E2 = np.zeros_like(E1)
        for step in np.arange(self.params['N']-1, -1, -1):
            if self.shared_cost[0] and self.shared_cost[1]:
                M1 = self.R / 2 + self.B1.T @ S1 @ self.B1
                M2 = self.R / 2 + self.B2.T @ S2 @ self.B2
            elif self.shared_cost[0] and not self.shared_cost[1]:
                M1 = self.R / 2 + self.B1.T @ S1 @ self.B1
                M2 = self.R + self.B2.T @ S2 @ self.B2
            elif not self.shared_cost[0] and self.shared_cost[1]:
                M1 = self.R + self.B1.T @ S1 @ self.B1
                M2 = self.R / 2 + self.B2.T @ S2 @ self.B2
            else:
                M1 = self.R + self.B1.T @ S1 @ self.B1
                M2 = self.R + self.B2.T @ S2 @ self.B2

            self.L1[step, :, :] = np.linalg.inv(M1 - self.B1.T @ S1 @ self.B2 @ np.linalg.inv(M2) @ self.B2.T @ S2 @ self.B1) @ \
                                    (self.B1.T @ S1 @ self.A - self.B1.T @ S1 @ self.B2 @ np.linalg.inv(M2) @ self.B2.T @ S2 @ self.A)
            self.L2[step, :, :] = np.linalg.inv(M2) @ (self.B2.T @ S2 @ self.A - (self.B2.T @ S2 @ self.B1) @ self.L1[step, :, :])

            self.l1[step, :, :] = np.linalg.inv(M1 - self.B1.T @ S1 @ self.B2 @ np.linalg.inv(M2) @ self.B2.T @ S2 @ self.B1) @ \
                                    (self.B1.T @ E1 - self.B1.T @ S1 @ self.B2 @ np.linalg.inv(M2) @ self.B2.T @ E2)
            self.l2[step, :, :] = np.linalg.inv(M2) @ (self.B2.T @ E2 - (self.B2.T @ S2 @ self.B1) @ self.l1[step, :, :])

            F = self.A - self.B1 @ self.L1[step, :, :] - self.B2 @ self.L2[step, :, :]
            b = -self.B1 @ self.l1[step, :, :] -self.B2 @ self.l2[step, :, :]

            # Individual costs for each player
            if self.shared_cost[0] and self.shared_cost[1]:
                # shared costs for both players
                if step == self.params['N'] - 1:
                    S1 = self.Q_N1 / 2 + self.Q_N2 / 2 + F.T @ S1 @ F + self.L1[step, :, :].T @ self.R / 2 @ self.L1[step, :, :] + self.L2[step, :, :].T @ self.R / 2 @ self.L2[step, :, :]
                    S2 = self.Q_N2 / 2 + self.Q_N2 / 2 + F.T @ S2 @ F + self.L2[step, :, :].T @ self.R / 2 @ self.L2[step, :, :] + self.L1[step, :, :].T @ self.R / 2 @ self.L1[step, :, :] 
                elif step == self.vp_t[0]:
                    S1 = self.Q1 / 2 + self.Q2 / 2  + self.Q_T1 / 2 + F.T @ S1 @ F + self.L1[step, :, :].T @ self.R / 2 @ self.L1[step, :, :] + self.L2[step, :, :].T @ self.R / 2 @ self.L2[step, :, :]
                    S2 = self.Q1 / 2 + self.Q2 / 2 + self.Q_T1 / 2 + F.T @ S2 @ F + self.L2[step, :, :].T @ self.R / 2 @ self.L2[step, :, :] + self.L1[step, :, :].T @ self.R / 2 @ self.L1[step, :, :] 
                elif step == self.vp_t[1]:
                    S1 = self.Q1 / 2 + self.Q2 / 2 + self.Q_T2 / 2 + F.T @ S1 @ F + self.L1[step, :, :].T @ self.R / 2 @ self.L1[step, :, :] + self.L2[step, :, :].T @ self.R / 2 @ self.L2[step, :, :]
                    S2 = self.Q1 / 2 + self.Q2 / 2 + self.Q_T2 / 2 + F.T @ S2 @ F + self.L2[step, :, :].T @ self.R / 2 @ self.L2[step, :, :] + self.L1[step, :, :].T @ self.R / 2 @ self.L1[step, :, :] 
                else:
                    S1 = self.Q1 / 2 + self.Q2 / 2 + F.T @ S1 @ F + self.L1[step, :, :].T @ self.R / 2 @ self.L1[step, :, :] + self.L2[step, :, :].T @ self.R / 2 @ self.L2[step, :, :] 
                    S2 = self.Q1 / 2 + self.Q2 / 2 + F.T @ S2 @ F + self.L2[step, :, :].T @ self.R / 2 @ self.L2[step, :, :] + self.L1[step, :, :].T @ self.R / 2 @ self.L1[step, :, :]

                E1 = F.T @ (E1 + S1 @ b) + self.L1[step, :, :].T @ self.R / 2 @ self.l1[step, :, :] + self.L2[step, :, :].T @ self.R / 2 @ self.l2[step, :, :]
                E2 = F.T @ (E2 + S2 @ b) + self.L2[step, :, :].T @ self.R / 2 @ self.l2[step, :, :] + self.L1[step, :, :].T @ self.R / 2 @ self.l1[step, :, :]
            elif self.shared_cost[0] and not self.shared_cost[1]:
                # shared cost for 1 and not shared cost for 2
                if step == self.params['N'] - 1:
                    S1 = self.Q_N1 + self.Q_N2 / 2 + F.T @ S1 @ F + self.L1[step, :, :].T @ self.R / 2 @ self.L1[step, :, :] + self.L2[step, :, :].T @ self.R / 2 @ self.L2[step, :, :]
                    S2 = self.Q_N2 + F.T @ S2 @ F + self.L2[step, :, :].T @ self.R @ self.L2[step, :, :]
                elif step == self.vp_t[0]:
                    S1 = self.Q1 + self.Q2 / 2 + self.Q_T1 + F.T @ S1 @ F + self.L1[step, :, :].T @ self.R / 2 @ self.L1[step, :, :] + self.L2[step, :, :].T @ self.R / 2 @ self.L2[step, :, :]
                    S2 = self.Q2 + F.T @ S2 @ F + self.L2[step, :, :].T @ self.R @ self.L2[step, :, :]
                elif step == self.vp_t[1]:
                    S1 = self.Q1 + self.Q2 / 2 + self.Q_T2 / 2 + F.T @ S1 @ F + self.L1[step, :, :].T @ self.R / 2 @ self.L1[step, :, :] + self.L2[step, :, :].T @ self.R / 2 @ self.L2[step, :, :]
                    S2 = self.Q2 + self.Q_T2 + F.T @ S2 @ F + self.L2[step, :, :].T @ self.R @ self.L2[step, :, :]
                else:
                    S1 = self.Q1 + self.Q2 / 2 + F.T @ S1 @ F + self.L1[step, :, :].T @ self.R / 2 @ self.L1[step, :, :] + self.L2[step, :, :].T @ self.R / 2 @ self.L2[step, :, :] 
                    S2 = self.Q2 + F.T @ S2 @ F + self.L2[step, :, :].T @ self.R @ self.L2[step, :, :]

                E1 = F.T @ (E1 + S1 @ b) + self.L1[step, :, :].T @ self.R / 2 @ self.l1[step, :, :] + self.L2[step, :, :].T @ self.R / 2 @ self.l2[step, :, :]
                E2 = F.T @ (E2 + S2 @ b) + self.L2[step, :, :].T @ self.R @ self.l2[step, :, :]
            elif not self.shared_cost[0] and self.shared_cost[1]:
                # shared cost for 2 and not shared cost for 1
                if step == self.params['N'] - 1:
                    S1 = self.Q_N1 + F.T @ S1 @ F + self.L1[step, :, :].T @ self.R @ self.L1[step, :, :]
                    S2 = self.Q_N2 + self.Q_N1 / 2 + F.T @ S2 @ F + self.L2[step, :, :].T @ self.R / 2 @ self.L2[step, :, :] + self.L1[step, :, :].T @ self.R / 2 @ self.L1[step, :, :] 
                elif step == self.vp_t[0]:
                    S1 = self.Q1 + self.Q_T1 + F.T @ S1 @ F + self.L1[step, :, :].T @ self.R @ self.L1[step, :, :]
                    S2 = self.Q1 / 2 + self.Q2 + self.Q_T1 / 2 + F.T @ S2 @ F + self.L2[step, :, :].T @ self.R / 2 @ self.L2[step, :, :] + self.L1[step, :, :].T @ self.R / 2 @ self.L1[step, :, :] 
                elif step == self.vp_t[1]:
                    S1 = self.Q1 + F.T @ S1 @ F + self.L1[step, :, :].T @ self.R @ self.L1[step, :, :]
                    S2 = self.Q1 / 2 + self.Q2 + self.Q_T2 + F.T @ S2 @ F + self.L2[step, :, :].T @ self.R / 2 @ self.L2[step, :, :] + self.L1[step, :, :].T @ self.R / 2 @ self.L1[step, :, :] 
                else:
                    S1 = self.Q1 + F.T @ S1 @ F + self.L1[step, :, :].T @ self.R @ self.L1[step, :, :]
                    S2 = self.Q1 / 2 + self.Q2 + F.T @ S2 @ F + self.L2[step, :, :].T @ self.R / 2 @ self.L2[step, :, :] + self.L1[step, :, :].T @ self.R / 2 @ self.L1[step, :, :]

                E1 = F.T @ (E1 + S1 @ b) + self.L1[step, :, :].T @ self.R @ self.l1[step, :, :] 
                E2 = F.T @ (E2 + S2 @ b) + self.L2[step, :, :].T @ self.R / 2 @ self.l2[step, :, :] + self.L1[step, :, :].T @ self.R / 2 @ self.l1[step, :, :]
            else:
                # individual costs for each player
                if step == self.params['N'] - 1:
                    S1 = self.Q_N1 + F.T @ S1 @ F + self.L1[step, :, :].T @ self.R @ self.L1[step, :, :]
                    S2 = self.Q_N2 + F.T @ S2 @ F + self.L2[step, :, :].T @ self.R @ self.L2[step, :, :]
                elif step == self.vp_t[0]:
                    S1 = self.Q1 + self.Q_T1 + F.T @ S1 @ F + self.L1[step, :, :].T @ self.R @ self.L1[step, :, :]
                    S2 = self.Q2 + F.T @ S2 @ F + self.L2[step, :, :].T @ self.R @ self.L2[step, :, :]
                elif step == self.vp_t[1]:
                    S1 = self.Q1 + F.T @ S1 @ F + self.L1[step, :, :].T @ self.R @ self.L1[step, :, :]
                    S2 = self.Q2 + self.Q_T2 + F.T @ S2 @ F + self.L2[step, :, :].T @ self.R @ self.L2[step, :, :]
                else:
                    S1 = self.Q1 + F.T @ S1 @ F + self.L1[step, :, :].T @ self.R @ self.L1[step, :, :]
                    S2 = self.Q2 + F.T @ S2 @ F + self.L2[step, :, :].T @ self.R @ self.L2[step, :, :]
                
                E1 = F.T @ (E1 + S1 @ b) + self.L1[step, :, :].T @ self.R @ self.l1[step, :, :] 
                E2 = F.T @ (E2 + S2 @ b) + self.L2[step, :, :].T @ self.R @ self.l2[step, :, :] 

    def compute_trajectory_cost(self, x, u1, u2):
        cost1 = 0
        cost2 = 0
        for step in range(u1.shape[1]):
            if not self.shared_cost[0] and not self.shared_cost[1]:
                if step == self.vp_t[0]:
                    cost1 += x[:, step].T @ (self.Q1 + self.Q_T1) @ x[:, step] + u1[:, step].T @ self.R @ u1[:, step]
                    cost2 += x[:, step].T @ self.Q2 @ x[:, step] + u2[:, step].T @ self.R @ u2[:, step]
                elif step == self.vp_t[1]:
                    cost1 += x[:, step].T @ self.Q1 @ x[:, step] + u1[:, step].T @ self.R @ u1[:, step]
                    cost2 += x[:, step].T @ (self.Q2 + self.Q_T2) @ x[:, step] + u2[:, step].T @ self.R @ u2[:, step]
                else:
                    cost1 += x[:, step].T @ self.Q1 @ x[:, step] + u1[:, step].T @ self.R @ u1[:, step]
                    cost2 += x[:, step].T @ self.Q2 @ x[:, step] + u2[:, step].T @ self.R @ u2[:, step]
            elif self.shared_cost[0] and not self.shared_cost[1]:
                if step == self.vp_t[0]:
                    cost1 += x[:, step].T @ (self.Q1 + self.Q2 / 2 + self.Q_T1) @ x[:, step] + u1[:, step].T @ self.R / 2 @ u1[:, step] + u2[:, step].T @ self.R / 2 @ u2[:, step]
                    cost2 += x[:, step].T @ self.Q2 @ x[:, step] + u2[:, step].T @ self.R @ u2[:, step]
                elif step == self.vp_t[1]:
                    cost1 += x[:, step].T @ (self.Q1 + self.Q2 / 2 + self.Q_T2 / 2) @ x[:, step] + u1[:, step].T @ self.R / 2 @ u1[:, step] + u2[:, step].T @ self.R / 2 @ u2[:, step]
                    cost2 += x[:, step].T @ (self.Q2 + self.Q_T2) @ x[:, step] + u2[:, step].T @ self.R @ u2[:, step]
                else:
                    cost1 += x[:, step].T @ (self.Q1 + self.Q2 / 2) @ x[:, step] + u1[:, step].T @ self.R / 2 @ u1[:, step] + u2[:, step].T @ self.R / 2 @ u2[:, step]
                    cost2 += x[:, step].T @ self.Q2 @ x[:, step] + u2[:, step].T @ self.R @ u2[:, step]
            elif not self.shared_cost[0] and self.shared_cost[1]:
                if step == self.vp_t[0]:
                    cost1 += x[:, step].T @ (self.Q1 + self.Q_T1) @ x[:, step] + u1[:, step].T @ self.R @ u1[:, step]
                    cost2 += x[:, step].T @ (self.Q1 / 2 + self.Q2 + self.Q_T1 / 2) @ x[:, step] + u2[:, step].T @ self.R / 2 @ u2[:, step] + u1[:, step].T @ self.R / 2 @ u1[:, step]
                elif step == self.vp_t[1]:
                    cost1 += x[:, step].T @ self.Q1 @ x[:, step] + u1[:, step].T @ self.R @ u1[:, step]
                    cost2 += x[:, step].T @ (self.Q1 / 2 + self.Q2 + self.Q_T2) @ x[:, step] + u2[:, step].T @ self.R / 2 @ u2[:, step] + u1[:, step].T @ self.R / 2 @ u1[:, step]
                else:
                    cost1 += x[:, step].T @ self.Q1 @ x[:, step] + u1[:, step].T @ self.R @ u1[:, step]
                    cost2 += x[:, step].T @ (self.Q1 / 2 + self.Q2) @ x[:, step] + u2[:, step].T @ self.R / 2 @ u2[:, step] + u1[:, step].T @ self.R / 2 @ u1[:, step]
            else:            
                if step == self.vp_t[0]:
                    cost1 += x[:, step].T @ (self.Q1 / 2 + self.Q2 / 2 + self.Q_T1 / 2) @ x[:, step] + u1[:, step].transpose() @ self.R / 2 @ u1[:, step] + u2[:, step].transpose() @ self.R / 2 @ u2[:, step]
                    cost2 += x[:, step].T @ (self.Q1 / 2 + self.Q2 / 2 + self.Q_T1 / 2) @ x[:, step] + u2[:, step].transpose() @ self.R / 2 @ u2[:, step] + u1[:, step].transpose() @ self.R / 2 @ u1[:, step]
                elif step == self.vp_t[1]:
                    cost1 += x[:, step].T @ (self.Q1 / 2 + self.Q2 / 2 + self.Q_T2 / 2) @ x[:, step] + u1[:, step].transpose() @ self.R / 2 @ u1[:, step] + u2[:, step].transpose() @ self.R / 2 @ u2[:, step]
                    cost2 += x[:, step].T @ (self.Q1 / 2 + self.Q2 / 2 + self.Q_T2 / 2) @ x[:, step] + u2[:, step].transpose() @ self.R / 2 @ u2[:, step] + u1[:, step].transpose() @ self.R / 2 @ u1[:, step]
                else:
                    cost1 += x[:, step].T @ (self.Q1 / 2 + self.Q2 / 2) @ x[:, step] + u1[:, step].transpose() @ self.R / 2 @ u1[:, step] + u2[:, step].transpose() @ self.R / 2 @ u2[:, step]
                    cost2 += x[:, step].T @ (self.Q1 / 2 + self.Q2 / 2) @ x[:, step] + u2[:, step].transpose() @ self.R / 2 @ u2[:, step] + u1[:, step].transpose() @ self.R / 2 @ u1[:, step]

        cost1 += x[:, -1].T @ self.Q_N1 @ x[:, -1]
        cost2 += x[:, -1].T @ self.Q_N2 @ x[:, -1]

        return cost1, cost2


# Differential game theoretic model. See Basar Olsder Chapter 6.
# Full state observation.
# Option to apply individual costs or shared costs 
class TwoHands_Model_GameTheoretic_StateEst(TwoHands_Model_GameTheoretic_Base):
    def __init__(self, params, shared_cost, vp_cross_timestep):
        self.params = params
        self.h = params['h']
        self.kh = params['k_h'] # spring constant for the spring connecting hands
        self.kvp = params['k_vp'] # spring constant for the spring connecting to self via point
        self.kvp_ = params['k_vp_'] # spring constant for the spring connecting to others via point
        self.c = params['c'] # spring constant for the spring connecting object to the hand
        self.m = params['m'] # mass of the object, mass of the hand
        self.tau = params['tau'] # time constants of the second order muscle filter
        self.vp_t = vp_cross_timestep
        self.shared_cost = shared_cost
        
        self.setup_model()
        self.setup_observationmatrices()
        self.set_costmatrices()
        self.set_noisevariables()

    def setup_observationmatrices(self):
        self.C1 = np.eye(self.A.shape[0])
        self.C2 = np.eye(self.A.shape[0])

    def set_noisevariables(self):
        self.sig_self_scale = self.params['sig self noise']
        self.sig_partner_scale = self.params['sig partner noise']

        s_p = self.params['sig p']
        s_v = self.params['sig v']
        s_f = self.params['sig f']
        
        self.noise_s1 = np.concatenate((self.sig_self_scale * np.array([s_p, s_p, s_v, s_v, s_f, s_f, s_f, s_f]), 
                                        self.sig_partner_scale * np.array([s_p, s_p, s_v, s_v, s_f, s_f, s_f, s_f]),
                                        self.sig_self_scale * np.array([s_p / 10, s_p / 10, s_p / 10, s_p / 10, s_p / 10, s_p / 10])))
        
        self.noise_s2 = np.concatenate((self.sig_partner_scale * np.array([s_p, s_p, s_v, s_v, s_f, s_f, s_f, s_f]), 
                                        self.sig_self_scale * np.array([s_p, s_p, s_v, s_v, s_f, s_f, s_f, s_f]),
                                        self.sig_self_scale * np.array([s_p / 10, s_p / 10, s_p / 10, s_p / 10, s_p / 10, s_p / 10])))

        self.Ws1 = np.diag(self.noise_s1 ** 2) 
        self.Ws2 = np.diag(self.noise_s2 ** 2) 

        self.E1 = 0.01 * np.eye(self.A.shape[0]) # state covariance matrix initial for estimation
        self.E2 = 0.01 * np.eye(self.A.shape[0]) # state covariance matrix initial for estimation

        f_cov = self.params['force cov']
        v_cov = self.params['vel cov']
        p_cov = self.params['pos cov'] 

        self.noise_c1 = np.array([0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, v_cov, v_cov, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0])
        self.noise_c2 = np.array([0, 0, v_cov, v_cov, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0])
        
        self.Wc1 = np.diag(self.noise_c1 ** 2)
        self.Wc2 = np.diag(self.noise_c2 ** 2)

    def estimate_state(self, x_true_next, x_est_curr, u_curr, p_id):
        C, Ws, B, Wc, E = (self.C1, self.Ws1, self.B1, self.Wc1, self.E1) if p_id == 1 else\
                             (self.C2, self.Ws2, self.B2, self.Wc2, self.E2)

        y = C @ x_true_next + np.random.normal(0, np.sqrt(np.diag(Ws)))
            
        # prediction steps
        x_pred = self.A @ x_est_curr + B @ u_curr
        E_pred = self.A @ E @ self.A.T + Wc
        
        # update steps
        Ko = E_pred @ C.T @ np.linalg.inv(C @ E_pred @ C.T + Ws)

        x_est_next = x_pred + Ko @ (y - C @ x_pred)

        if p_id == 1:
            self.E1 = (np.eye(C.shape[1]) - Ko @ C) @ E_pred
        else:
            self.E2 = (np.eye(C.shape[1]) - Ko @ C) @ E_pred
                
        return x_est_next


# Differential game theoretic model. See Basar Olsder Chapter 6. 
# Full state observation not enough. Require estimation of partner control for unbiased state estimation of all states. 
class TwoHands_Model_GameTheoretic_ControlEst(TwoHands_Model_GameTheoretic_Base):
    def __init__(self, params, shared_cost, vp_cross_timestep):
        self.params = params
        self.h = params['h']
        self.kh = params['k_h'] # spring constant for the spring connecting hands
        self.kvp = params['k_vp'] # spring constant for the spring connecting to self via point
        self.kvp_ = params['k_vp_'] # spring constant for the spring connecting to others via point
        self.c = params['c'] # spring constant for the spring connecting object to the hand
        self.m = params['m'] # mass of the object, mass of the hand
        self.tau = params['tau'] # time constants of the second order muscle filter
        self.vp_t = vp_cross_timestep
        self.shared_cost = shared_cost

        self.setup_model()
        self.setup_observationmatrices()
        self.set_costmatrices()
        self.set_noisevariables()

    def setup_observationmatrices(self):
        self.A1_aug = np.block([[self.A, self.B2], [np.zeros((2, self.A.shape[0])), 0.95 * np.eye(self.B1.shape[1])]])
        self.A2_aug = np.block([[self.A, self.B1], [np.zeros((2, self.A.shape[0])), 0.95 * np.eye(self.B1.shape[1])]])

        self.B1_aug = np.block([[self.B1], [np.zeros((self.B1.shape[1], self.B1.shape[1]))]])
        self.B2_aug = np.block([[self.B2], [np.zeros((self.B2.shape[1], self.B1.shape[1]))]])
    
        # self.C1 = np.zeros((18, self.A1_aug.shape[0]))
        # self.C1[np.arange(self.C1.shape[0]), 
        #         np.concatenate((np.arange(12), np.arange(16, 22)))] = 1

        # self.C2 = np.zeros((18, self.A2_aug.shape[0]))
        # self.C2[np.arange(self.C2.shape[0]), 
        #         np.concatenate(([0, 1, 2, 3, 8, 9, 10, 11, 12, 13, 14, 15], np.arange(16, 22)))] = 1
        
        self.C1 = np.zeros((22, self.A1_aug.shape[0]))
        self.C1[np.arange(self.C1.shape[0]), np.arange(self.C1.shape[0])] = 1

        self.C2 = np.zeros((22, self.A2_aug.shape[0]))
        self.C2[np.arange(self.C2.shape[0]), np.arange(self.C2.shape[0])] = 1

    def set_noisevariables(self):
        self.sig_self_scale = self.params['sig self noise']
        self.sig_partner_scale = self.params['sig partner noise']

        s_p = self.params['sig p']
        s_v = self.params['sig v']
        s_f = self.params['sig f']
        
        self.noise_s1 = np.concatenate((self.sig_self_scale * np.array([s_p, s_p, s_v, s_v, s_f, s_f, s_f, s_f]), 
                                        self.sig_partner_scale * np.array([s_p, s_p, s_v, s_v, s_f, s_f, s_f, s_f]),
                                        self.sig_self_scale * np.array([s_p / 10, s_p / 10, s_p / 10, s_p / 10, s_p / 10, s_p / 10])))
        
        self.noise_s2 = np.concatenate((self.sig_partner_scale * np.array([s_p, s_p, s_v, s_v, s_f, s_f, s_f, s_f]), 
                                        self.sig_self_scale * np.array([s_p, s_p, s_v, s_v, s_f, s_f, s_f, s_f]),
                                        self.sig_self_scale * np.array([s_p / 10, s_p / 10, s_p / 10, s_p / 10, s_p / 10, s_p / 10])))
        
        self.Ws1 = np.diag(self.noise_s1 ** 2) 
        self.Ws2 = np.diag(self.noise_s2 ** 2) 

        self.E1 = 0.01 * np.eye(self.A1_aug.shape[0]) # state covariance matrix initial for estimation
        self.E2 = 0.01 * np.eye(self.A2_aug.shape[0]) # state covariance matrix initial for estimation

        u_cov = self.params['u cov']
        v_cov = self.params['vel cov']
        f_cov = self.params['pos cov'] 

        self.noise_c1 = np.array([0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0,
                                 u_cov, u_cov])
        self.noise_c2 = np.array([0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0,
                                u_cov, u_cov])
        
        self.Wc1 = np.diag(self.noise_c1 ** 2)
        self.Wc2 = np.diag(self.noise_c2 ** 2)

    def estimate_state(self, x_true_next, x_est_curr, u_curr, p_id):
        A, C, Ws, B, Wc, E = (self.A1_aug, self.C1, self.Ws1, self.B1_aug, self.Wc1, self.E1) if p_id == 1 else\
                             (self.A2_aug, self.C2, self.Ws2, self.B2_aug, self.Wc2, self.E2)
        
        y = C @ np.concatenate((x_true_next, [0, 0])) + np.random.normal(0, np.sqrt(np.diag(Ws)))

        # prediction steps
        x_pred = A @ x_est_curr + B @ u_curr

        E_pred = A @ E @ A.T + Wc
        
        # update steps
        Ko = E_pred @ C.T @ np.linalg.inv(C @ E_pred @ C.T + Ws)

        x_est_next = x_pred + Ko @ (y - C @ x_pred)

        if p_id == 1:
            self.E1 = (np.eye(C.shape[1]) - Ko @ C) @ E_pred
        else:
            self.E2 = (np.eye(C.shape[1]) - Ko @ C) @ E_pred
                
        return x_est_next


