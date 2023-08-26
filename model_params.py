# -*- coding: utf-8 -*-
import numpy as np
import os

h = 0.01 # time step in s
N = int(3.0 / h) # total number of time steps

target_coordinates = (0.0, 0.15) # coordinates of the target

"""
mh: mass of the hand (kg),
k: spring constant of haptic spring connecting the participant hands (N/m)
c: damping constant modeling the viscous properties of muscles and limbs (Ns/ m)
tau: time constant of the second order filter modeling activation to force (s)
r: control cost weighting
wp: position cost weighting
wv: velocity cost weighting
wf: force cost weighting
sig_u: standard deviation of process or control noise
sig_p: standard deviation of position measurement noise
sig_v: standard deviation of velocity measurement noise
sig_f: standard deviation of force measurement noise
sig prop noise: scaling of measurement noise standard deviation for states derived from proprioception
sig haptic noise: scaling of measurement noise standard deviation for states derived from partner haptic
sig vision noise: scaling of measurement noise standard deviation for states derived from partner vision
haptic delay: time delay of haptic partner feedback or self propcioceptive feedback
visual delay: time delay of visual partner feedback 
vel cov: scaling for velocity covariance in process noise covariance matrix
pos cov: scaling for position covariance in process noise covariance matrix
settle_steps: step count to settle at target
"""

params = {'h': h, 'N': N,
        'm': 1.0, 'k_h': 50, 'k_vp': 0, 'k_vp_': 0, 'c': 20, 'tau': 0.04,
        'r': 0.00001, 'wp': 1, 'wv': 0.2, 'wf': 0.01,
        'sig_u': 0.05, 'sig p': 0.01, 'sig v': 0.1, 'sig f': 1,
        'sig prop noise': 0.4, 'sig partner noise': 0.2, 'haptic delay': 5, 'sig self noise':0.2, 'visual delay': 10,
        'vel cov': 0.03, 'pos cov': 0.001, 'force cov': 0.01, 'u cov': 0.2,
        'settle_steps': 1}

process_noise = params['sig_u'] * np.array([0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0,       
                                            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # process additive noise standard deviations

hand_init = [0, 0] # hand initial x coordinates

target_coordinates = [0, 0.15]

viapoint1_coordinates = [0.05, 0.05]
viapoint2_coordinates = [-0.05, 0.1]

viapoint_crossing_timestep = [100, 200]

data_save_directory = os.getcwd() + r'\\Data'






