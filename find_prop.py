# -*- coding: utf-8 -*-
"""
Created on Thu Apr 12 14:46:42 2018

@author: Jimmy-Exjobb
"""
import skopt
import sys
import subprocess
import numpy as np
import re
#import cs
import os
import glob

##SIZE OF LISTENER VECTOR
vector_size = 6

test_array = []

def run_simulations(arguments):
    cp = subprocess.run(arguments, stdout=subprocess.PIPE)
    
    
    
def run_simulation_first(bend_true, stretch_true, true_or_temp):
    
    arguments = ['python', 'C:/Users/Jimmy-Exjobb/Desktop/Master Thesis Final/test_robot.agxpy'
                 ,'--timeStep', '0.01',
                 '--stopAfter', '3',
 ##                '-a',
                 '--bend', str(bend_true),
                 '--stretch', str(stretch_true),
                 '--test'
                 ]
            
    cp = subprocess.run(arguments, stdout=subprocess.PIPE)

#def run_simulations(arguments):
#    cp = subprocess.run(arguments, stdout=subprocess.PIPE)
    
    
    
def run_simulation_first_damping(bend_true, stretch_true, true_or_temp):
    
    arguments = ['python', 'C:/Users/Jimmy-Exjobb/Desktop/Master Thesis Final/test_robot2.agxpy'
                 ,'--timeStep', '0.01',
                 '--stopAfter', '3',
 ##                '-a',
                 '--bend', str(bend_true),
                 '--stretch', str(stretch_true),
                 '--test'
                 ]
            
    cp = subprocess.run(arguments, stdout=subprocess.PIPE)
    
##C:\Users\Jimmy-Exjobb\Desktop\Master Thesis Final\data\positions




def read_files(file_type):
    marker_max = 0
    step_max = -1
    if file_type == 'temp':
        files_temp = glob.glob('data/positions/temp/*.npz')
        print("Reading temporary position files ...")
    if file_type == 'true':
        files_temp = glob.glob('data/positions/true/*.npz')
        print("Reading true position files ...")
    for file in files_temp:
        temp = (re.findall(r'\d+', file))
        marker_n = int(temp[0])
        step_n = int(temp[1])
        
        if marker_n>marker_max:
            marker_max = marker_n
        if step_n>step_max:
            step_max = step_n
            
    ##create empty numpy array
        
    np_array = np.zeros((step_max+1, marker_max, vector_size))
    
    ##put the data    
    for file in files_temp:
        file_np = np.load(file)
        temp = (re.findall(r'\d+', file))
        marker_n = int(temp[0])
        step_n = int(temp[1])
        data = file_np.f.arr_0
        np_array[step_n][marker_n-1] = data
    #print(np_array[step_max,:,0:3])
    #print(np.sum(np_array[step_max,:,1:3]))
    
    
    return (np_array, step_max)

    

    
    
    

def main():
    
#==============================================================================
#     a = 0
#     call = 1
#     '''Define true values that the function will try to "guess"'''
#     bend = 5e5
#     stretch = 1e5
#     true_or_temp = False
#     '''Run a simulation to get "true" positions'''
#     run_simulation_first(bend, stretch, true_or_temp)
# 
#     '''Get true positions from files'''
#     pos_true, step_max = read_files('true')
#     print(step_max)
# #==============================================================================
# #     for step_iter in range(0,step_max-1):
# #         a += np.sum(np.power(pos_temp[step_iter,:,1:4]-pos_true[step_iter,:,1:4],2))
# #     print(a)
# #==============================================================================
#     true_or_temp = False
#     
#     def find_youngs(vec):
#         ''' This function starts the simulation of the swinging string and sets the
#         bend and stretch YM. It then parses the position of the markers and
#         calculates an error between the true position and the parsed. The error is returned.
#         vec - is a vector with parameters from the optimization function
#         the first value is bending the second one is stretching coefficient        '''
#         arguments = ['python', 'C:/Users/Jimmy-Exjobb/Desktop/Master Thesis Final/test_robot.agxpy',
#                      '--timeStep', '0.01',
#                      '--stopAfter', '3',
#                      '-a',
#                      '--bend', str(vec[0]),
#                      '--stretch', str(vec[1]),
#                      ]
#         
#         run_simulations(arguments)
#         pos_temp, step_max_temp = read_files('temp')
#         e = 0
#         ##error calculation
#         for step_iter in range(0,step_max):
#             e += np.sum(np.power(pos_temp[step_iter,:,1:4]-pos_true[step_iter,:,1:4],2))
#         nonlocal call
#         print('Call number %d' % call)
#         call += 1
#         print(e)
#         return e
# 
#     '''Call the function'''
#     res = skopt.gp_minimize(find_youngs, [(1e5, 1e6), (5e4, 5e5)], n_calls=200)
#     print(res['x'][0], res['x'][1])
#     young_bend = res['x'][0]
#     young_stretch = res['x'][1]
# 
#     print('Estimated bending: %f and stretching: %f'  % (young_bend, young_stretch))
#     print('True bend: %f and strech: %f' %(bend, stretch))
#     
#     absolute_bend_error = young_bend - bend
#     absolute_stretch_error = young_stretch - stretch
#     percentage_bend_error = 100 * ((bend - young_bend)/bend)
#     percentage_stretch_error = 100 * ((stretch - young_stretch)/stretch)
#     
#     print('Absolute bend error: %f Absolute stretch error: %f' % (abs(absolute_bend_error), abs(absolute_stretch_error)))
#     print('Percent bend error: %f, Percent stretch error: %f' % (abs(percentage_bend_error), abs(percentage_stretch_error)))
#==============================================================================
    
    
    '''Start of damping part'''
    

    call = 1
    '''Define the true values that the function will try to "guess"'''
    bend = 1
    stretch = 1
    true_or_temp = False
    '''Run a simulation to get "true" positions'''
    run_simulation_first_damping(bend, stretch, true_or_temp)

    '''Get true positions from files'''
    pos_true, step_max = read_files('true')
    print(step_max)
    
    def find_damping(vec):
        ''' This function starts the simulation of the swinging string and sets the
        bend and stretch YM. It then parses the position of the markers and
        calculates an error between the true position and the parsed. The error is returned.
        vec - is a vector with parameters from the optimization function
        the first value is bending the second one is stretching coefficient        '''
        arguments = ['python', 'C:/Users/Jimmy-Exjobb/Desktop/Master Thesis Final/test_robot2.agxpy',
                     '--timeStep', '0.01',
                     '--stopAfter', '3',
                     '-a',
                     '--bend', str(vec[0]),
                     '--stretch', str(vec[1])
                     ]
        
        run_simulations(arguments)
        pos_temp, step_max_temp = read_files('temp')
        e = 0
        ##error calculation
        for step_iter in range(0,step_max):
            e += np.sum(np.power(pos_temp[step_iter,:,1:4]-pos_true[step_iter,:,1:4],2))
        print(e)
        nonlocal call
        print('Call number %d' % call)
        call += 1
        return e
    res = skopt.gp_minimize(find_damping, [(0.01, 2), (0.01, 2)], n_calls=400)
    print(res['x'][0], res['x'][1])
    damp_bend = res['x'][0]
    damp_stretch = res['x'][1]

    print('Estimated bending: %f and stretching: %f'  % (damp_bend, damp_stretch))
    print('True bend: %f and stretch: %f' % (bend, stretch))
    
    absolute_bend_error = damp_bend - bend
    absolute_stretch_error = damp_stretch - stretch
    percentage_bend_error = 100 * ((bend - damp_bend)/bend)
    percentage_stretch_error = 100 * ((stretch - damp_stretch)/stretch)
    
    print('Absolute bend error: %f Absolute stretch error: %f' % (abs(absolute_bend_error), abs(absolute_stretch_error)))
    print('Percent bend error: %f, Percent stretch error: %f' % (abs(percentage_bend_error), abs(percentage_stretch_error)))
if __name__ == "__main__":
    main()

    
    
#==============================================================================
#     def initiateDir(directory):
#         ''' Initiate directory path given.'''
#         # If the folder exists remove the contents
#         if os.path.isdir(directory):
#             files = os.listdir(directory)
#             for f in files:
#                 try:
#                     os.remove(os.path.join(directory,f))
#                 except Exception as e:
#                         print(e)
#         else:
#             os.makedirs(directory)
#         # Create the dir where the temporary position numpy array are saved
#     path = 'data/free_swinging_string/'
#     temp_path = '%stemp' % path
#     true_path = '%strue' % path
#     result_path = 'output/'
#     initiateDir(temp_path)
# 
#     pos_true = []
#     # Load the true or measured positions
#     for f in os.listdir(true_path):
#         if os.path.splitext(f)[1] == '.npz':
#             pos = np.load(os.path.join(true_path, f))
#             pos_true.append(pos['arr_0'])
#         else:
#             print('%s is not a npz file' % f)
# 
#     time = pos_true[0][-1,0]
#     call = 0
#     
#     def find(x):
#         
#         """ a - Only step AGX, no graphics"""
#         
#         ## Define the arguments the simulations should start with
#         arguments = ['python', 'C:/Users/Jimmy-Exjobb/Desktop/Master Thesis/RobotCableAnalyzer/scripts/robot_listener.agxpy',
#                      '-a',
#                      '--timeStep','0.01',
#                      '--stopAfter', '5',
#                      '--o', temp_path,
#                      '--stretch', str(x[0]),
#                      '--bend', str(x[1])]
#         ## Run the simulations
#         # Run the simulations
#         cp = subprocess.run(arguments, stdout=subprocess.PIPE)
#         if cp.returncode != 0:
#             print('ERROR! One simulation did not end well.', file=sys.stderr)
# 
#         pos_temp = []
#         # Load the true or measured positions
#         for f in os.listdir(temp_path):
#             if os.path.splitext(f)[1] == '.npz':
#                 pos = np.load(os.path.join(temp_path, f))
#                 pos_temp.append(pos['arr_0'])
#             else:
#                 print('%s is not a npz file' % f)
# 
#         if len(pos_temp) != len(pos_true):
#             print('Not the same number of markers found.', file=sys.stderr)
#             sys.exit(1)
#         e = 0
#         for p_temp, p_true in zip(pos_temp, pos_true):
#             e += np.sum(np.power(p_temp[:,1:4] - p_true[:,1:4], 2))
# 
#         nonlocal call
#         print('Call number %d' % call)
#         call += 1
#         return e
# 
#         # Each pos measurement must have been done on the same timesteps. Check that!
# 
#         # Calculate the error at the last time step 
#                         
#         res = skopt.gp_minimize(find, [(5e4,5e5), (1e5, 1e6)], n_calls=50)
#==============================================================================
