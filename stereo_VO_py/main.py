#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar 20 11:58:43 2021

@author: kamil
"""
# Write '%matplotlib qt' to show plots in separate windows!

# Import libraries
import numpy as np
import cv2
import time
import matplotlib.pyplot as plt
import warnings
warnings.filterwarnings("ignore")

import KITTIDatasetWIN
import solver3Dto2D as solver
import helperFunctions as hf

# =============================================================================
# def showTrajectory():
#     N = len(poses)
#     xs_est=np.zeros((N))
#     ys_est=np.zeros((N))
#     xs_gt=np.zeros((N))
#     ys_gt=np.zeros((N))
#     
#     for i in range(N):
#         xs_est[i] = poses[i][2,3]
#         ys_est[i] = -poses[i][0,3]
#         xs_gt[i] = gts[i][2,3]
#         ys_gt[i] = -gts[i][0,3]
#     
#     # Plot
#     plt.plot(xs_gt, ys_gt, '.', color='red');
#     plt.plot(xs_est, ys_est, '.', color='black');
#     plt.gca().set_aspect('equal', adjustable='datalim')
# =============================================================================
    
def showTrajectory():
    xs_est = np.array(poses)[:,2,3]
    ys_est = -np.array(poses)[:,0,3]
    xs_gt = np.array(gts)[:,2,3]
    ys_gt = -np.array(gts)[:,0,3]
    # Plot
    plt.plot(xs_gt, ys_gt, '.', color='red');
    plt.plot(xs_est, ys_est, '.', color='black');
    plt.gca().set_aspect('equal', adjustable='datalim')

def exeTimeSummary(exeTime):
    print('='*30)
    print('Exectution times summary ')
    for key in exeTime:
        try:
            print('-'*30)
            print(key+' time')
            print('Mean: {:2f} ms'.format(np.mean(exeTime[key])))
            print('STD:  {:2f} ms'.format(np.std(exeTime[key])))
            print('MIN:  {:2f} ms'.format(np.min(exeTime[key])))
            print('MAX:  {:2f} ms'.format(np.max(exeTime[key])))
        except:
            pass

    print('='*30)
    
# Dataset parameters
dataset_path = 'C:\\Users\\zcq\\Downloads\\data_odometry_gray\\dataset\\'
sequence_no = '06'

# Class that fetches the data
dataset=KITTIDatasetWIN.KITTIDatasetWIN(dataset_path,sequence_no)
cam1,cam2=dataset.getCameraMatrix()

sol = solver.solver3Dto2D(cam1[0:3,0:3],0.99,0.7)


pose_global = np.eye(4)
poses = [pose_global]       # History of estimated poses
gts = []                    # History of ground truth poses

matcher = cv2.BFMatcher(cv2.NORM_HAMMING)

iteration = 0

exeTime = {'data':[],'detectFeatures':[],'stereoMatch':[],
           'matchFeatures':[],'pnp':[],'plot':[],'loop':[]}

# While there are frames left, grab one
while True:
    
    # Print progress
    if iteration%10==0:
        print(iteration)
    
    t_loop = time.time()
    
    # Read next frames and save time
    t0 = time.time()
    frame_available,fl,fr,gt = dataset.getData()
    exeTime['data'].append((time.time()-t_loop)*1000)

    # Quit if there is no frame
    if not frame_available:
        break

    gts.append(gt)

    t0 = time.time()
    # find keypoints on left
    orb = cv2.ORB_create(nfeatures=2000)
    kpL, desL = orb.detectAndCompute(fl,None)
    kpR, desR = orb.detectAndCompute(fr,None)
    exeTime['detectFeatures'].append((time.time()-t_loop)*1000)

    t0 = time.time()
    kpL,desL,pts3D = hf.find_matches_stereo(cam1,cam2,desL,desR,kpL,kpR,matcher)   
    exeTime['stereoMatch'].append((time.time()-t_loop)*1000)

    # On first iteration there is nothing to compute, just move on
    if iteration == 0:
        pts3D_prev = pts3D
        kpL_prev = kpL
        desL_prev = desL
        fl_prev = fl
    else:
        t0=time.time()
        pts_im1,pts_im2,pts3D_prev = hf.find_matches(desL, desL_prev, kpL, kpL_prev,
                                                   pts3D_prev.T, matcher)
        exeTime['matchFeatures'].append((time.time()-t_loop)*1000)
        
        pts2D = pts_im1.T
        pts3D_prev = pts3D_prev.T

        # Run RANSAC in pnp to get a transform between current and previous 
        # time
        t0 = time.time()
        R_est,t_est = sol.getTransform(pts3D_prev, pts2D)
        exeTime['pnp'].append((time.time()-t_loop)*1000)

        # Compute new global pose estimation from the pnp output
        pose_global = hf.combineTransforms(pose_global,R_est,t_est)
        poses.append(pose_global)

        # Save variables for next iteration
        pts3D_prev = pts3D
        kpL_prev = kpL
        desL_prev = desL
        fl_prev = fl
    # Save every pose for evaluation
    
    pose_single_line = np.reshape(pose_global[0:3,:], (12)).tolist()
    pose_single_line.insert(0, iteration)
    pose_single_line = [str(x) for x in pose_single_line]
    pose_single_line = ' '.join(pose_single_line)
    with open("results/"+str(sequence_no)+".txt", "a") as f:
        f.writelines(pose_single_line)
        f.writelines(['\n'])
    iteration+=1
    
    # Show stuff
    t0 = time.time()
    cv2.imshow('frame',fl)
    plt.figure(1)
    showTrajectory()
    exeTime['plot'].append((time.time()-t_loop)*1000)
# =============================================================================
#     if iteration%100==0:
#         plt.pause(0.05)
# =============================================================================
    
    exeTime['loop'].append((time.time()-t_loop)*1000)

    #cv2.imshow('frame_right',fr)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Remove cv windows
cv2.destroyAllWindows()
plt.show()