#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import numpy as np
import cv2

def combineTransforms(RTg,Rr,tr):
    """ RTg: global [R t; 0,0,0,1]
        Rr: relative rotation
        tr: relative translation
    """

    # Change the transfrom from 3Dto2D and the other way around
    RTr = np.concatenate((np.transpose(Rr),-np.transpose(Rr)@tr),axis=1)
    RTr = np.concatenate((RTr,np.array([[0,0,0,1]])),axis=0)

    return RTg@RTr

def find_matches(des1, des2, kp1, kp2, pts3D, matcher, thres=0.6):
    
    matches = matcher.knnMatch(des1,des2,k=2)
    # Apply ratio test
    good_matches = []
    
    for m,n in matches:
        if m.distance/(n.distance+10e-10) < thres:
            good_matches.append([m])
    
    # Find coordinates
    pts_im1 = [kp1[m[0].queryIdx].pt for m in good_matches]
    
    pts_im1 = np.array(pts_im1, dtype=np.float32)
    
    pts_im2 = [kp2[m[0].trainIdx].pt for m in good_matches]
    pts_im2 = np.array(pts_im2, dtype=np.float32)
    
    pts_im1_3D = [pts3D[m[0].trainIdx] for m in good_matches]
    pts_im1_3D = np.array(pts_im1_3D, dtype=np.float32)
    
    return pts_im1, pts_im2, pts_im1_3D

def find_matches_stereo(P1,P2,des1, des2, kp1, kp2, matcher, thres=0.6 ):

    matches = matcher.knnMatch(des1,des2,k=2)
    # Apply ratio test
    good_matches = []
    
    for m,n in matches:
        if m.distance/(n.distance+10e-10) < thres:
            good_matches.append((m,n))
    
    kp1_ret =[kp1[m[0].queryIdx] for m in good_matches]
    des1_ret = [des1[m[0].queryIdx] for m in good_matches]
    des1_ret = np.array(des1_ret)
    
    kp2_ret =[kp2[m[0].trainIdx] for m in good_matches]
    
    kp1_ep,des1_ep,kp2_ep = [],[],[]
    for i in range(len(kp1_ret)):
        if kp1_ret[i].pt[1]==kp2_ret[i].pt[1]:
            kp1_ep.append(kp1_ret[i])
            kp2_ep.append(kp2_ret[i])
            des1_ep.append(des1_ret[i])
            

    pts_im1_epipolar = np.array([point.pt for point in kp1_ep])
    pts_im2_epipolar = np.array([point.pt for point in kp2_ep])
        
    #print(pts_im1_epipolar - pts_im2_epipolar)
    Q = cv2.triangulatePoints(P1, P2, pts_im1_epipolar.T, pts_im2_epipolar.T)  # 3D points
    Q /= Q[-1] # homogeneous
    
    return kp1_ep,np.array(des1_ep),Q[0:3,:]



