# -*- coding: utf-8 -*-

"""
Created on Mon Apr 19 08:50:36 2021
@author: Kamil
"""


import cv2

import numpy as np
from scipy.spatial.transform import Rotation as rot
from numpy import ones,zeros,squeeze,cos,sin,reshape
from numpy import concatenate as concat
from numpy import square as pow2

import matplotlib.pyplot as plt

def jacobian_pt2D(x,Q):
    r,p,y,tx,ty,tz = squeeze(x)
    X,Y,Z = squeeze(Q[0:3])
    Jx = zeros((1,9));
    Jy = zeros((1,9));

    # Precompute trigs
    cosr = cos(r)
    cosp = cos(p)
    cosy = cos(y)
    sinr = sin(r)
    sinp = sin(p)
    siny = sin(y)
    
    
    Jx[0,0] = (Y*(sinr*siny-cosr*cosy*sinp)-Z*(cosr*siny+cosy*sinp*sinr)-ty*(sinr*siny-cosr*cosy*sinp)+tz*(cosr*siny+cosy*sinp*sinr))/(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr)-(Y*cosp*cosr+Z*cosp*sinr-ty*cosp*cosr-tz*cosp*sinr)*(Y*(cosr*siny+cosy*sinp*sinr)+Z*(sinr*siny-cosr*cosy*sinp)-ty*(cosr*siny+cosy*sinp*sinr)-tz*(sinr*siny-cosr*cosy*sinp)+X*cosp*cosy-tx*cosp*cosy)*1.0/pow(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr,2.0)
    Jx[0,1] = (X*cosy*sinp-tx*cosy*sinp+Z*cosp*cosr*cosy-Y*cosp*cosy*sinr-tz*cosp*cosr*cosy+ty*cosp*cosy*sinr)/(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr)+(Y*(cosr*siny+cosy*sinp*sinr)+Z*(sinr*siny-cosr*cosy*sinp)-ty*(cosr*siny+cosy*sinp*sinr)-tz*(sinr*siny-cosr*cosy*sinp)+X*cosp*cosy-tx*cosp*cosy)*1.0/pow(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr,2.0)*(X*cosp-tx*cosp-Z*cosr*sinp+Y*sinp*sinr+tz*cosr*sinp-ty*sinp*sinr)
    Jx[0,2] = -(Y*(cosr*cosy-sinp*sinr*siny)+Z*(cosy*sinr+cosr*sinp*siny)-ty*(cosr*cosy-sinp*sinr*siny)-tz*(cosy*sinr+cosr*sinp*siny)-X*cosp*siny+tx*cosp*siny)/(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr)
    Jx[0,3] = -sinp*(Y*(cosr*siny+cosy*sinp*sinr)+Z*(sinr*siny-cosr*cosy*sinp)-ty*(cosr*siny+cosy*sinp*sinr)-tz*(sinr*siny-cosr*cosy*sinp)+X*cosp*cosy-tx*cosp*cosy)*1.0/pow(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr,2.0)+(cosp*cosy)/(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr)
    Jx[0,4] = (cosr*siny+cosy*sinp*sinr)/(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr)+cosp*sinr*(Y*(cosr*siny+cosy*sinp*sinr)+Z*(sinr*siny-cosr*cosy*sinp)-ty*(cosr*siny+cosy*sinp*sinr)-tz*(sinr*siny-cosr*cosy*sinp)+X*cosp*cosy-tx*cosp*cosy)*1.0/pow(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr,2.0)
    Jx[0,5] = (sinr*siny-cosr*cosy*sinp)/(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr)-cosp*cosr*(Y*(cosr*siny+cosy*sinp*sinr)+Z*(sinr*siny-cosr*cosy*sinp)-ty*(cosr*siny+cosy*sinp*sinr)-tz*(sinr*siny-cosr*cosy*sinp)+X*cosp*cosy-tx*cosp*cosy)*1.0/pow(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr,2.0)
    Jx[0,6] = sinp*(Y*(cosr*siny+cosy*sinp*sinr)+Z*(sinr*siny-cosr*cosy*sinp)-ty*(cosr*siny+cosy*sinp*sinr)-tz*(sinr*siny-cosr*cosy*sinp)+X*cosp*cosy-tx*cosp*cosy)*1.0/pow(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr,2.0)-(cosp*cosy)/(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr)
    Jx[0,7] = -(cosr*siny+cosy*sinp*sinr)/(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr)-cosp*sinr*(Y*(cosr*siny+cosy*sinp*sinr)+Z*(sinr*siny-cosr*cosy*sinp)-ty*(cosr*siny+cosy*sinp*sinr)-tz*(sinr*siny-cosr*cosy*sinp)+X*cosp*cosy-tx*cosp*cosy)*1.0/pow(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr,2.0)
    Jx[0,8] = -(sinr*siny-cosr*cosy*sinp)/(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr)+cosp*cosr*(Y*(cosr*siny+cosy*sinp*sinr)+Z*(sinr*siny-cosr*cosy*sinp)-ty*(cosr*siny+cosy*sinp*sinr)-tz*(sinr*siny-cosr*cosy*sinp)+X*cosp*cosy-tx*cosp*cosy)*1.0/pow(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr,2.0)
 
    Jy[0][0] = (Y*(cosy*sinr+cosr*sinp*siny)-Z*(cosr*cosy-sinp*sinr*siny)-ty*(cosy*sinr+cosr*sinp*siny)+tz*(cosr*cosy-sinp*sinr*siny))/(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr)-(Y*cosp*cosr+Z*cosp*sinr-ty*cosp*cosr-tz*cosp*sinr)*1.0/pow(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr,2.0)*(Y*(cosr*cosy-sinp*sinr*siny)+Z*(cosy*sinr+cosr*sinp*siny)-ty*(cosr*cosy-sinp*sinr*siny)-tz*(cosy*sinr+cosr*sinp*siny)-X*cosp*siny+tx*cosp*siny)
    Jy[0][1] = -(X*sinp*siny-tx*sinp*siny+Z*cosp*cosr*siny-Y*cosp*sinr*siny-tz*cosp*cosr*siny+ty*cosp*sinr*siny)/(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr)+1.0/pow(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr,2.0)*(Y*(cosr*cosy-sinp*sinr*siny)+Z*(cosy*sinr+cosr*sinp*siny)-ty*(cosr*cosy-sinp*sinr*siny)-tz*(cosy*sinr+cosr*sinp*siny)-X*cosp*siny+tx*cosp*siny)*(X*cosp-tx*cosp-Z*cosr*sinp+Y*sinp*sinr+tz*cosr*sinp-ty*sinp*sinr)
    Jy[0][2] = (Y*(cosr*siny+cosy*sinp*sinr)+Z*(sinr*siny-cosr*cosy*sinp)-ty*(cosr*siny+cosy*sinp*sinr)-tz*(sinr*siny-cosr*cosy*sinp)+X*cosp*cosy-tx*cosp*cosy)/(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr)
    Jy[0][3] = -(cosp*siny)/(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr)-sinp*1.0/pow(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr,2.0)*(Y*(cosr*cosy-sinp*sinr*siny)+Z*(cosy*sinr+cosr*sinp*siny)-ty*(cosr*cosy-sinp*sinr*siny)-tz*(cosy*sinr+cosr*sinp*siny)-X*cosp*siny+tx*cosp*siny)
    Jy[0][4] = (cosr*cosy-sinp*sinr*siny)/(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr)+cosp*sinr*1.0/pow(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr,2.0)*(Y*(cosr*cosy-sinp*sinr*siny)+Z*(cosy*sinr+cosr*sinp*siny)-ty*(cosr*cosy-sinp*sinr*siny)-tz*(cosy*sinr+cosr*sinp*siny)-X*cosp*siny+tx*cosp*siny)
    Jy[0][5] = (cosy*sinr+cosr*sinp*siny)/(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr)-cosp*cosr*1.0/pow(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr,2.0)*(Y*(cosr*cosy-sinp*sinr*siny)+Z*(cosy*sinr+cosr*sinp*siny)-ty*(cosr*cosy-sinp*sinr*siny)-tz*(cosy*sinr+cosr*sinp*siny)-X*cosp*siny+tx*cosp*siny)
    Jy[0][6] = (cosp*siny)/(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr)+sinp*1.0/pow(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr,2.0)*(Y*(cosr*cosy-sinp*sinr*siny)+Z*(cosy*sinr+cosr*sinp*siny)-ty*(cosr*cosy-sinp*sinr*siny)-tz*(cosy*sinr+cosr*sinp*siny)-X*cosp*siny+tx*cosp*siny)
    Jy[0][7] = -(cosr*cosy-sinp*sinr*siny)/(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr)-cosp*sinr*1.0/pow(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr,2.0)*(Y*(cosr*cosy-sinp*sinr*siny)+Z*(cosy*sinr+cosr*sinp*siny)-ty*(cosr*cosy-sinp*sinr*siny)-tz*(cosy*sinr+cosr*sinp*siny)-X*cosp*siny+tx*cosp*siny)
    Jy[0][8] = -(cosy*sinr+cosr*sinp*siny)/(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr)+cosp*cosr*1.0/pow(X*sinp-tx*sinp+Z*cosp*cosr-Y*cosp*sinr-tz*cosp*cosr+ty*cosp*sinr,2.0)*(Y*(cosr*cosy-sinp*sinr*siny)+Z*(cosy*sinr+cosr*sinp*siny)-ty*(cosr*cosy-sinp*sinr*siny)-tz*(cosy*sinr+cosr*sinp*siny)-X*cosp*siny+tx*cosp*siny)

    return Jx,Jy

def JACOBIAN(Rs,ts,Qs,qs):
    
    assert(len(Rs.shape)==3)
    assert(len(ts.shape)==3)
    assert(len(qs.shape)==3)
    assert(len(Qs.shape)==2)
    assert(Qs.shape[1]==qs.shape[2])
    assert(Rs.shape[0]==qs.shape[0]==ts.shape[0])
        
    num_pts = Qs.shape[1]
    num_poses = qs.shape[0]
    
    # Preallocate 
    J = zeros((2*num_pts*num_poses,6*num_poses+3*num_pts))
    d = zeros((num_poses*num_pts*2))
    
    num_pts = Qs.shape[1]
    num_poses = qs.shape[0]
    
    for pose_idx in range(num_poses):
        
        rot_mat=rot.from_matrix(Rs[pose_idx])
        # Get rpy and stack into column vector
        rpy = reshape(rot_mat.as_euler('xyz',degrees=False),(3,1))
        x = concat((rpy,ts[pose_idx]),axis=0)
        for pt_idx in range(num_pts):
            
            Jx,Jy = jacobian_pt2D(x,Qs[:,pt_idx])
            dx_pose = Jx[0,0:6]
            dy_pose = Jy[0,0:6]
            dx_pt3D = Jx[0,6:9]
            dy_pt3D = Jy[0,6:9]
            
            # Fill in the derivative of R P Y tx ty tz
            J[pose_idx*num_pts*2+pt_idx*2  ,pose_idx*6:(pose_idx+1)*6]  = dx_pose
            J[pose_idx*num_pts*2+pt_idx*2+1,pose_idx*6:(pose_idx+1)*6]  = dy_pose
            # Fill in the derivative of X Y Z
            J[pose_idx*num_pts*2+pt_idx*2  ,num_poses*6+ pt_idx*3:num_poses*6+(pt_idx+1)*3] = dx_pt3D
            J[pose_idx*num_pts*2+pt_idx*2+1,num_poses*6+ pt_idx*3:num_poses*6+(pt_idx+1)*3] = dy_pt3D
            
            # Project the point and save the difference between projected and observed
            uvs = concat((Rs[pose_idx].T,-Rs[pose_idx].T@ts[pose_idx]),axis=1) @ Qs[:,pt_idx]
            up = uvs[0]/uvs[2]
            vp = uvs[1]/uvs[2]
            d[pose_idx*2*num_pts+pt_idx*2  ] = qs[pose_idx,0,pt_idx]-up
            d[pose_idx*2*num_pts+pt_idx*2+1] = qs[pose_idx,1,pt_idx]-vp
            
            if np.isnan(qs[pose_idx,0,pt_idx]-up):
                assert(1==0)
                print(qs[pose_idx,0,pt_idx])
                print(uvs)
                print(Qs[:,pt_idx])
                print(pt_idx)
                print(Rs[pose_idx])
                print(ts[pose_idx])
                print(pose_idx)

    return J,d

def getError(Rs,ts,Qs,qs):
    
    assert(len(Rs.shape)==3)
    assert(len(ts.shape)==3)
    assert(len(qs.shape)==3)
    assert(len(Qs.shape)==2)
    assert(Qs.shape[1]==qs.shape[2])
    assert(Rs.shape[0]==qs.shape[0]==ts.shape[0])

    num_pts = Qs.shape[1]
    num_poses = qs.shape[0]
    
    # Preallocate 
    d = zeros((num_poses*num_pts*2))
    
    num_pts = Qs.shape[1]
    num_poses = qs.shape[0]
    
    for pose_idx in range(num_poses):
        
        for pt_idx in range(num_pts):
                        
            # Project the point and save the difference between projected and observed
            uvs = concat((Rs[pose_idx].T,-Rs[pose_idx].T@ts[pose_idx]),axis=1) @ Qs[:,pt_idx]
            up = uvs[0]/uvs[2]
            vp = uvs[1]/uvs[2]
            d[pose_idx*2*num_pts+pt_idx*2  ] = qs[pose_idx,0,pt_idx]-up
            d[pose_idx*2*num_pts+pt_idx*2+1] = qs[pose_idx,1,pt_idx]-vp



    return d.T@d

def x2pose(x,num_poses):
    Rs = zeros((num_poses,3,3))
    ts = zeros((num_poses,3,1))
    
    for pose_idx in range(num_poses):
        rpyt = x[pose_idx*6:(pose_idx+1)*6]
        R = (rot.from_euler('xyz',rpyt[0:3].T)).as_matrix()
        t = rpyt[3:6]
        Rs[pose_idx] = R
        ts[pose_idx] = t
    
    return Rs,ts
    
def x2Qs(x,num_poses,num_pts):
    
    Qs = reshape(x[num_poses*6:],(3,num_pts),order='F')
    Qs = concat((Qs,ones((1,num_pts))),axis=0)
    return Qs
    
def pose2x(Rs,ts,Qs):
    
    num_poses = Rs.shape[0]
    num_pts = Qs.shape[1]
    
    x = zeros((num_poses*6+num_pts*3,1))
    for pose_idx in range(num_poses):
        rot_mat=rot.from_matrix(Rs[pose_idx])
        rpy = reshape(rot_mat.as_euler('xyz',degrees=False),(3,1))
        
        x[pose_idx*6:(pose_idx+1)*6] = concat((rpy,ts[pose_idx]),axis=0)
    
    # Append Qs
    x[num_poses*6:] = reshape(Qs[0:3,:],(num_pts*3,1),order='F')
    
    return x

def optimizeBA(K,Rs,ts,Qs,qs,n_iter=100,tolerance=1e-9,Lambda=0.01):
    
    num_poses = Rs.shape[0]
    num_pts = Qs.shape[1]
    
    # Convert Qs to homogeneous if that is not the case
    if Qs.shape[0] == 3:
        Qs = concat((Qs,np.ones((1,Qs.shape[1]))),axis=0)
        
        
    if qs.shape[1] == 2:
        qs = concat((qs,ones((num_poses,1,num_pts))),axis=1)
    
    # remove the K from further calculations
    for i in range(num_poses):
        qs[i,:,:] = np.linalg.inv(K)@qs[i,:,:]
        qs[i,:,:] = qs[i,:,:]/qs[i,2,:]
        
    # Make the 
    x = pose2x(Rs,ts,Qs)

    updateJ = True
    for iteration in range(n_iter):
        
        if updateJ:
            
            J,d = JACOBIAN(Rs,ts,Qs,qs)     
            
             
            
            H = J.T @ J
            e = d.T @ d
        # End updateJ



        H_lm = H + np.eye(H.shape[0]) * Lambda
        dx = -np.linalg.inv(H_lm)@ J.T @ d
        dx = reshape(dx,(dx.shape[0],1))        
        
        # Update temporary parameters
        x_lm = x + dx
        
        Rs_lm,ts_lm = x2pose(x_lm,num_poses)
        Qs_lm =x2Qs(x_lm,num_poses,num_pts)
        
        e_lm = getError(Rs_lm,ts_lm,Qs_lm,qs)
        
        if e_lm < e:
            Lambda /= 10
            updateJ = True
            x = x_lm
        else:
            updateJ = False
            Lambda *= 10

        if np.linalg.norm(dx)<tolerance:
            break        
        
        Rs,ts=x2pose(x,Rs.shape[0])

    return Rs,ts


def featureTracking(img_prev,img, qs_prev):
    """
    Use OpenCV to find the prev_points from the prev_img in the next_img
    Remember to remove points that could not be found from prev_points, next_points, and world_points
    hint: status == 1
    """
    params = dict(winSize=(21, 21),
                 maxLevel=3,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    
    qs,status,_ = cv2.calcOpticalFlowPyrLK(img_prev,img,qs_prev,None,None,None,**params)
    status = np.array(status.ravel(), dtype=bool)
    return qs,status

class BundleAdjustment:
    
    
    def __init__(self,Qs,qs,img,R,t,K,window_size,min_pts,max_pts=None):
        
        self.Qs=Qs
        
        assert(abs(np.linalg.det(R)-1)<1e-9)
        
        self.K=K
        self.min_pts=min_pts
        self.max_pts=max_pts
        
        
        self.Rs = np.zeros((window_size,3,3))
        self.ts = np.zeros((window_size,3,1))
        self.Rs[0,:,:]=R
        self.ts[0,:,:]=t
        self.qs_prev = np.zeros((qs.shape[0],qs.shape[1],window_size),dtype=np.float32)
        self.qs_prev[:,:,0]=qs
        
        
        self.img_prev = img
        self.iter = 0
        
    
    # Initialize with a set of 3D and 2D points, current pose, picture 
    
    
    # get a picture and estimate of the current pose
    # Do optical flow of the points
    def update(self,img,R,t):
        
        assert(abs(np.linalg.det(R)-1)<1e-9)
        
        # Track features
        qs,status = featureTracking(self.img_prev,img,self.qs_prev[:,:,self.iter])
                
        self.Qs = self.Qs[status]
        self.qs_prev = self.qs_prev[status,:,:]       
        
        
        self.iter+=1
        
        self.qs_prev[:,:,self.iter] = qs[status]
        
        # Check for possible outliers
        qs_ = self.K@(R.T@self.Qs.T-R.T@t)
        not_neg = qs_[2,:]>0
        qs_=qs_/qs_[2,:]
        

        dist = np.linalg.norm(qs_[:2,:]-qs[status].T,axis=0)
        inliers_mask = (dist<20) #& (qs_[0,:]>0) & (qs_[1,:]>0) & not_neg

        self.Qs=self.Qs[inliers_mask]
        self.qs_prev=self.qs_prev[inliers_mask]
        
        self.Rs[self.iter,:,:]=R
        self.ts[self.iter,:,:]=t
        self.img_prev=img
        #print("BA num point: {}".format(self.Qs.shape[0]))
        
        return self.Qs.shape[0]<self.min_pts
    
    # Compute the BA
    def compute(self):
        
        qs = np.swapaxes(self.qs_prev,0,2)
        
        if self.max_pts is not None:
            # Limit the number of points
            Qs = self.Qs[:self.max_pts].T
            qs=qs[:,:,:self.max_pts]
        else:
            Qs = self.Qs.T
            
        Rs,ts = optimizeBA(self.K,self.Rs[:self.iter+1],
                           self.ts[:self.iter+1],Qs,qs[:self.iter+1,:,:],
                           n_iter=50,tolerance=1e-6,Lambda=0.01)
        return Rs,ts
    
    
