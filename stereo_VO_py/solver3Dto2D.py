#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar 27 17:42:17 2021
@author: kamil
"""
import time
import numpy as np
from numpy import sin, cos
from numpy import square as pow2

from scipy.spatial.transform import Rotation

from pnp import p3p, getUniqueSolution

np.random.seed(0)

def P3Pransac(K,pts3D,ptsImg,num_iter=10,epsilon=None):
    # K:        camera intrinsics matrix
    # pts3D:    homogeneous 3D points [x y z 1]'.
    # ptsImg:   image points [pixels x, pixels y]'
    n_best = 0
    R_best = np.eye(3)
    t_best = np.zeros((3,1))
    inliers=[]

    # Default epsilon = Horizontal Resolution / 100
    if epsilon == None:
        if K[0,2] == 0:
            raise Exception("Cannot assign default epsilon K[0,2]==0! \
                            Define a proper K or assign epsilon yourself")
        epsilon = K[0,2]*0.02

    # Convert to homogeneous if needed
    N = pts3D.shape[1]
    if pts3D.shape[0] == 3:
        pts3D = np.concatenate((pts3D,np.ones([1,N])),axis=0)


    # convert to homogeneous if needed
    if ptsImg.shape[0]==2:
        ptsImg = np.concatenate((ptsImg,np.ones([1,N])),axis=0)


    # Get 2D points in camera frame
    pts2D = np.linalg.inv(K)@ptsImg
    pts2D = pts2D/pts2D[2,:]

    for i in range(num_iter):

        # get 4 random points. 3 for p3p and one extra to choose the correct
        # solution
        rand_idx = np.random.choice(N,4,replace=False)


        # Normalize the 2D points before passing it to p3p
        pts2D_norm = pts2D/np.linalg.norm(pts2D, axis = 0)
        poses = p3p(pts3D[0:3,rand_idx[0:3]], pts2D_norm[:,rand_idx[0:3]])

        # get unique solution using 4th point
        R_estim, t_estim, valid_estim = getUniqueSolution(poses,pts3D[0:3,rand_idx[0:3]],pts3D[0:3,3],pts2D[0:2,3])

        # if the solution is valid, check for the number of inliers, and save/discard the solution
        if valid_estim:
            # Project 3d points using the computed transform
            t_estim = t_estim.reshape([3,1])
            ptsImg_hat = K @ np.concatenate((R_estim,t_estim),axis=1) @ pts3D
            ptsImg_hat = ptsImg_hat/ptsImg_hat[2,:]

            # Check consensus
            dist=np.sum( np.square(ptsImg-ptsImg_hat),axis=0)
            inliers_idx = np.where(dist<epsilon)[0]
            n_inliers = len(inliers_idx)

            # Save best result
            if n_inliers > n_best:
                R_best = R_estim
                t_best = t_estim
                n_best = n_inliers
                inliers = inliers_idx

    return R_best, t_best, inliers



def jacobian_lm(x,Q):
    r,p,y,tx,ty,tz = np.squeeze(x)
    X,Y,Z = np.squeeze(Q[0:3])
    Jx = np.zeros((1,6));
    Jy = np.zeros((1,6));
    
    # Precompute trigs
    cosr = cos(r)
    sinr = sin(r)
    sinp = sin(p)
    cosp = cos(p)
    siny = sin(y)
    cosy = cos(y)
    
    # Precompute common vars
    var2 = cosy*sinr+cosr*sinp*siny
    var4 = sinr*siny-cosr*cosy*sinp
    var1 = pow2(tz+X*(var4)+Y*(var2)+Z*cosp*cosr)
    var3 = cosr*siny+cosy*sinp*sinr
    var5 = cosr*cosy-sinp*sinr*siny
    var6 = tx+Z*sinp+X*cosp*cosy-Y*cosp*siny
    var7 = Z*cosr*sinp+X*cosp*cosr*cosy-Y*cosp*cosr*siny
    

    Jx[0,0] = (X*(var3)+Y*(var5)-Z*cosp*sinr)*(var6 )*1.0/var1;
    Jx[0,1] = -(Z*cosp-X*cosy*sinp+Y*sinp*siny)/(var1)-(var7)*(var6 )*1.0/var1;
    Jx[0,2] = (Y*cosp*cosy+X*cosp*siny)/(var1)+(X*(var2)-Y*(var4))*(var6 )*1.0/var1;
    Jx[0,3] = -1.0/(var1);
    Jx[0,5] = (var6 )*1.0/var1;

    Jy[0,0] = (X*(var4)+Y*(var2)+Z*cosp*cosr)/(var1)+(X*(var3)+Y*(var5)-Z*cosp*sinr)*1.0/var1*(ty+X*(var3)+Y*(var5)-Z*cosp*sinr);
    Jy[0,1] = -(Z*sinp*sinr+X*cosp*cosy*sinr-Y*cosp*sinr*siny)/(var1)-(var7)*1.0/var1*(ty+X*(var3)+Y*(var5)-Z*cosp*sinr);
    Jy[0,2] = -(X*(var5)-Y*(var3))/(var1)+(X*(var2)-Y*(var4))*1.0/var1*(ty+X*(var3)+Y*(var5)-Z*cosp*sinr);
    Jy[0,4] = -1.0/(var1);
    Jy[0,5] = 1.0/var1*(ty+X*(var3)+Y*(var5)-Z*cosp*sinr);

    return Jx,Jy

def optimizeLM(K,R_init,t_init,Qs,qs,n_iter=100,tolerance=1e-9,Lambda=0.01):
    
    # Convert Qs to homogeneous if that is not the case
    if Qs.shape[0] == 3:
        Qs = np.concatenate((Qs,np.ones((1,Qs.shape[1]))),axis=0)
        
    if qs.shape[0] == 2:
        qs = np.concatenate((qs,np.ones((1,qs.shape[1]))),axis=0)
    
    # remove the K from further calculations
    qs = np.linalg.inv(K)@qs
    qs=qs/qs[2,:]
    
    #qs_inh = qs[0:2,:] # Inhomogeneous qs
    qs_inh_stack = np.reshape(qs[0:2,:],(qs.shape[1]*2,1),order='F')
    
    n_param = 6
    n_data = Qs.shape[1]
    n_data2 = n_data*2


    rot_mat=Rotation.from_matrix(R_init)
    # Get rpy and stack into column vector
    rpy = np.reshape(rot_mat.as_euler('xyz',degrees=False),(3,1))
    # Vector of variables to be optimized, roll,pitch,yaw,tx,ty,tz
    x = np.concatenate((rpy,t_init),axis=0)
    
    updateJ = True
    for iteration in range(n_iter):
        
        if updateJ:
            R = Rotation.from_euler('xyz',x[0:3].T)
            R = R.as_matrix()[0]
            A = np.concatenate((R,x[3:6]),axis=1) # Camera extrinsics
            
            J = np.zeros((n_data2,n_param))
            #d = np.zeros((n_data2,1))
            
            for i in range(0,n_data,1):

                Jx,Jy = jacobian_lm(x,Qs[:,i])
                J[2*i,:] = Jx
                J[2*i+1,:] = Jy
                """
                uvs = A @ Qs[:,i]
                up = uvs[0]/uvs[2]
                vp = uvs[1]/uvs[2]
                
                d[2*i] = qs[0,i]-up
                d[2*i+1] = qs[1,i]-vp
                """
            # Vectorized implementation
            uvs = A@ Qs
            uvs = uvs/uvs[2,:]
            uvs = uvs[0:2,:] # Convert to inhomogeneous
            uvs_stack = np.reshape(uvs,(uvs.shape[1]*2,1),order='F')
            d = qs_inh_stack - uvs_stack       
                
    
            H = J.T @ J
            e = d.T @ d
        # End updateJ
        
        # Add the damping factor to hessian
        H_lm = H + np.eye(n_param) * Lambda
        
        # Compute the gradient 
        dx = -np.linalg.inv(H_lm)@ J.T @ d
        
        # Update temporary parameters
        x_lm = x + dx
        
        
        # Check if the updated parameters lead to smaller error
        R = Rotation.from_euler('xyz',x_lm[0:3].T)
        R = R.as_matrix()[0]
        A = np.concatenate((R,x_lm[3:6]),axis=1) # Camera extrinsics
                
        # Vectorized implementation
        uvs = A@ Qs
        uvs = uvs/uvs[2,:]
        uvs = uvs[0:2,:] # Convert to inhomogeneous
        uvs_stack = np.reshape(uvs,(uvs.shape[1]*2,1),order='F')
        d_lm = qs_inh_stack - uvs_stack
        """
        d_lm = np.zeros((n_data2,1))
        for i in range(0,n_data,1):
         
            uvs = A @ Qs[:,i]
            up = uvs[0]/uvs[2]
            vp = uvs[1]/uvs[2]
            
            d_lm[2*i] = qs[0,i]-up
            d_lm[2*i+1] = qs[1,i]-vp
        """
        e_lm = d_lm.T @ d_lm
        
        if e_lm < e:
            Lambda /= 10
            updateJ = True
            x = x_lm
        else:
            updateJ = False
            Lambda *= 10
            
            
        if np.linalg.norm(dx)<tolerance:
            break
            
    # Return R and t
    R = Rotation.from_euler('xyz',x[0:3].T)
    R = R.as_matrix()[0]
    return R, x[3:6], iteration
    

class solver3Dto2D:

    def __init__(self, K, sol_conf,inlier_prob,nonLinearOpt = True,epsilon=None):
        """
        Parameters
        ----------
        K : 3x3 numpy array
            Intrinsic camera parameters.
        sol_conf : float
            Desired RANSAC solution confidence between 0 and 1.
        inlier_prob : float
            Estimated probability of a data being an inlier. between 0 and 1
        """
        self.K = K
        self.sol_conf = sol_conf
        self.inlier_prob = inlier_prob

        self.RANSAC_num_iter = int(np.ceil(np.log(1-self.sol_conf)/(np.log(1-self.inlier_prob**4))))

        self.nonLinearOpt = nonLinearOpt
        self.epsilon_ransac = epsilon

    def getTransform(self,pts3D,ptsImg):

        # Run RANSAC
        R_estim,t_estim,inliers = P3Pransac(self.K,pts3D,ptsImg,self.RANSAC_num_iter,self.epsilon_ransac)
        # Adjust the number of iteration based on the number of observed inliers?

        
        if self.nonLinearOpt:
            #t0=time.time()
            R_estim,t_estim,num_iter = optimizeLM(self.K,R_estim,t_estim,pts3D[:,inliers],
                                         ptsImg[:,inliers])
            #print('LM exe time {}'.format(time.time()-t0))
        return R_estim, t_estim,inliers
