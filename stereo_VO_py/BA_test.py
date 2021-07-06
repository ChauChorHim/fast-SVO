import solver3Dto2D as solver
import numpy as np
from scipy.spatial.transform import Rotation as rot
from numpy import ones,zeros, eye,squeeze,cos,sin,reshape
from numpy import array as arr
from numpy import concatenate as concat
from numpy import square as pow2

from bundle_adjustment import optimizeBA

#%%

# Make some 3D points to project
N=100
Qs = np.random.uniform(-5,5,(3,N))
Qs[2,:]+= 10
Qs = concat((Qs,ones((1,N))),axis=0)

# 
R1 = (rot.from_euler('xyz',arr([-0.1,0,0]))).as_matrix()
t1 = arr([[1,1,1]]).T
R2 = (rot.from_euler('xyz',arr([0,0,0]))).as_matrix()
t2 = arr([[1,1,2]]).T
R3 = (rot.from_euler('xyz',arr([0.1,0,0]))).as_matrix()
t3 = arr([[1,1,3]]).T


Cam1 = concat((R1,t1),axis=1)
Cam2 = concat((R2,t2),axis=1)
Cam3 = concat((R3,t3),axis=1)

qs1 = Cam1 @ Qs
qs1 = qs1/qs1[2,:]
qs2 = Cam2 @ Qs
qs2 = qs2/qs2[2,:]
qs3 = Cam3 @ Qs
qs3 = qs3/qs3[2,:]

# Add noise
std = 0.005
qs1[0:2] += np.random.normal(scale = std,size=(2,N))
qs2[0:2] += np.random.normal(scale = std,size=(2,N))
qs3[0:2] += np.random.normal(scale = std,size=(2,N))

sol = solver.solver3Dto2D(eye(3),0.99,0.9,True,0.1)
sol.getTransform(Qs,qs1)

R1_est,t1_est=sol.getTransform(Qs,qs1)
R3_est,t2_est=sol.getTransform(Qs,qs2)
R2_est,t3_est=sol.getTransform(Qs,qs3)

Rs = zeros((3,3,3))
ts = zeros((3,3,1))
qs = zeros((3,3,N))

Rs[0,:,:] = R1_est
Rs[1,:,:] = R2_est
Rs[2,:,:] = R3_est

ts[0,:] = t1_est
ts[1,:] = t2_est
ts[2,:] = t3_est

qs[0,:,:] = qs1
qs[1,:,:] = qs2
qs[2,:,:] = qs3


Rs_ba,ts_ba=optimizeBA(eye(3), Rs, ts, Qs, qs,100)


Rs_gt = zeros((3,3,3))
ts_gt = zeros((3,3,1))
Rs_gt[0,:,:] = R1
Rs_gt[1,:,:] = R2
Rs_gt[2,:,:] = R3
ts_gt[0,:] = t1
ts_gt[1,:] = t2
ts_gt[2,:] = t3

print('SSD of ts_gt and ts_ba {}'.format(np.sum((ts_gt-ts_ba)**2)))
print('SSD of Rs_gt and Rs_ba {}'.format(np.sum((Rs_gt-Rs_ba)**2)))
print('SSD of ts_gt and ts_base {}'.format(np.sum((ts_gt-ts)**2)))
print('SSD of Rs_gt and Rs_base {}'.format(np.sum((Rs_gt-Rs)**2)))