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
from solver3Dto2D import optimizeLM
import helperFunctions as hf
import bundle_adjustment as ba


MAP = []

def showTrajectoryZoom():
    xs_est = np.array(poses_base)[:,2,3]
    ys_est = -np.array(poses_base)[:,0,3]
    xs_est_ba = np.array(poses_ba)[:,2,3]
    ys_est_ba = -np.array(poses_ba)[:,0,3]
    xs_gt = np.array(gts)[:,2,3]
    ys_gt = -np.array(gts)[:,0,3]
    # Plot
    

    
    try:
        plt.scatter(MAP[:,2],-MAP[:,0],s=0.1,color='black')
    except:
        pass   
    
    try:
        Qs = BA.Qs
        plt.scatter(Qs[:,2],-Qs[:,0],s=10,color='green')
    except:
        pass
    
    size=30
    plt.plot(xs_gt, ys_gt, '.', color='black');
    plt.plot(xs_est, ys_est, '.', color='red');
    plt.plot(xs_est_ba, ys_est_ba, '.', color='blue');
    plt.gca().set_aspect('equal', adjustable='datalim')
    plt.xlim(xs_est_ba[-1]-size,xs_est_ba[-1]+size)
    plt.ylim(ys_est_ba[-1]-size,ys_est_ba[-1]+size)
    
    
def plotNiceStuff(img):
    
    xs_est = np.array(poses_base)[:,2,3]
    ys_est = -np.array(poses_base)[:,0,3]
    xs_est_ba = np.array(poses_ba)[:,2,3]
    ys_est_ba = -np.array(poses_ba)[:,0,3]
    xs_gt = np.array(gts)[:,2,3]
    ys_gt = -np.array(gts)[:,0,3]
    # Plot
    
    try:
        Qs = BA.Qs
        plt.plot(Qs[:,2],-Qs[:,0],',',color='green')
    except:
        pass
    size=30
    
    fig, ax = plt.subplots(1,2)

    
    ax[0].plot(xs_gt, ys_gt, 'o', color='black');
    ax[0].plot(xs_est, ys_est, '.', color='red');
    ax[0].plot(xs_est_ba, ys_est_ba, '.', color='blue');
    #ax[0].gca().set_aspect('equal', adjustable='datalim')
    #ax[0].xlim(xs_gt[-1]-size,xs_gt[-1]+size)
    #ax[0].ylim(ys_gt[-1]-size,ys_gt[-1]+size)
    ax[0].set_xlim(xs_gt[-1]-size,xs_gt[-1]+size)
    ax[0].set_ylim(ys_gt[-1]-size,ys_gt[-1]+size)
    
    ax[1].imshow(img,cmap = 'gray')
    
def showTrajectory():
    xs_est = np.array(poses_base)[:,2,3]
    ys_est = -np.array(poses_base)[:,0,3]
    xs_est_ba = np.array(poses_ba)[:,2,3]
    ys_est_ba = -np.array(poses_ba)[:,0,3]
    xs_gt = np.array(gts)[:,2,3]
    ys_gt = -np.array(gts)[:,0,3]
    # Plot
    plt.plot(xs_gt, ys_gt, '.', color='black');
    plt.plot(xs_est, ys_est, '.', color='red');
    plt.plot(xs_est_ba, ys_est_ba, '.', color='blue');
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
    
    

def createMap(imgL,imgR,cam2,R,t):
    
    f = cam2[0,3]/1000
    
    Q = np.array([[1,0,0,-cam2[0,2]],
                  [0,1,0,-cam2[1,2]],
                  [0,0,0,cam2[0,0]],
                  [0,0, -1/f, (cam2[0,2]-cam2[0,2])/f]])

    stereo = cv2.StereoSGBM_create(minDisparity = 4,
                                   numDisparities=128,
                                   blockSize=7,
                                   speckleWindowSize = 200,
                                   speckleRange=2,
                                   disp12MaxDiff=2,
                                   uniquenessRatio = 15,
                                   P1 = 32,
                                   P2 = 128,
                                   )
    disparity = stereo.compute(imgL,imgR)

    disparity = np.array(disparity/16,dtype=np.float32)
    pts3D = cv2.reprojectImageTo3D(disparity,Q)

    pts3D = pts3D.reshape(disparity.shape[0]*disparity.shape[1],3)
    pts3D = pts3D[abs(pts3D[:,2])<20,:]

    
    pts3D = pts3D[(pts3D[:,1]<1) & (pts3D[:,1]>-2),:]
    
    pts3D = R@pts3D.T+t
    
    
    return pts3D
    

    
# Dataset parameters
dataset_path = 'KITTI Dataset\\data_odometry_gray\\dataset\\'
dataset_path = 'C:\\Users\\zcq\\Downloads\\data_odometry_gray\\dataset\\'
sequence_no = '09'

# Class that fetches the data
dataset=KITTIDatasetWIN.KITTIDatasetWIN(dataset_path,sequence_no)
cam1,cam2=dataset.getCameraMatrix()

sol = solver.solver3Dto2D(cam1[0:3,0:3],0.99,0.7,nonLinearOpt=False)


pose_global_base = np.eye(3,4)
poses_base = [pose_global_base]       # History of estimated poses


pose_global_lm = np.eye(3,4)
poses_lm = [pose_global_lm]

gts = []                    # History of ground truth poses

matcher = cv2.BFMatcher(cv2.NORM_HAMMING)

iteration = 0

exeTime = {'data':[],'detectFeatures':[],'stereoMatch':[],
           'matchFeatures':[],'pnp':[],'plot':[],'loop':[]}

# For bundle adjustment
iterationBA = 0
BA = []
# Window size 
nBA = 50
# BA poses
pose_global_ba = np.eye(3,4)
poses_ba = [pose_global_ba]

tot_dist = 0

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
    
    # REmove far away
    #close_idx = pts3D[2,:]<50
    #kpL_new = []
    #for i in range(len(close_idx)):
    #    if close_idx[i]:
    #        kpL_new.append(kpL[i])
    #kpL=kpL_new
    #desL = desL[close_idx,:]
    #pts3D = pts3D[:,close_idx]
    
    successBA = 0
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
        
        R_est_base,t_est_base,inliers = sol.getTransform(pts3D_prev, pts2D)
        R_est_lm,t_est_lm,_ = optimizeLM(cam1[:3,:3], R_est_base,
                                       t_est_base,pts3D_prev[:,inliers],pts2D[:,inliers])
        
        exeTime['pnp'].append((time.time()-t_loop)*1000)
        
        # Compute new global pose estimation from the pnp output
        pose_global_base = hf.combineTransforms(pose_global_base,R_est_base,t_est_base)
        pose_global_lm = hf.combineTransforms(pose_global_lm,R_est_lm,t_est_lm)
        pose_global_ba = hf.combineTransforms(pose_global_ba,R_est_lm,t_est_lm)
        poses_ba.append(pose_global_ba[:3,:4])
        poses_lm.append(pose_global_lm[:3,:4])
        poses_base.append(pose_global_base[:3,:4])
        
        pose_single_line = np.reshape(pose_global_base[0:3,:], (12)).tolist()
        pose_single_line.insert(0, iteration)
        pose_single_line = [str(x) for x in pose_single_line]
        pose_single_line = ' '.join(pose_single_line)
        with open("results/base_"+str(sequence_no)+".txt", "a") as f:
            f.writelines(pose_single_line)
            f.writelines(['\n'])
            
        pose_single_line = np.reshape(pose_global_lm[0:3,:], (12)).tolist()
        pose_single_line.insert(0, iteration)
        pose_single_line = [str(x) for x in pose_single_line]
        pose_single_line = ' '.join(pose_single_line)
        with open("results/lm_"+str(sequence_no)+".txt", "a") as f:
            f.writelines(pose_single_line)
            f.writelines(['\n'])
            
        pose_single_line = np.reshape(pose_global_ba[0:3,:], (12)).tolist()
        pose_single_line.insert(0, iteration)
        pose_single_line = [str(x) for x in pose_single_line]
        pose_single_line = ' '.join(pose_single_line)
        with open("results/ba_"+str(sequence_no)+".txt", "a") as f:
            f.writelines(pose_single_line)
            f.writelines(['\n'])
                
        if iterationBA==0:
            qs = []
            for p in kpL:
                qs.append([p.pt[0],p.pt[1]])
            qs=np.array(qs)
            Qs = pose_global_ba[:3,:3] @ pts3D + pose_global_ba[:3,3:4]
            BA = ba.BundleAdjustment(Qs.T,qs,fl, pose_global_ba[:3,:3],
                                pose_global_ba[:3,3:4], cam1[:3,:3], nBA, 80,400)
            iterationBA+=1
        elif iterationBA>=nBA-1:
            low_pts = BA.update(fl,pose_global_ba[:3,:3],pose_global_ba[:3,3:4])
            Rs_ba,ts_ba=BA.compute()
            
            # Remove VO poses and replace with BA
            for i in range (Rs_ba.shape[0]):
                poses_ba.pop(-1)
            for i in range (Rs_ba.shape[0]):
                poses_ba.append(np.c_[Rs_ba[i,:,:], ts_ba[i,:]])
            pose_global_ba = np.c_[Rs_ba[-1,:,:], ts_ba[-1,:]]
            iterationBA=0
            #break
        else:
            low_pts=BA.update(fl,pose_global_ba[:3,:3],pose_global_ba[:3,3:4])
            iterationBA+=1
            if low_pts:
                print('low pts')
                Rs_ba,ts_ba=BA.compute() 
                # Remove VO poses and replace with BA
                for i in range (Rs_ba.shape[0]):
                    poses_ba.pop(-1)
                for i in range (Rs_ba.shape[0]):
                    poses_ba.append(np.c_[Rs_ba[i,:,:], ts_ba[i,:]])
                pose_global_ba = np.c_[Rs_ba[-1,:,:], ts_ba[-1,:]]
                iterationBA=0
                
            
        # Save variables for next iteration
        pts3D_prev = pts3D
        kpL_prev = kpL
        desL_prev = desL
        fl_prev = fl


    iteration+=1
    
    # Show stuff
    t0 = time.time()
    try:
        img_disp = cv2.cvtColor(fl.copy(), cv2.COLOR_GRAY2BGR)
        qs = BA.qs_prev[:,:,BA.iter]
        for i in range(qs.shape[0]):
            cv2.circle(img_disp,(int(qs[i,0]), int(qs[i,1])),2,(0,255,0),-1)
            
        speed=np.linalg.norm(poses_ba[-1][:3,3]-poses_ba[-2][:3,3])
        speed_str = 'Speed (km/h): '+str(int(speed*36))
        cv2.putText(img_disp,speed_str,(30,330),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,(50,250,50),2)
        tot_dist+=speed
        dist_str = 'Total dist (km) : '+str(np.round(tot_dist/1000,2))
        cv2.putText(img_disp,dist_str,(30,360),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,(50,250,50),2)
            
    except:
        pass
    #MAP=createMap(fl,fr,cam2,pose_global_ba[:3,:3],pose_global_ba[:3,3:4])
    #MAP=MAP.T
    showTrajectoryZoom()
    #plotNiceStuff(img_disp)
    #plt.pause(0.01)
    
    
    #cv2.resizeWindow('left Camera',img_disp.shape[0]//2,img_disp.shape[1]//2)
    cv2.imshow('Left Camera',cv2.resize(img_disp,(950,290)))
    
    
    
    
    
    exeTime['plot'].append((time.time()-t_loop)*1000)
    
    exeTime['loop'].append((time.time()-t_loop)*1000)

    #cv2.imshow('frame_right',fr)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Remove cv windows
cv2.destroyAllWindows()
plt.show()




#%%

xs_vo = np.array(poses_base)[:,2,3]
ys_vo = -np.array(poses_base)[:,0,3]
xs_ba = np.array(poses_ba)[:,2,3]
ys_ba = -np.array(poses_ba)[:,0,3]
xs_gt = np.array(gts)[:,2,3]
ys_gt = -np.array(gts)[:,0,3]

dxs_gt = np.zeros(xs_gt.shape)
dys_gt = np.zeros(ys_gt.shape)
dxs_ba = np.zeros(xs_ba.shape)
dys_ba = np.zeros(ys_ba.shape)
dxs_vo = np.zeros(xs_vo.shape)
dys_vo = np.zeros(ys_vo.shape)
for i in range(1,xs_gt.shape[0]):
    dxs_gt[i] = xs_gt[i]-xs_gt[i-1]
    dys_gt[i] = ys_gt[i]-ys_gt[i-1]
    dxs_ba[i] = xs_ba[i]-xs_ba[i-1]
    dys_ba[i] = ys_ba[i]-ys_ba[i-1]
    dxs_vo[i] = xs_vo[i]-xs_vo[i-1]
    dys_vo[i] = ys_vo[i]-ys_vo[i-1]

plt.plot(dys_vo)
plt.plot(dys_ba)
plt.plot(dys_gt)

#%%
fig, (ax1, ax2) = plt.subplots(1,2)
ax1.plot( abs(dxs_gt-dxs_vo)+abs(dys_gt-dys_vo) ,label='VO alone')
ax1.legend()
ax2.plot( abs(dxs_gt-dxs_ba)+abs(dys_gt-dys_ba) ,label='VO + BA')
ax2.legend()

#%%
import pickle
file_path='sequence{}'.format(sequence_no)
file = open(file_path,'wb')
pickle.dump([gts,poses_base,poses_lm,poses_ba],file)
file.close()

#%%

def showTrajectory():
    xs_est = np.array(poses_base)[:,2,3]
    ys_est = -np.array(poses_base)[:,0,3]
    xs_est_ba = np.array(poses_ba)[:,2,3]
    ys_est_ba = -np.array(poses_ba)[:,0,3]
    xs_gt = np.array(gts)[:,2,3]
    ys_gt = -np.array(gts)[:,0,3]
    # Plot
    plt.plot(xs_gt, ys_gt, '.', color='black');
    plt.plot(xs_est, ys_est, '.', color='red');
    plt.plot(xs_est_ba, ys_est_ba, '.', color='blue');
    plt.gca().set_aspect('equal', adjustable='datalim')

file = open('main/results/sequence05','rb')
data = pickle.load(file)
file.close()

gts = data[0]
poses_base = data[1]
poses_ba = data[3]

showTrajectory()


#%%



#%%
