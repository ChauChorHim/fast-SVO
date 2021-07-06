#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar 20 12:49:38 2021

@author: kamil
"""

import cv2
import numpy as np



class KITTIDatasetWIN:
    def __init__(self, dataset_path,sequence_no):
        self.dataset_path = dataset_path
        self.sequence_no = sequence_no
        self.idx = 0

        ground_truth_path = self.dataset_path+'\\poses\\'+self.sequence_no+'.txt'
        ground_truth_file = open(ground_truth_path,'r')
        self.ground_truth_lines = ground_truth_file.readlines()
        ground_truth_file.close()

    def getCameraMatrix(self):
        file_path = self.dataset_path+'\\sequences\\'+self.sequence_no+'\\calib.txt'
        file = open(file_path, 'r')
        lines = file.readlines()
        file.close()

        # Projection matrices for monohrome cameras are P0 and P1 (left and right)
        P0_line = lines[0].split()
        P0_arr = np.array(P0_line[1:])
        P0_mat = np.reshape(P0_arr,[3,4])

        P1_line = lines[1].split()
        P1_arr = np.array(P1_line[1:])
        P1_mat = np.reshape(P1_arr,[3,4])

        P0_mat = P0_mat.astype(np.float)
        P1_mat = P1_mat.astype(np.float)

        return P0_mat, P1_mat


    def getData(self):
        """ Returns next left and right frame from the dataset, ground truth position,
            and a bool success value. True if it works, false in case there
            are no more frames or error
        """
        img_idx_str = "{:06d}.png".format(self.idx)
        frame_left = cv2.imread(self.dataset_path+'\\sequences\\'+self.sequence_no+'\\image_0\\'+img_idx_str,0)
        frame_right = cv2.imread(self.dataset_path+'\\sequences\\'+self.sequence_no+'\\image_1\\'+img_idx_str,0)

        if frame_left is None or frame_right is None:
            success = False
        else:
            success = True

        # Get ground truth data
        try:
            ground_truth = np.reshape(np.array(self.ground_truth_lines[self.idx].split()),[3,4]).astype(np.float)
        except IndexError:
            success = False
            ground_truth = 0

        self.idx += 1

        return success,frame_left,frame_right,ground_truth
    
