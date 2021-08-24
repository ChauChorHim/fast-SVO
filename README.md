# fast-SVO
fast-SVO is a basic stereo visual odometry. A python version SVO implemented during the course 02504 (Computer Vision F21) in DTU and a popular SLAM structure ORB-SLAM give  inspiration for the SVO design and C++ implementation style for this project. Feature detection and matching parts use inline functions provided by OpenCV. P3P solution provided by Kneip http://rpg.ifi.uzh.ch/docs/CVPR11_kneip.pdfis is implemented in this project.
## Prerequisites
This project is implemented and teseted in Ubuntu 18.04.
### C++11 Compiler
This project use std::thread, std::chrono.
### OpenCV
This project uses the latest OpenCV 4.5.1.
### Eigen3
This project uses version 3.3.91.

## Example
### KITTI Dataset
Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php  
The setting files are stored in **${PROJECT_SOURCE_DIR}/Example/KITTI setting/**  
Usage example:  
````bash
./Example/Stereo/stereo_kitti Example/KITTI_setting/KITTI04-12.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/ 07 
````

## Evaluation

![](https://github.com/ChauChorHim/Books-Pictures/raw/main/pictures/20210824102148.png)
![](https://github.com/ChauChorHim/Books-Pictures/raw/main/pictures/20210824102147.png)
![](https://github.com/ChauChorHim/Books-Pictures/raw/main/pictures/20210824102143.png)
![](https://github.com/ChauChorHim/Books-Pictures/raw/main/pictures/20210824102144.png)
![](https://github.com/ChauChorHim/Books-Pictures/raw/main/pictures/20210824102145.png)
![](https://github.com/ChauChorHim/Books-Pictures/raw/main/pictures/20210824102146.png)

