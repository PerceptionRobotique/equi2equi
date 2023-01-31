equi2equi: warps an equirectangular image to a sphere, transform it, then map back to the equirectangular plane

January 2023
Author: G. Caron
Contact: guillaume.caron@u-picardie.fr

Prerequisities
0. CMake (version 3.16.3 tested)
1. ViSP (version 3.4.1 tested)
2. libPeR_base (version 0.0.3 tested, https://github.com/PerceptionRobotique/libPeR_base)

Configure and prepare equi2equi to build
0. export PER_DIR=/path/to/libPeR_base/build/
1. create a "equi2equi-build" directory in the same directory than the equi2equi directory is and cd in
2. run ccmake ../equi2equi, configure and generate
3. make

Run equi2equi
0. cd ..
1. Download and unzip examples of equirectangular images in the media directory downloaded from : https://mis.u-picardie.fr/~g-caron/data/2023_SVMISplus_er-0-9.zip (not publicly available for the moment; later, please see the SVMISplus official website https://mis.u-picardie.fr/~g-caron/datasets for the full list of image sequences)
2. run from the command line
  ./equi2equi-build/equi2equi ./2023_SVMISplus_er-0-9/SVMISplus/Matrice600Pro/SixDOFs/Images_Equirectangular/ 0 9 1 ./2023_SVMISplus_er-0-9/SVMISplus/Matrice600Pro/SixDOFs/camera_poses.txt
command line arguments are:
* imagesDir directory where equirectangular images to read (with 'e_' followed with 6 digits before the extension) are and where the output transformed equirectangular images will be written (with characters 'te_' before 6 digits)
* iFirst the number of the first image to transform
* iLast the number of the last image to transform
* iStep (optional if none after) the increment to the next image to transform
* posesFic (optional if none after) a text file of one 3D pose per image (one row - one image) stored as the 3 elements of the translation vector followed by the 3 elements of the axis-angle vector representation of the rotation
* iInvertPose (optional) a flag to let the program knows if poses of posesFic must be applied to images as they are or inversed

TODO: remove boost dependencies currently obtained through ViSP

