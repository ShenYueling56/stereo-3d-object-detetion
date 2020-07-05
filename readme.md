#Introdution
This is a project for 3D object detection with stereo camera. With the bounding box proposals from some DL-based 2D detection methods such as YOLO and YOLO-Nano, the position of objects can be obtained through stereo reconstruction.
#How to use
##Dataset Preparation 
```
.
├── 2d_objects
│   ├── left
│   └── right
├── calib_img
│   ├── left
│   ├── right
│   ├── stereocalibrateresult_L.txt
│   └── stereocalibrateresult_matlab.txt
├── ground_truth
└── img
    ├── left
    └── right
```
##Stereo Calibration
####OpenCV method
```
# gen calib file list
python ../scripts/gen_file_list/gen_file_list.py /home/shenyl/Documents/sweeper/data/exp0702/calib_img/

./stereo_calib -w=11 -h=8 -s=15 -d=/home/shenyl/Documents/sweeper/data/exp0702/calib_img/

# To vertify, the rectified image pairs can be shown
./stereo_calib -w=11 -h=8 -s=15 -d=/home/shenyl/Documents/sweeper/data/exp0702/./stereo_calib -w=11 -h=8 -s=15 -d=/home/shenyl/Documents/sweeper/data/exp0702/calib_img/ -show=true
```
####matlab method
```
# run stereoCameraCalibrator in matlab
# run matlab_calib.m in ./scripts/matlab_calib/
# change the parameters in /home/shenyl/Documents/sweeper/data/exp0702/calib_img/stereocalibrateresult_matlab.txt
```
##Generate rectified images
The images should be rectified before feeding into 2D detection networks.
```
python ../scripts/gen_file_list/gen_file_list.py /home/shenyl/Documents/sweeper/data/exp0702/img/

# use opencv calib result and sgbm stereo matching method
./gen_sparities -d=/home/shenyl/Documents/sweeper/data/exp0702/ -c=opencv -a=sgbm

# use matlab calib result and sgbm stereo matching method
./gen_sparities -d=/home/shenyl/Documents/sweeper/data/exp0702/ -c=matlab -a=sgbm

# use opencv calib result and sgbm stereo matching method
./gen_sparities -d=/home/shenyl/Documents/sweeper/data/exp0702/ -c=opencv -a=elas

# use matlab calib result and elas stereo matching method
./gen_sparities -d=/home/shenyl/Documents/sweeper/data/exp0702/ -c=matlab -a=elas


```

##Object Detection
```
# use opencv calib result and sgbm stereo matching method
./object_detection -d=/home/shenyl/Documents/sweeper/data/exp0702/ -c=opencv -a=sgbm

# use matlab calib result and sgbm stereo matching method
./object_detection -d=/home/shenyl/Documents/sweeper/data/exp0702/ -c=matlab -a=sgbm

# use opencv calib result and elas stereo matching method
./object_detection -d=/home/shenyl/Documents/sweeper/data/exp0702/ -c=opencv -a=elas

# use matlab calib result and elas stereo matching method
./object_detection -d=/home/shenyl/Documents/sweeper/data/exp0702/ -c=matlab -a=elas

```
the result will be saved to object_det_result/
pcd result will be saved to pcd/

## Evaluation
```
# evaluate the result
python ../scripts/eval/eval.py /home/shenyl/Documents/sweeper/data/exp0702/ object_det_opencv_sgbm.txt

```

