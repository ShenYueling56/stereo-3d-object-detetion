##step1 calibration
cd ../build
#./stereo_calib -d=/home/shenyl/Documents/sweeper/data/exp0624/calib_img/
# if want to show the rectified images
#./stereo_calib -d=/home/shenyl/Documents/sweeper/data/exp0624/calib_img/ -show=1

## step2 stereo match
./stereo_match -d=/home/shenyl/Documents/sweeper/data/exp0624/