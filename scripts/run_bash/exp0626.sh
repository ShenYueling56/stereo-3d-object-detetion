##step1 calibration (only have to do at the first time)
cd ../build
# ./stereo_calib -d=/home/shenyl/Documents/sweeper/data/exp0626/calib_img/

## step2 stereo match
# ./stereo_match -d=/home/shenyl/Documents/sweeper/data/exp0626/

## step3 object detection with matlab calib result and sgbm stereo match algorithm
./object_detection -d=/home/shenyl/Documents/sweeper/data/exp0626/ -c=matlab
## step3 object detection with opencv calib result and sgbm stereo match algorithm
# ./object_detection -d=/home/shenyl/Documents/sweeper/data/exp0626/ -c=opencv