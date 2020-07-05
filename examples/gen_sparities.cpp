//
// Created by shenyl on 2020/7/2.
// function：读取标定结果文件并进行图像的双目矫正、匹配和深度估计，生成校正后图像和视差图
// usage：./gen_sparities -d=<dir default=/home/shenyl/Documents/sweeper/data/> -a=<stereo match algorithm default = sgbm> -c=<calib_method default matlab>
//
#include "stereo_match.h"


int main(int argc,char *argv[])
{
    cv::CommandLineParser parser(argc, argv, "{d|/home/shenyl/Documents/sweeper/data/|}{c|matlab|}{a|sgbm|}{help||}");
    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }
    String root_path = parser.get<String>("d");
    String algorithm = parser.get<String>("a");
    string calib_method = parser.get<String>("c");

    String imageList_L = root_path + "img/file_left.txt";
    String imageList_R = root_path + "img/file_right.txt";
    string calib_opencv_file =  root_path + "calib_img/stereocalibrateresult_L.txt";
    string calib_matlab_file = root_path + "calib_img/stereocalibrateresult_matlab.txt";
    string calib_file;
    String rectified_parameters = root_path + "calib_img/stereoRectifyParams.txt";

    Mat cameraMatrix_L = Mat(3, 3, CV_32FC1, Scalar::all(0)); // 相机的内参数
    Mat cameraMatrix_R = Mat(3, 3, CV_32FC1, Scalar::all(0)); // 初始化相机的内参数
    Mat distCoeffs_L = Mat(1, 5, CV_32FC1, Scalar::all(0)); // 相机的畸变系数
    Mat distCoeffs_R = Mat(1, 5, CV_32FC1, Scalar::all(0)); // 初始化相机的畸变系数
    Mat R, T;
    Size imageSize;
    Rect validRoi[2];//双目矫正有效区域
    Mat R1, R2, P1, P2, Q; // 立体校正参数
    Mat mapl1, mapl2, mapr1, mapr2; // 图像重投影映射表
    Mat img1_rectified, img2_rectified; // 校正图像
    Mat disparity; // 视差图

    //Step1 read camera parameters
    if (calib_method == "matlab")
    {
        calib_file = calib_matlab_file;
    }
    if (calib_method == "opencv")
    {
        calib_file = calib_opencv_file;
    }

    //Step1 read camera parameters
    read_calib_parameters(calib_file, cameraMatrix_L, distCoeffs_L, cameraMatrix_R, distCoeffs_R,
            R, T, imageSize);
//    cout<< R << endl<< T << endl << cameraMatrix_L << endl << distCoeffs_L << endl << cameraMatrix_R << endl<< distCoeffs_R<<endl;
    cout<<"finish read calib parameters"<<endl;
    //Step2 get stereo rectified parameters
    validRoi[0], validRoi[1] = stereoRectification(cameraMatrix_L, distCoeffs_L, cameraMatrix_R, distCoeffs_R,
                                                   imageSize, R, T, R1, R2, P1, P2, Q, mapl1, mapl2, mapr1, mapr2);
    cout<<"finish stereo rectification"<<endl;
    string left_rectified = root_path + "rectified/left/";
    string right_rectified = root_path + "rectified/right/";
    string pairs_rectified = root_path + "rectified/pairs/";
    string disparities = root_path + "disparities/";
    string command = "mkdir -p " + left_rectified;
    system(command.c_str());
    command = "mkdir -p " + right_rectified;
    system(command.c_str());
    command = "mkdir -p " + pairs_rectified;
    system(command.c_str());
    command = "mkdir -p " + disparities;
    system(command.c_str());

    ifstream imageStore_L(imageList_L); // 打开存放标定图片名称的txt
    ifstream imageStore_R(imageList_R); // 打开存放标定图片名称的txt
    string imageName_L; // 读取的标定图片的名称
    string imageName_R; // 读取的标定图片的名称
    int count = 0 ;
    // for each pair rectified images and generate sparity
    while (getline(imageStore_L, imageName_L))
    {
        getline(imageStore_R, imageName_R);
        // Step3 rectified images
        Rectification(root_path, imageName_L, imageName_R, img1_rectified, img2_rectified, mapl1, mapl2, mapr1, mapr2, validRoi, count);
        // Step4 generate sparities
        computeDisparityImage(root_path, img1_rectified,
                img2_rectified, disparity, count, algorithm);
        count = count +1;
    }
}
