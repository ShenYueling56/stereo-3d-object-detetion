//
// creator: Created by shenyl on 2020/6/26.
// function：进行双目内外的标定和双目矫正,将内外参数和矫正参数存入标定文件夹下的calib_result.txt文件中
// usage：./stereo_calib -w=<board_width default=11> -h=<board_height default=8> -s=<square_size default=15> -d=<dir default=/home/shenyl/Documents/sweeper/data/> -show=<if_show default=False>
//
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <fstream>
#include <stdio.h>

using namespace cv;
using namespace std;

static int print_help()
{
    cout <<
         " Given a list of chessboard images, the number of corners (nx, ny)\n"
         " on the chessboards, the size of the square and the root dir of the calib image and the flag of if show the rectified results \n"
         " Calibrate and rectified the stereo camera, and save the result to txt \n"<< endl;
    cout << "Usage:\n // usage：./stereo_calib -w=<board_width default=11> -h=<board_height default=8> -s=<square_size default=15> -d=<dir default=/home/shenyl/Documents/sweeper/data/> -show=<if_show default=False>\n" << endl;
    return 0;
}

/*
单目标定
参数：
	imageList		存放标定图片名称的txt
	singleCalibrateResult	存放标定结果的txt
	objectPoints	世界坐标系中点的坐标
	corners_seq		存放图像中的角点,用于立体标定
	cameraMatrix	相机的内参数矩阵
	distCoeffs		相机的畸变系数
	imageSize		输入图像的尺寸（像素）
	patternSize		标定板每行的角点个数, 标定板每列的角点个数 (9, 6)
	chessboardSize	棋盘上每个方格的边长（mm）
注意：亚像素精确化时，允许输入单通道，8位或者浮点型图像。由于输入图像的类型不同，下面用作标定函数参数的内参数矩阵和畸变系数矩阵在初始化时也要数据注意类型。
*/
bool singleCameraCalibrate(string imageList, string singleCalibrateResult, vector<vector<Point3f>>& objectPoints,
                           vector<vector<Point2f>>& corners_seq, Mat& cameraMatrix, Mat& distCoeffs, Size& imageSize, Size patternSize, Size chessboardSize)
{
    int n_boards = 0;
    ifstream imageStore(imageList); // 打开存放标定图片名称的txt
    ofstream resultStore(singleCalibrateResult); // 保存标定结果的txt
    // 开始提取角点坐标
    vector<Point2f> corners; // 存放一张图片的角点坐标
    string imageName; // 读取的标定图片的名称
    while (getline(imageStore, imageName)) // 读取txt的每一行（每一行存放了一张标定图片的名称）
    {
//        cout<<"img_name"<<imageName<<endl;

        Mat imageInput = imread(imageName);
        cvtColor(imageInput, imageInput, CV_RGB2GRAY);
        imageSize.width = imageInput.cols; // 获取图片的宽度
        imageSize.height = imageInput.rows; // 获取图片的高度
        // 查找标定板的角点
        bool found = findChessboardCorners(imageInput, patternSize, corners); // 最后一个参数int flags的缺省值为：CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE
        // 亚像素精确化。在findChessboardCorners中自动调用了cornerSubPix，为了更加精细化，我们自己再调用一次。
        if (found) // 当所有的角点都被找到
        {
            TermCriteria criteria = TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001); // 终止标准，迭代40次或者达到0.001的像素精度
            cornerSubPix(imageInput, corners, Size(11, 11), Size(-1, -1), criteria);// 由于我们的图像只存较大，将搜索窗口调大一些，（11， 11）为真实窗口的一半，真实大小为（11*2+1， 11*2+1）--（23， 23）
            corners_seq.push_back(corners); // 存入角点序列
            // 绘制角点
            drawChessboardCorners(imageInput, patternSize, corners, true);
//            imshow("cornersframe", imageInput);
            n_boards++;
            cout<<"img_name"<<imageName<<endl;
//
//            if (waitKey(0) == 27)
//            {
//                destroyWindow("cornersframe");
//            }
        }
    }
    //destroyWindow("cornersframe");
    // 进行相机标定
    // 计算角点对应的三维坐标
    int pic, i, j;
    for (pic = 0; pic < n_boards; pic++)
    {
        vector<Point3f> realPointSet;
        for (i = 0; i < patternSize.height; i++)
        {
            for (j = 0; j < patternSize.width; j++)
            {
                Point3f realPoint;
                // 假设标定板位于世界坐标系Z=0的平面
                realPoint.x = j * chessboardSize.width;
                realPoint.y = i * chessboardSize.height;
                realPoint.z = 0;
                realPointSet.push_back(realPoint);
            }
        }
        objectPoints.push_back(realPointSet);
    }
    // 执行标定程序
    cout<<"start to calib"<<endl;
    vector<Mat> rvec; // 旋转向量
    vector<Mat> tvec; // 平移向量
    cout<<"object point num"<<endl<<objectPoints.size()<<endl;
    cout<<"corners_seq num"<<endl<<corners_seq.size()<<endl;
    calibrateCamera(objectPoints, corners_seq, imageSize, cameraMatrix, distCoeffs, rvec, tvec, 0);
    // 保存标定结果
    resultStore << "相机内参数矩阵" << endl;
    resultStore << cameraMatrix << endl << endl;
    resultStore << "相机畸变系数" << endl;
    resultStore << distCoeffs << endl << endl;
    // 计算重投影点，与原图角点比较，得到误差
    double errPerImage = 0.; // 每张图像的误差
    double errAverage = 0.; // 所有图像的平均误差
    double totalErr = 0.; // 误差总和
    vector<Point2f> projectImagePoints; // 重投影点
    for (i = 0; i < n_boards; i++)
    {
        vector<Point3f> tempObjectPoints = objectPoints[i]; // 临时三维点
        // 计算重投影点
        projectPoints(tempObjectPoints, rvec[i], tvec[i], cameraMatrix, distCoeffs, projectImagePoints);
        // 计算新的投影点与旧的投影点之间的误差
        vector<Point2f> tempCornersPoints = corners_seq[i];// 临时存放旧投影点
        Mat tempCornersPointsMat = Mat(1, tempCornersPoints.size(), CV_32FC2); // 定义成两个通道的Mat是为了计算误差
        Mat projectImagePointsMat = Mat(1, projectImagePoints.size(), CV_32FC2);
        // 赋值
        for (int j = 0; j < tempCornersPoints.size(); j++)
        {
            projectImagePointsMat.at<Vec2f>(0, j) = Vec2f(projectImagePoints[j].x, projectImagePoints[j].y);
            tempCornersPointsMat.at<Vec2f>(0, j) = Vec2f(tempCornersPoints[j].x, tempCornersPoints[j].y);
        }
        // opencv里的norm函数其实把这里的两个通道分别分开来计算的(X1-X2)^2的值，然后统一求和，最后进行根号
        errPerImage = norm(tempCornersPointsMat, projectImagePointsMat, NORM_L2) / (patternSize.width * patternSize.height);
        totalErr += errPerImage;
        resultStore << "第" << i + 1 << "张图像的平均误差为：" << errPerImage << endl;
    }
    resultStore << "全局平局误差为：" << totalErr / n_boards << endl;
    imageStore.close();
    resultStore.close();
    return true;
}

/*
双目标定:计算两摄像机相对旋转矩阵 R,平移向量 T, 本征矩阵E, 基础矩阵F
参数：
	stereoCalibrateResult	存放立体标定结果的txt
	objectPoints			三维点
	imagePoints				二维图像上的点
	cameraMatrix			相机内参数
	distCoeffs				相机畸变系数
	imageSize				图像尺寸
	R		左右相机相对的旋转矩阵
	T		左右相机相对的平移向量
	E		本征矩阵
	F		基础矩阵
*/
bool stereoCalibrate(string stereoCalibrateResult, vector<vector<Point3f>> objectPoints, vector<vector<Point2f>> imagePoints1, vector<vector<Point2f>> imagePoints2,
                     Mat& cameraMatrix1, Mat& distCoeffs1, Mat& cameraMatrix2, Mat& distCoeffs2, Size& imageSize, Mat& R, Mat& T, Mat& E, Mat& F)
{
    ofstream stereoStore(stereoCalibrateResult);
    TermCriteria criteria = TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 30, 1e-6); // 终止条件
    stereoCalibrate(objectPoints, imagePoints1, imagePoints2, cameraMatrix1, distCoeffs1,
                    cameraMatrix2, distCoeffs2, imageSize, R, T, E, F, CALIB_FIX_INTRINSIC, criteria); // 注意参数顺序，可以到保存的文件中查看，避免返回时出错
    stereoStore << "image size" << endl;
    stereoStore << imageSize << endl;
    stereoStore << "cameraMatrix_L" << endl;
    stereoStore << cameraMatrix1 << endl;
    stereoStore << "cameraMatrix_R" << endl;
    stereoStore << cameraMatrix2 << endl;
    stereoStore << "distCoeffs_L" << endl;
    stereoStore << distCoeffs1 << endl;
    stereoStore << "distCoeffs_R" << endl;
    stereoStore << distCoeffs2 << endl;
    stereoStore << "R" << endl;
    stereoStore << R << endl;
    stereoStore << "T" << endl;
    stereoStore << T << endl;
    stereoStore << "E" << endl;
    stereoStore << E << endl;
    stereoStore << "F" << endl;
    stereoStore << F << endl;
    stereoStore.close();
    return true;
}

/*
立体校正
参数：
	stereoRectifyParams	存放立体校正结果的txt
	cameraMatrix			相机内参数
	distCoeffs				相机畸变系数
	imageSize				图像尺寸
	R						左右相机相对的旋转矩阵
	T						左右相机相对的平移向量
	R1, R2					行对齐旋转校正
	P1, P2					左右投影矩阵
	Q						重投影矩阵
	map1, map2				重投影映射表
*/
Rect stereoRectification(string stereoRectifyParams, Mat& cameraMatrix1, Mat& distCoeffs1, Mat& cameraMatrix2, Mat& distCoeffs2,
                         Size& imageSize, Mat& R, Mat& T, Mat& R1, Mat& R2, Mat& P1, Mat& P2, Mat& Q, Mat& mapl1, Mat& mapl2, Mat& mapr1, Mat& mapr2)
{
    Rect validRoi[2];
    ofstream stereoStore(stereoRectifyParams);
    cout<<cameraMatrix1<<endl<<distCoeffs1<<endl<<cameraMatrix2<<endl<<distCoeffs2<<endl<<imageSize<<endl<<R <<endl<<T<<endl;
    stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageSize,
                  R, T, R1, R2, P1, P2, Q, 0, -1, imageSize, &validRoi[0], &validRoi[1]);
    // 计算左右图像的重投影映射表
    stereoStore << "R1：" << endl;
    stereoStore << R1 << endl;
    stereoStore << "R2：" << endl;
    stereoStore << R2 << endl;
    stereoStore << "P1：" << endl;
    stereoStore << P1 << endl;
    stereoStore << "P2：" << endl;
    stereoStore << P2 << endl;
    stereoStore << "Q：" << endl;
    stereoStore << Q << endl;
    stereoStore.close();
    cout << "R1:" << endl;
    cout << R1 << endl;
    cout << "R2:" << endl;
    cout << R2 << endl;
    cout << "P1:" << endl;
    cout << P1 << endl;
    cout << "P2:" << endl;
    cout << P2 << endl;
    cout << "Q:" << endl;
    cout << Q << endl;
    initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize, CV_32FC1, mapl1, mapl2);
    initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize, CV_32FC1, mapr1, mapr2);
    return validRoi[0], validRoi[1];
}
/*
显示双目矫正结果
参数：
	imageList_L          	存放左相机标定图片地址的txt
 	imageList_R          	存放右相机标定图片地址的txt
 	img1_rectified          左相机标定图片矫正后的结果图
 	img2_rectified          右相机标定图片矫正后的结果图
	mapl1, mapl2			左相机重投影映射表
    mapr1, mapr2            右相机重投影映射表
    validRoi                有效区域
*/
bool show_Rectification_result(string imageList_L, string imageList_R, Mat& img1_rectified,
                           Mat& img2_rectified, Mat& mapl1, Mat& mapl2, Mat& mapr1, Mat& mapr2, Rect validRoi[2]) {
    ifstream imageStore_L(imageList_L); // 打开存放标定图片名称的txt
    ifstream imageStore_R(imageList_R); // 打开存放标定图片名称的txt
    string imageName_L; // 读取的标定图片的名称
    string imageName_R; // 读取的标定图片的名称
    Size imageSize;
    while (getline(imageStore_L, imageName_L)) // 读取txt的每一行（每一行存放了一张标定图片的名称）
    {
        getline(imageStore_R, imageName_R);
        Mat img1 = imread(imageName_L);
        Mat img2 = imread(imageName_R);
        if (img1.empty() | img2.empty()) {
            cout << "图像为空" << endl;
        }
        Mat gray_img1, gray_img2;
        cvtColor(img1, gray_img1, COLOR_BGR2GRAY);
        cvtColor(img2, gray_img2, COLOR_BGR2GRAY);
        imageSize.width = img1.cols; // 获取图片的宽度
        imageSize.height = img1.rows; // 获取图片的高度
        Mat canvas(imageSize.height, imageSize.width * 2, CV_8UC1); // 注意数据类型
        Mat canLeft = canvas(Rect(0, 0, imageSize.width, imageSize.height));
        Mat canRight = canvas(Rect(imageSize.width, 0, imageSize.width, imageSize.height));
        gray_img1.copyTo(canLeft);
        gray_img2.copyTo(canRight);

        remap(gray_img1, img1_rectified, mapl1, mapl2, INTER_LINEAR);
        remap(gray_img2, img2_rectified, mapr1, mapr2, INTER_LINEAR);

        img1_rectified.copyTo(canLeft);
        img2_rectified.copyTo(canRight);
        rectangle(canLeft, validRoi[0], Scalar(255, 255, 255), 5, 8);
        rectangle(canRight, validRoi[1], Scalar(255, 255, 255), 5, 8);
        for (int j = 0; j <= canvas.rows; j += 16)
            line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
        imshow("rectified", canvas);
        if (waitKey(0) == 27) {
            destroyAllWindows();
        }
    }
}

int main(int argc,char *argv[])
{
    cv::CommandLineParser parser(argc, argv, "{w|11|}{h|8|}{s|15|}{d|/home/shenyl/Documents/sweeper/data/|}{show|False|}{help||}");
    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }
    int w = parser.get<int>("w");
    int h = parser.get<int>("h");
    int s = parser.get<int>("s");
    bool show = parser.get<bool>("show");
    string root_path = parser.get<string>("d");
    Size patternSize = Size(w, h); // 棋盘行列内角点个数
    Size chessboardSize = Size(s,s); //棋盘一个方格的边长（mm）
//    cout<<w<<endl<<h<<endl<<s<<endl<<show<<endl<<root_path<<endl;

    string imageList_L = root_path + "file_left.txt";
    string imageList_R = root_path + "file_right.txt";
    string singleCalibrate_result_L = root_path + "calibrationresults_L.txt";
    string singleCalibrate_result_R = root_path + "calibrationresults_R.txt";
    string stereoRectifyParams = root_path + "stereoRectifyParams.txt"; // 存放立体矫正结果
    string stereoCalibrate_result_L = root_path + "stereocalibrateresult_L.txt";

    vector<vector<Point3f>> objectPoints_L; // 三维坐标
    vector<vector<Point3f>> objectPoints_R;
    vector<vector<Point2f>> corners_seq_L; // 所有角点坐标
    vector<vector<Point2f>> corners_seq_R;
    Mat cameraMatrix_L = Mat(3, 3, CV_32FC1, Scalar::all(0)); // 相机的内参数
    Mat cameraMatrix_R = Mat(3, 3, CV_32FC1, Scalar::all(0)); // 初始化相机的内参数
    Mat distCoeffs_L = Mat(1, 5, CV_32FC1, Scalar::all(0)); // 相机的畸变系数
    Mat distCoeffs_R = Mat(1, 5, CV_32FC1, Scalar::all(0)); // 初始化相机的畸变系数
    Size imageSize; // 图像尺寸
    Rect validRoi[2];//双目矫正有效区域
    Mat R, T, E, F; // 立体标定参数
    Mat R1, R2, P1, P2, Q; // 立体校正参数
    Mat mapl1, mapl2, mapr1, mapr2; // 图像重投影映射表
    Mat img1_rectified, img2_rectified; // 校正图像

    //step1 calib left camera
    singleCameraCalibrate(imageList_L, singleCalibrate_result_L, objectPoints_L, corners_seq_L, cameraMatrix_L,
                          distCoeffs_L, imageSize, patternSize, chessboardSize);
    cout<<"finish left camera calibration!"<<endl;

    //step2 calib right camera
    singleCameraCalibrate(imageList_R, singleCalibrate_result_R, objectPoints_R, corners_seq_R, cameraMatrix_R,
                          distCoeffs_R, imageSize, patternSize, chessboardSize);
    cout<<"finish right camera calibration!"<<endl;

    //step3 stereo calibration
    stereoCalibrate(stereoCalibrate_result_L, objectPoints_L, corners_seq_L, corners_seq_R, cameraMatrix_L, distCoeffs_L,
                    cameraMatrix_R, distCoeffs_R, imageSize, R, T, E, F);
    cout<<"finish stereo calibration"<<endl;

    //step4 stereo Rectification
    validRoi[0], validRoi[1] = stereoRectification(stereoRectifyParams, cameraMatrix_L, distCoeffs_L, cameraMatrix_R, distCoeffs_R,
                                                   imageSize, R, T, R1, R2, P1, P2, Q, mapl1, mapl2, mapr1, mapr2);
    cout << "finish stereo Rectification" << endl;

    //if show == True, show image after rectification
    if (show==true)
    {
        show_Rectification_result(imageList_L, imageList_R, img1_rectified,
                img2_rectified, mapl1, mapl2, mapr1, mapr2, validRoi);
    }
}

