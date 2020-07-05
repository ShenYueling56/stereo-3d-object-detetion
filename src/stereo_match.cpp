//
// Created by shenyl on 2020/6/26.
//

#include "stereo_match.h"

static int print_help()
{
    cout <<
         " Given the root dir of the exp dir and the match algorithm \n"
         " output the rectified images and sparities\n"<< endl;
    cout << "Usage:\n // usage：./stereo_match -d=<dir default=/home/shenyl/Documents/sweeper/data/> -a=<stereo match algorithm default = sgbm>\n" << endl;
}


//attention: data must be read as double to be the same with the parameter definition of cv::stereoRectify function
void read_calib_parameters(string calib_parameters, Mat& cameraMatrix_L, Mat& distCoeffs_L, Mat& cameraMatrix_R, Mat& distCoeffs_R,
        Mat& R, Mat& T, Size& imageSize)
{

    ifstream calib_file(calib_parameters);
    if (!calib_file)
    {
        cout<<calib_parameters<<"is not exist"<<endl;
    }
    string line;
    enum StringValue { evNotDefined,
        evStringValue1,
        evStringValue2,
        evStringValue3,
        evStringValue4,
        evStringValue5,
        evStringValue6,
        evStringValue7,
        evEnd };
    map<std::string, StringValue> s_mapStringValues;
    s_mapStringValues["image size"] = evStringValue1;
    s_mapStringValues["cameraMatrix_L"] = evStringValue2;
    s_mapStringValues["cameraMatrix_R"] = evStringValue3;
    s_mapStringValues["distCoeffs_L"] = evStringValue4;
    s_mapStringValues["distCoeffs_R"] = evStringValue5;
    s_mapStringValues["R"] = evStringValue6;
    s_mapStringValues["T"] = evStringValue7;
    s_mapStringValues["end"] = evEnd;

    while (getline(calib_file, line))
    {
        switch(s_mapStringValues[line]){
            case evStringValue1:  //image size
            {
                getline(calib_file, line, '[');
                getline(calib_file, line, ' ');
                double w = atof(line.c_str());
                getline(calib_file, line, ' ');
                getline(calib_file, line, ']');
                double h = atof(line.c_str());
                imageSize = Size(w, h);;
                cout<< "imagesize"<< endl<< imageSize<<endl;
                break;
            }
            case evStringValue2:  //cameraMatrix_L
            {
                getline(calib_file, line, '[');
                getline(calib_file, line, ',');
                double m11 = atof(line.c_str());
                getline(calib_file, line, ',');
                double m12 = atof(line.c_str());
                getline(calib_file, line, ';');
                double m13 = atof(line.c_str());
                getline(calib_file, line, ',');
                double m21 = atof(line.c_str());
                getline(calib_file, line, ',');
                double m22 = atof(line.c_str());
                getline(calib_file, line, ';');
                double m23 = atof(line.c_str());
                cameraMatrix_L = (Mat_<double>(3, 3) << m11, m12, m13, m21, m22, m23, 0, 0, 1);
                cout<< "cameraMatrix_L"<< endl<< cameraMatrix_L<<endl;
                getline(calib_file, line);
                break;
            }
            case evStringValue3: //cameraMatrix_R
            {
                getline(calib_file, line, '[');
                getline(calib_file, line, ',');
                double m11 = atof(line.c_str());
                getline(calib_file, line, ',');
                double m12 = atof(line.c_str());
                getline(calib_file, line, ';');
                double m13 = atof(line.c_str());
                getline(calib_file, line, ',');
                double m21 = atof(line.c_str());
                getline(calib_file, line, ',');
                double m22 = atof(line.c_str());
                getline(calib_file, line, ';');
                double m23 = atof(line.c_str());
                cameraMatrix_R = (Mat_<double>(3, 3) << m11, m12, m13, m21, m22, m23, 0, 0, 1);
                cout<< "cameraMatrix_R"<< endl<< cameraMatrix_R<<endl;
                getline(calib_file, line);
                break;
            }
            case evStringValue4: //distCoeffs_L
            {
                getline(calib_file, line, '[');
                getline(calib_file, line, ',');
                double v1 = atof(line.c_str());
                getline(calib_file, line, ',');
                double v2 = atof(line.c_str());
                getline(calib_file, line, ',');
                double v3 = atof(line.c_str());
                getline(calib_file, line, ',');
                double v4 = atof(line.c_str());
                getline(calib_file, line, ']');
                double v5 = atof(line.c_str());
                distCoeffs_L = (Mat_<double>(1, 5) << v1, v2, v3, v4, v5);
                cout<< "distCoeffs_L"<< endl<< distCoeffs_L<<endl;
                break;
            }
            case evStringValue5: //distCoeffs_R
            {
                getline(calib_file, line, '[');
                getline(calib_file, line, ',');
                double v1 = atof(line.c_str());
                getline(calib_file, line, ',');
                double v2 = atof(line.c_str());
                getline(calib_file, line, ',');
                double v3 = atof(line.c_str());
                getline(calib_file, line, ',');
                double v4 = atof(line.c_str());
                getline(calib_file, line, ']');
                double v5 = atof(line.c_str());
                distCoeffs_R = (Mat_<double>(1, 5) << v1, v2, v3, v4, v5);
                cout<< "distCoeffs_R"<< endl<< distCoeffs_R<<endl;
                break;
            }
            case evStringValue6: //R
            {
                getline(calib_file, line, '[');
                getline(calib_file, line, ',');
                double m11 = atof(line.c_str());
                getline(calib_file, line, ',');
                double m12 = atof(line.c_str());
                getline(calib_file, line, ';');
                double m13 = atof(line.c_str());
                getline(calib_file, line, ',');
                double m21 = atof(line.c_str());
                getline(calib_file, line, ',');
                double m22 = atof(line.c_str());
                getline(calib_file, line, ';');
                double m23 = atof(line.c_str());
                getline(calib_file, line, ',');
                double m31 = atof(line.c_str());
                getline(calib_file, line, ',');
                double m32 = atof(line.c_str());
                getline(calib_file, line, ']');
                double m33 = atof(line.c_str());
                R = (Mat_<double>(3, 3) << m11, m12, m13, m21, m22, m23, m31, m32, m33);
                cout<< "R"<< endl<< R<<endl;
                break;
            }
            case evStringValue7: //T
            {
                getline(calib_file, line, '[');
                getline(calib_file, line, ';');
                double v1 = atof(line.c_str());
                getline(calib_file, line, ';');
                double v2 = atof(line.c_str());
                getline(calib_file, line, ']');
                double v3 = atof(line.c_str());
                T = (Mat_<double>(3, 1) << v1, v2, v3);
                cout<< "T"<< endl<< T<<endl;
                break;
            }
        }

    }
}

Rect stereoRectification(Mat& cameraMatrix1, Mat& distCoeffs1, Mat& cameraMatrix2, Mat& distCoeffs2,
                         Size& imageSize, Mat& R, Mat& T, Mat& R1, Mat& R2, Mat& P1, Mat& P2, Mat& Q, Mat& mapl1, Mat& mapl2, Mat& mapr1, Mat& mapr2)
{
    Rect validRoi[2];
//    cout<<cameraMatrix1<<endl<<distCoeffs1<<endl<<cameraMatrix2<<endl<<distCoeffs2<<endl<<imageSize<<endl<<R <<endl<<T<<endl;
//    flags-可选的标志有两种零或者 CV_CALIB_ZERO_DISPARITY ,如果设置 CV_CALIB_ZERO_DISPARITY 的话，该函数会让两幅校正后的图像的主点有相同的像素坐标。否则该函数会水平或垂直的移动图像，以使得其有用的范围最大
//    alpha-拉伸参数。如果设置为负或忽略，将不进行拉伸。如果设置为0，那么校正后图像只有有效的部分会被显示（没有黑色的部分），如果设置为1，那么就会显示整个图像。设置为0~1之间的某个值，其效果也居于两者之间。
//    alpha 参数必须设置为0，否则图像可能为倒像？？？
    stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageSize,
                  R, T, R1, R2, P1, P2, Q, 0, 0, imageSize, &validRoi[0], &validRoi[1]);
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

bool Rectification(string root_path, string imageName_L, string imageName_R, Mat& img1_rectified,
                               Mat& img2_rectified, Mat& mapl1, Mat& mapl2, Mat& mapr1, Mat& mapr2, Rect validRoi[2], int count) {
    Size imageSize;
    string left_rectified = root_path + "rectified/left/";
    string right_rectified = root_path + "rectified/right/";
    string pairs_rectified = root_path + "rectified/pairs/";
    Mat img1 = imread(imageName_L);
    Mat img2 = imread(imageName_R);
    if (img1.empty() | img2.empty()) {
        cout << imageName_L << " , " <<imageName_R<<" is not exist" << endl;
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

    char left_file[200];
    sprintf(left_file, "%06d.jpg", count);
    imwrite(left_rectified + left_file, img1_rectified);
    char right_file[200];
    sprintf(right_file, "%06d.jpg", count);
    imwrite(right_rectified + right_file, img2_rectified);

    img1_rectified.copyTo(canLeft);
    img2_rectified.copyTo(canRight);

    rectangle(canLeft, validRoi[0], Scalar(255, 255, 255), 5, 8);
    rectangle(canRight, validRoi[1], Scalar(255, 255, 255), 5, 8);
    for (int j = 0; j <= canvas.rows; j += 16)
        line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);

    char pairs_file[200];
    sprintf(pairs_file, "%06d.jpg", count);
    imwrite(pairs_rectified + pairs_file, canvas);
//        imshow("rectified", canvas);
//        if (waitKey(0) == 27) {
//            destroyAllWindows();
//        }
}

bool computeDisparityImage(string root_path, Mat& img1_rectified,
                           Mat& img2_rectified, Mat& disparity, int count, string algorithm)
{
    enum StringValue { evNotDefined,
        evStringValue1,
        evStringValue2,
        evStringValue3,
        evEnd };
    map<std::string, StringValue> s_mapStringValues;
    s_mapStringValues["bm"] = evStringValue1;
    s_mapStringValues["sgbm"] = evStringValue2;
    s_mapStringValues["elas"] = evStringValue3;
    s_mapStringValues["end"] = evEnd;

    switch(s_mapStringValues[algorithm]){
        case evStringValue1:
        {
            // 进行立体匹配 bm
            Ptr<StereoBM> bm;
            bm = StereoBM::create(16, 9); // Ptr<>是一个智能指针
            bm->compute(img1_rectified, img2_rectified, disparity); // 计算视差图 input rectified must be one-channel image
            disparity.convertTo(disparity, CV_32F, 1.0 / 16);
            // 归一化视差映射
            normalize(disparity, disparity, 0, 256, NORM_MINMAX, -1);
            break;
        }
        case evStringValue2:
        {
            // 进行立体匹配 sgbm
            Ptr<StereoSGBM> sgbm;
//            cout<<"image type: "<<img1_rectified.type()<<endl;
//            cout<<"image shape"<<img1_rectified.cols << "," << img1_rectified.rows<<endl;
            //todo: the numDisparities determine the nearest distance for det, try to modify the numDisparities according to the possible distance
            sgbm = cv::StereoSGBM::create(
                    0, 160, 8, 8*8*8, 32*8*8, 1, 1, 10, 200, 200, cv::StereoSGBM::MODE_SGBM);
            sgbm->compute(img1_rectified, img2_rectified, disparity); // 计算视差图

            disparity.convertTo(disparity, CV_32F, 1.0 / 16);
            // 归一化视差映射
//            normalize(disparity, disparity, 0, 256, NORM_MINMAX, CV_8U);
            break;
        }
        case evStringValue3:
        {
            // generate disparity image using LIBELAS
            cv::Mat disp_l,disp_r,disp8u_l,disp8u_r;
            double minVal; double maxVal; //视差图的极值
            int bd = 0;
            const int32_t dims[3] = {img1_rectified.cols,img1_rectified.rows,img1_rectified.cols};
            cv::Mat leftdpf = cv::Mat::zeros(cv::Size(img1_rectified.cols,img1_rectified.rows), CV_32F);
            cv::Mat rightdpf = cv::Mat::zeros(cv::Size(img1_rectified.cols,img1_rectified.rows), CV_32F);
            Elas::parameters param;
            param.postprocess_only_left = false;
            Elas elas(param);
            elas.process(img1_rectified.data,img2_rectified.data,leftdpf.ptr<float>(0),rightdpf.ptr<float>(0),dims);

            cv::Mat(leftdpf(cv::Rect(bd,0,img1_rectified.cols,img1_rectified.rows))).copyTo(disp_l);
            cv::Mat(rightdpf(cv::Rect(bd,0,img2_rectified.cols,img2_rectified.rows))).copyTo(disp_r);

            //-- Check its extreme values
            cv::minMaxLoc( disp_l, &minVal, &maxVal );
            cout<<"Min disp: Max value"<< minVal<<maxVal; //numberOfDisparities.= (maxVal - minVal)

            //-- Display it as a CV_8UC1 image
//            disp_l.convertTo(disp8u_l, CV_8U, 255/(maxVal - minVal));//(numberOfDisparities*16.)
            disp_l.convertTo(disp8u_l, CV_8U);

            cv::minMaxLoc( disp_r, &minVal, &maxVal );
            cout<<"Min disp: Max value"<< minVal<<maxVal; //numberOfDisparities.= (maxVal - minVal)

            //-- Display it as a CV_8UC1 image
//            disp_r.convertTo(disp8u_r, CV_8U, 255/(maxVal - minVal));//(numberOfDisparities*16.)

//            cv::normalize(disp8u_l, disp8u_l, 0, 255, CV_MINMAX, CV_8UC1);    // obtain normalized image
//            cv::normalize(disp8u_r, disp8u_r, 0, 255, CV_MINMAX, CV_8UC1);    // obtain normalized image
            disparity = disp8u_l;
            break;
        }
    }
    // write sparities to files
    string disparities = root_path + "disparities/";
//    imshow("disparity", disparity);
//    if (waitKey(0) == 27) {
//        destroyAllWindows();
//    }
    char disparities_file[200];
    sprintf(disparities_file, "%06d.jpg", count);
    imwrite(disparities + disparities_file, disparity);
    return true;
}




