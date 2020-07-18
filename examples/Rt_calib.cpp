//
// Created by shenyl on 2020/7/6.
//
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <map>
#include <stdlib.h>
#include <cstdlib>
#include <string>

using namespace cv;
using namespace std;

Mat img1;
Mat img2;
Mat img3;
Mat img4;
static vector<Point2f> points;

static int print_help()
{
    cout <<
         " Given a list of chessboard images, the number of corners (nx, ny)\n"
         " on the chessboards, the size of the square and the root dir of the calib image and the flag of if show the rectified results \n"
         " Calibrate and rectified the stereo camera, and save the result to txt \n"<< endl;
    cout << "Usage:\n // usage：./Rt_calib -d=<dir default=/home/shenyl/Documents/sweeper/data/> \n" << endl;
    return 0;
}

// 鼠标回调函数，点击视差图显示深度
void onMouse(int event, int x, int y, int flags, void *param)
{
    Point point;
    point.x = x;
    point.y = y;
    if(event == EVENT_LBUTTONDOWN)
    {
        cout << x << " " << y << endl;
        points.push_back(point);
    }
}

bool read_3d_points(string points3d_path, vector<Point3f>& points3d)
{
    ifstream points3d_file(points3d_path);
    if (!points3d_file)
    {
        cout<<points3d_path<<"is not exist"<<endl;
    }
    Point3f p_3d;
    float x, y, z;
    while (points3d_file >> x >> y >> z)
    {
        p_3d.x=x;
        p_3d.y=y;
        p_3d.z=z;
//        cout<<x<<" "<<y<<" "<<z<<endl;
        points3d.push_back(p_3d);
    }
    points3d_file.close();
    return true;
}

bool read_camera_para(string camera_para_path, Mat& CameraMatrix, Mat& distCoeffs)
{
    ifstream camera_para_file(camera_para_path);
    if (!camera_para_file)
    {
        cout<<camera_para_path<<"is not exist"<<endl;
    }
    string line;
    enum StringValue { evNotDefined,
        evStringValue1,
        evStringValue2,
        evEnd };
    map<std::string, StringValue> s_mapStringValues;
    s_mapStringValues["cameraMatrix_L"] = evStringValue1;
    s_mapStringValues["distCoeffs_L"] = evStringValue2;
    s_mapStringValues["end"] = evEnd;

    while (getline(camera_para_file, line)) {
        switch (s_mapStringValues[line]) {
            case evStringValue1:  //image size
            {
                getline(camera_para_file, line, '[');
                getline(camera_para_file, line, ',');
                double m11 = atof(line.c_str());
                getline(camera_para_file, line, ',');
                double m12 = atof(line.c_str());
                getline(camera_para_file, line, ';');
                double m13 = atof(line.c_str());
                getline(camera_para_file, line, ',');
                double m21 = atof(line.c_str());
                getline(camera_para_file, line, ',');
                double m22 = atof(line.c_str());
                getline(camera_para_file, line, ';');
                double m23 = atof(line.c_str());
                CameraMatrix = (Mat_<double>(3, 3) << m11, m12, m13, m21, m22, m23, 0, 0, 1);
                cout << "cameraMatrix_L" << endl << CameraMatrix << endl;
                getline(camera_para_file, line);
                break;
            }
            case evStringValue2: //distCoeffs_L
            {
                getline(camera_para_file, line, '[');
                getline(camera_para_file, line, ',');
                double v1 = atof(line.c_str());
                getline(camera_para_file, line, ',');
                double v2 = atof(line.c_str());
                getline(camera_para_file, line, ',');
                double v3 = atof(line.c_str());
                getline(camera_para_file, line, ',');
                double v4 = atof(line.c_str());
                getline(camera_para_file, line, ']');
                double v5 = atof(line.c_str());
                distCoeffs = (Mat_<double>(1, 5) << v1, v2, v3, v4, v5);
                cout << "distCoeffs_L" << endl << distCoeffs << endl;
                break;
            }
        }
    }
}

int main(int argc,char *argv[]) {
    cv::CommandLineParser parser(argc, argv, "{p1|/home/shenyl/Documents/sweeper/data/|}{p2|/home/shenyl/Documents/sweeper/data/|}{p3|/home/shenyl/Documents/sweeper/data/|}{help||}");
    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }
    string img_path = parser.get<String>("p1");
    string points3d_path = parser.get<String>("p2");
    string camera_para_path = parser.get<String>("p3");
    string img_path1 = img_path + "000000.jpg";
    string img_path2 = img_path + "000001.jpg";
    string img_path3 = img_path + "000002.jpg";
    cout<<"img_path"<<endl;
    img1 = imread(img_path1);
    img2 = imread(img_path2);
    img3 = imread(img_path3);
    //pick points on images
    imshow("img1", img1);
    setMouseCallback("img1", onMouse);
    if (waitKey(0) == 27) {
        destroyAllWindows();
    }
    imshow("img2", img2);
    setMouseCallback("img2", onMouse);
    if (waitKey(0) == 27) {
        destroyAllWindows();
    }
    imshow("img3", img3);
    setMouseCallback("img3", onMouse);
    if (waitKey(0) == 27) {
        destroyAllWindows();
    }

    for (int i=0;i<points.size();i++)
    {
        cout<<points[i].x << " "<<points[i].y <<endl;
    }
    //read 3d coordinates from txt
    vector<Point3f> points3d;
    read_3d_points(points3d_path, points3d);
    for(int i =0;i<points3d.size();i++)
    {
        cout<<points3d[i].x<<" "<<points3d[i].y<<" "<<points3d[i].z<<endl;
    }
    //read camera camera parameters
    Mat CameraMatrix, distcoeffs;
    read_camera_para(camera_para_path, CameraMatrix, distcoeffs);

    // calculate R and t
    Mat rvec, tvec;
    // RANSAC parameters
    int iterationsCount = 300;      // number of Ransac iterations.
    float reprojectionError = 5.991;  // maximum allowed distance to consider it an inlier.
    double confidence = 0.95;        // ransac successful confidence.
    Mat inliers;

    cout<<"2d points num"<<points.size()<<endl;
    cout<<"3d points num"<<points3d.size()<<endl;
    solvePnPRansac(points3d, points, CameraMatrix, distcoeffs, rvec, tvec, false, iterationsCount, reprojectionError, confidence, inliers, SOLVEPNP_ITERATIVE);
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cout<<R<<endl<<tvec<<endl;

}

