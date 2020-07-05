//
// Created by shenyl on 2020/6/27.
// function：利用标定结果进行图像双目矫正、匹配和深度图计算，读取二维物体检测结果进行物体三维位置估计
// usage：./object_detection -d=<dir default=/home/shenyl/Documents/sweeper/data/> -a=<stereo match algorithm default=sgbm> -c=<calib_method default=matlab>
//

#include "bbox2box3d.h"


static int print_help()
{
    cout <<
         " Given the root dir of the exp dir and the match algorithm \n"
         " output the rectified images and sparities\n"<< endl;
    cout << "Usage:\n // usage：./stereo_match -d=<dir default=/home/shenyl/Documents/sweeper/data/> -a=<stereo match algorithm default = sgbm>\n" << endl;
}



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



    Mat rectified_l, rectified_r;
    Mat disparity;
    srand(time(NULL));//设置随机数种子，使每次获取的随机序列不同。
    //dir for save rectificated images
    string left_rectified = root_path + "rectified/left/";
    string right_rectified = root_path + "rectified/right/";
    string pairs_rectified = root_path + "rectified/pairs/";
    string disparities = root_path + "disparities/";
    //dir for save sparity img with bbox
    string disparity_with_bbox_path = root_path + "disparity_with_bbox/";
    //dir for save pcd result
    string pcd_path = root_path + "pcd/";
    string frame_pc_path = pcd_path + "frame_pc/";
    string object_pc_before_cluster_path = pcd_path + "object_pc_before_cluster/";
    string object_pc_after_cluster_path = pcd_path + "object_pc_after_cluster/";
    string object_pc_path = pcd_path + "object_pc/";
    string object_pc_frame_path = pcd_path + "object_pc_frame/";
    //dir for object det result
    string object_det_result_path = root_path + "object_det_result/";

    string command = "mkdir -p " + left_rectified;
    system(command.c_str());
    command = "mkdir -p " + right_rectified;
    system(command.c_str());
    command = "mkdir -p " + pairs_rectified;
    system(command.c_str());
    command = "mkdir -p " + disparities;
    system(command.c_str());
    command = "mkdir -p " + disparity_with_bbox_path;
    system(command.c_str());
    command = "mkdir -p " + frame_pc_path;
    system(command.c_str());
    command = "mkdir -p " + object_pc_before_cluster_path;
    system(command.c_str());
    command = "mkdir -p " + object_pc_after_cluster_path;
    system(command.c_str());
    command = "mkdir -p " + object_pc_path;
    system(command.c_str());
    command = "mkdir -p " + object_pc_frame_path;
    system(command.c_str());
    command = "mkdir -p " + object_det_result_path;
    system(command.c_str());

    //step1 进行双目矫正、匹配得到深度图
    string imageList_L = root_path + "img/file_left.txt";
    string imageList_R = root_path + "img/file_right.txt";
    string calib_opencv_file =  root_path + "calib_img/stereocalibrateresult_L.txt";
    string calib_matlab_file = root_path + "calib_img/stereocalibrateresult_matlab.txt";
    string calib_file;
    string rectified_parameters = root_path + "calib_img/stereoRectifyParams.txt";

    Mat cameraMatrix_L = Mat(3, 3, CV_32FC1, Scalar::all(0)); // 相机的内参数
    Mat cameraMatrix_R = Mat(3, 3, CV_32FC1, Scalar::all(0)); // 初始化相机的内参数
    Mat distCoeffs_L = Mat(1, 5, CV_32FC1, Scalar::all(0)); // 相机的畸变系数
    Mat distCoeffs_R = Mat(1, 5, CV_32FC1, Scalar::all(0)); // 初始化相机的畸变系数
    Mat R, T;
    Size imageSize;
    Rect validRoi[2];//双目矫正有效区域
    // 立体校正参数
    Mat R1(3,3,CV_64FC1,Scalar::all(0));
    Mat R2(3,3,CV_64FC1,Scalar::all(0));
    Mat P1(3,4,CV_64FC1,Scalar::all(0));
    Mat P2(3,4,CV_64FC1,Scalar::all(0));
    Mat Q(4,4,CV_64FC1,Scalar::all(0));
    Mat mapl1, mapl2, mapr1, mapr2; // 图像重投影映射表

    //Step1 read camera parameters
    if (calib_method == "matlab")
    {
        calib_file = calib_matlab_file;
    }
    if (calib_method == "opencv")
    {
        calib_file = calib_opencv_file;
    }
    read_calib_parameters(calib_file, cameraMatrix_L, distCoeffs_L, cameraMatrix_R, distCoeffs_R,
                          R, T, imageSize);
//    cout<< R << endl<< T << endl << cameraMatrix_L << endl << distCoeffs_L << endl << cameraMatrix_R << endl<< distCoeffs_R<<endl;
    //Step2 get stereo rectified parameters
    validRoi[0], validRoi[1] = stereoRectification(cameraMatrix_L, distCoeffs_L, cameraMatrix_R, distCoeffs_R,
                                                   imageSize, R, T, R1, R2, P1, P2, Q, mapl1, mapl2, mapr1, mapr2);

    ifstream imageStore_L(imageList_L); // 打开存放测试图片名称的txt
    ifstream imageStore_R(imageList_R); // 打开存放测试图片名称的txt
    string imageName_L; // 读取的测试图片的名称
    string imageName_R; // 读取的测试图片的名称
    int count = 0 ;
    char object_det_result_file[30];
    // for each pair rectified images and generate sparity
    sprintf(object_det_result_file, "object_det_");
    ofstream fResult(object_det_result_path + object_det_result_file+calib_method+"_"+algorithm+".txt", ios::out);
    while (getline(imageStore_L, imageName_L))
    {


        getline(imageStore_R, imageName_R);
        cout<<"*************FRAME"<<count<<"*****************"<<endl;
        // Step3 rectified images
        Rectification(root_path, imageName_L, imageName_R, rectified_l, rectified_r, mapl1, mapl2, mapr1, mapr2, validRoi, count);
        cout<<"finish rectification"<<endl;
        // Step4 generate sparities
        computeDisparityImage(root_path, rectified_l,rectified_r, disparity, count, algorithm);
//        imshow("rectified_l", rectified_l);
//        if (waitKey(0) == 27) {
//            destroyAllWindows();
//        }
//        imshow("rectified_r", rectified_r);
//        if (waitKey(0) == 27) {
//            destroyAllWindows();
//        }
        cout<<"finish compute disparity image"<<endl;
        //Step5 读取二维检测结果
        vector<bbox> bbox_list_l, bbox_list_r;
        vector<bbox> bbox_list_after_screen_l, bbox_list_after_screen_r;
        read_2d_object(root_path, bbox_list_l, bbox_list_r, count);
        cout<<"finish read 2d objects"<<endl;
        if (bbox_list_l.empty() || bbox_list_r.empty())
        {
            cout<<"find no 2d object!"<<endl;
            fResult<<count<<" -1 0 0 0 0"<<endl;
            count=count+1;
            continue;
        }
        //Step6 双目二维检测结果筛选
        screen_2d_object(bbox_list_l, bbox_list_r, bbox_list_after_screen_l, bbox_list_after_screen_r);
        cout<<"finish screen 2d objects"<<endl;
        if (bbox_list_after_screen_l.empty() || bbox_list_after_screen_r.empty())
        {
            cout<<"find no 2d object after screen!"<<endl;
            fResult<<count<<" -1 0 0 0 0"<<endl;
            count=count+1;
            continue;
        }
        //for debug: skip some frame
//        if(count == 32 || count ==33 || count == 59 || count == 60|| count ==91)
//        if(count != 79)
//        {
//            fResult<<count<<" -1 0 0 0 0"<<endl;
//            count = count + 1;
//            continue;
//        }
        //Step7 结合深度图和二维检测结果进行三维物体检测
        //for visualization: draw bbox on sparity
        show_bbox_on_sparity(root_path, bbox_list_after_screen_l, disparity, count);
        cout<<"finish show bbox on sparity"<<endl;
        //Step8 利用视差图和投影参数Q得到三维图像
        cout<<"Q"<<endl<<Q<<endl;
        reprojectImageTo3D(disparity, result3DImage, Q);
//        imshow("disparity", disparity);
//        setMouseCallback("disparity", onMouse);
//        if (waitKey(0) == 27) {
//            destroyAllWindows();
//        }
        cout<<"finish reproject image to 3d"<<endl;
        //Step9 三维物体检测
        vector<box3d> bbox_3d_list;
        obj_det_3d(root_path, bbox_list_after_screen_l, rectified_l, result3DImage, bbox_3d_list, count, imageSize, P1);
        if(!bbox_3d_list.empty())
        {
            cout<<"finish 3d object detection"<<endl;
            cout<<"3d detection result"<<endl;
            for(int i = 0; i<bbox_3d_list.size();i++)
            {
                box3d b3d=bbox_3d_list[i];
                cout<<"class:"<< b3d._c <<", p_x: "<<b3d._position_x <<", p_y: "<<b3d._position_y<<", p_z: "<<b3d._position_z<<", 2d score: "<< b3d._score<<endl;
                fResult<<b3d._c<<" "<<b3d._position_x <<" "<<b3d._position_y<<" "<<b3d._position_z<<" "<< b3d._score<<endl;
            }
        }
        else
        {
            fResult<<count<<" -1 0 0 0 0"<<endl;
        }

        count = count + 1;
    }
    fResult.close();

    return 0;
}