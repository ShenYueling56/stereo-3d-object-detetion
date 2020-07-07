//
// Created by shenyl on 2020/7/2.
//

#ifndef OBJECT_DETECTION_BBOX2BOX3D_H
#define OBJECT_DETECTION_BBOX2BOX3D_H

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <map>
#include <stdlib.h>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <typeinfo>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>

#include "bbox.h"
#include "box3d.h"
#include "stereo_match.h"

using namespace cv;
using namespace std;


// 和随机数相关
#define N 999 //随机数精确到三位小数。
#define dif_cy_threadshold 20 //the biggest y dif for bbox pairs
#define w1 0.3 //weight for dist
#define w2 2 //weight for IoU
#define offset_x 0//0.05
#define offset_y 0
#define offset_z 0
#define scale_offset 1.0//1.05

Mat result3DImage;

bool read_2d_object(string root_path, vector<bbox>& bbox_list_l, vector<bbox>& bbox_list_r, int count);
bool screen_2d_object(vector<bbox> bbox_list_l, vector<bbox> bbox_list_r, vector<bbox>& bbox_list_after_screen_l, vector<bbox>& bbox_list_after_screen_r);
bool show_bbox_on_sparity(string root_path, vector<bbox> bbox_list_l, Mat disparity, int count);
bool cluster_pc(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_object, vector<pcl::PointCloud<pcl::PointXYZRGB>>& pc_clusters, int frame_index, int object_index, string object_pc_before_cluster_path);
bool trans2w(Mat Twc, box3d b3d, box3d& b3d_w);
cv::Mat Cal3D_2D(pcl::PointXYZRGB point3D, Mat projectionMatrix, Size imageSize);
float intersectRect(const cv::Rect rectA, const cv::Rect rectB, cv::Rect& intersectRect);
bool pick_cluster(bbox b, vector<pcl::PointCloud<pcl::PointXYZRGB>> pc_clusters, pcl::PointCloud<pcl::PointXYZRGB>& object_cluster, double object_point_num, Size imageSize, Mat projectionMatrix, Mat rectified_l, box3d& b3d);
void onMouse(int event, int x, int y, int flags, void *param);
bool obj_det_3d(string root_path, vector<bbox> bbox_list_l, Mat rectified_l, Mat result3DImage, vector<box3d>& pclist, int count, Size imageSize, Mat P1);

#endif //OBJECT_DETECTION_BBOX2BOX3D_H