//
// Created by shenyl on 2020/7/2.
//

#include "bbox2box3d.h"

static map<int, string> index2classMap;
static map<string, int> class2indexMap;

void split(string &s, vector<string> &list1)
{
    istringstream tmp_string(s);
    string ss;
    while (getline(tmp_string, ss, ' '))
    {
        list1.push_back(ss);
//        cout<<ss<<endl;
    }

}

bool read_2d_object(string root_path, vector<bbox>& bbox_list_l, vector<bbox>& bbox_list_r, int count)
{
    //be the same with .names file
    index2classMap.insert(pair<int, string>(0, "badminton"));
    index2classMap.insert(pair<int, string>(1, "wire"));
    index2classMap.insert(pair<int, string>(2, "mahjong"));
    index2classMap.insert(pair<int, string>(3, "dogshit"));
    class2indexMap.insert(pair<string,int>("badminton",0));
    class2indexMap.insert(pair<string,int>("wire", 1));
    class2indexMap.insert(pair<string,int>("mahjong", 2));
    class2indexMap.insert(pair<string,int>("dogshit", 3));

    string bbox_dir = root_path + "2d_objects/";
    char bbox_file_l[30], bbox_file_r[30];
    sprintf(bbox_file_l, "left/%06d.txt", count);
    sprintf(bbox_file_r, "right/%06d.txt", count);
    string bbox_dir_l = bbox_dir + bbox_file_l;
    string bbox_dir_r = bbox_dir + bbox_file_r;

    ifstream f_bbox_l(bbox_dir_l);
    ifstream f_bbox_r(bbox_dir_r);
    if (!f_bbox_l)
    {
        cout<<bbox_dir_l<<" is not exist"<<endl;
        return false;
    }
    if (!f_bbox_r)
    {
        cout<<bbox_dir_r<<" is not exist"<<endl;
        return false;
    }

    string line;

    while (getline(f_bbox_l, line)) //for each object
    {
        vector<string> list;
//        cout<<"left"<<endl;
//        cout<< line<<endl;
        split(line, list);
        bbox b(class2indexMap[list[0]], atof(list[1].c_str()), atoi(list[2].c_str()),
               atoi(list[3].c_str()), atoi(list[4].c_str()),
               atoi(list[5].c_str()));
//        cout<< class2indexMap[list[0]] << endl <<atof(list[1].c_str())<<endl<<atoi(list[2].c_str())<<endl
//            << atoi(list[3].c_str()) << endl <<atoi(list[4].c_str())<<endl<<atoi(list[5].c_str())<<endl;
        bbox_list_l.push_back(b);
    }
    while (getline(f_bbox_r, line)) //for each object
    {
        vector<string> list;
//        cout<<"right"<<endl;
//        cout<< line<<endl;
        split(line, list);
        bbox b(class2indexMap[list[0]], atof(list[1].c_str()), atoi(list[2].c_str()),
               atoi(list[3].c_str()), atoi(list[4].c_str()),
               atoi(list[5].c_str()));
//        cout<< class2indexMap[list[0]] << endl <<atof(list[1].c_str())<<endl<<atoi(list[2].c_str())<<endl
//            << atoi(list[3].c_str()) << endl <<atoi(list[4].c_str())<<endl<<atoi(list[5].c_str())<<endl;
        bbox_list_r.push_back(b);
    }
    cout<<"find "<<bbox_list_l.size()<<" 2d objects in left and "<<bbox_list_r.size()<<" 2d objects in right"<<endl;
    return true;
}

bool screen_2d_object(vector<bbox> bbox_list_l, vector<bbox> bbox_list_r, vector<bbox>& bbox_list_after_screen_l, vector<bbox>& bbox_list_after_screen_r)
{
    for (int i=0; i<bbox_list_l.size(); i++)
    {
        bbox bbox_l=bbox_list_l[i];
        bbox_l._index = i;
        int c =bbox_l._c;
//        if (c == 1)
//        {
//            continue;
//        }
        int lt_lx = bbox_l._xmin;
        int lt_ly = bbox_l._ymin;
        int rb_lx = bbox_l._xmax;
        int rb_ly = bbox_l._ymax;
        int center_lx = (lt_lx + rb_lx)/2;
        int center_ly = (lt_ly + rb_ly)/2;
        //todo: the 140 should be parameters, be related to the match parameter
        if (center_lx <= 100||lt_lx<80)//delete the object too left in left img where the disparity doesn't have data
        {
            cout<<"box is too left! the score in left img is: "<<bbox_l._score<<endl;
            continue;
        }
        if(bbox_l._score<0.3)
        {
            continue;
        }
        for (int j =0;j<bbox_list_r.size(); j++)
        {
            bbox bbox_r=bbox_list_r[j];
            if(bbox_r._score<0.3)
            {
                continue;
            }
            bbox_r._index = j;
            if (bbox_r._c == c)
            {
                int lt_rx = bbox_r._xmin;
                int lt_ry = bbox_r._ymin;
                int rb_rx = bbox_r._xmax;
                int rb_ry = bbox_r._ymax;
                int center_rx = (lt_rx + rb_rx)/2;
                int center_ry = (lt_ry + rb_ry)/2;
                if (abs(center_ry-center_ly)<dif_cy_threadshold && center_lx > center_rx) //find candidate pair
                {
                    bbox_l._pair_index = j;
                    bbox_r._pair_index = i;
                    cout<< "find pairs: "<<i<<","<<j<<endl;
                    cout<< c <<","<<bbox_r._c<<endl;
                    cout<<bbox_l._score<<","<<bbox_r._score<<endl;
                    bbox_list_after_screen_l.push_back(bbox_l);
                    bbox_list_after_screen_r.push_back(bbox_r);
                    break;
                }
            }
        }
    }
    return true;
}
bool show_bbox_on_sparity(string root_path, vector<bbox> bbox_list_l, Mat disparity, int count)
{
    string disparity_with_bbox_path = root_path + "disparity_with_bbox/";

    Mat disparity_bbox;
    disparity.copyTo(disparity_bbox);
    for (int i=0;i<bbox_list_l.size();i++)
    {
        bbox b= bbox_list_l[i];
        if (b._pair_index == -1) continue;
        Rect rect(b._xmin, b._ymin, b._xmax-b._xmin,b._ymax-b._ymin);//左上坐标（x,y）和矩形的长(x)宽(y)
        rectangle(disparity_bbox, rect, Scalar(255, 0, 0),1, LINE_8,0);

    }
//    imshow("disparity with bbox", disparity_bbox);
//    if (waitKey(0)==27)
//    {
//        destroyAllWindows();
//    }
    char disparity_with_bbox_file[30];
    sprintf(disparity_with_bbox_file, "%06d.jpg", count);
    imwrite(disparity_with_bbox_path + disparity_with_bbox_file, disparity_bbox);

    return true;
}

bool cluster_pc(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_object_ptr, vector<pcl::PointCloud<pcl::PointXYZRGB>>& pc_clusters, int frame_index, int object_index, string object_pc_before_cluster_path){

//    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_object_ptr (new pcl::PointCloud<pcl::PointXYZ>);
//    pc_object_ptr=pc_object.makeShared();
//    pcl::PCDWriter writer;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (pc_object_ptr);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (30);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (pc_object_ptr);
    ec.extract (cluster_indices);

    cout<<"finish cluster"<<endl;

    int j = 0;
    int p_index_clusters = 0;
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusters (new pcl::PointCloud<pcl::PointXYZRGB>);
//    clusters->width    = 10000;
//    clusters->height   = 1;
//    clusters->is_dense = false;  //不是稠密型的
//    clusters->points.resize (clusters->width * clusters->height);  //点云总数大小
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        pc_cluster->width    = 20000;
        pc_cluster->height   = 1;
        pc_cluster->is_dense = false;  //不是稠密型的
        pc_cluster->points.resize (pc_cluster->width * pc_cluster->height);  //点云总数大小
        int i=0;
        double random_r = rand()%(N+1)/(float)(N+1); //random color for each cluster
        double random_g = rand()%(N+1)/(float)(N+1); //random color for each cluster
        double random_b = rand()%(N+1)/(float)(N+1); //random color for each cluster
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
//            cout<<pc_object_ptr->points[*pit].x<<" "<<pc_object_ptr->points[*pit].y<<" "<<pc_object_ptr->points[*pit].z<<endl;
            pc_cluster->points[i].x = pc_object_ptr->points[*pit].x;
            pc_cluster->points[i].y = pc_object_ptr->points[*pit].y;
            pc_cluster->points[i].z = pc_object_ptr->points[*pit].z;
            pc_cluster->points[i].r = random_r*255;
            pc_cluster->points[i].g = random_g*255;
            pc_cluster->points[i++].b = random_b*255;
        }
        pc_clusters.push_back(*pc_cluster);
        j++;
//        *clusters = *clusters + *pc_cluster;
    }
//    char clusters_file[30];
//    sprintf(clusters_file, "clusters_frame%06d_object%03d.pcd", frame_index, object_index);
//    pcl::io::savePCDFileASCII (object_pc_before_cluster_path + clusters_file, *clusters); //将点云保存到PCD文件中
    return true;
}
bool trans2w(Mat Tcw, box3d b3d, box3d& b3d_w)
{
    Mat cur(4, 1, CV_64FC1, cv::Scalar(0));
    cur.at<double>(0) = b3d._position_x; // attention: double not float!!!!
    cur.at<double>(1) = b3d._position_y;
    cur.at<double>(2) = b3d._position_z;
    cur.at<double>(3) = 1.000000e+00;
    Mat Twc;
    invert(Tcw, Twc);
    Mat world_coord = Twc * cur;
    b3d_w._position_x = world_coord.at<double>(0);
    b3d_w._position_y = world_coord.at<double>(1);
    b3d_w._position_z = world_coord.at<double>(2);
    b3d_w._score = b3d._score;
    b3d_w._c = b3d._c;
    cout<<cur.at<double>(0)<<";"<<cur.at<double>(1)<<";"<<cur.at<double>(2)<<";"<<cur.at<double>(3)<<endl;
    cout<<b3d_w._position_x<<";"<<b3d_w._position_y<<";"<<b3d_w._position_z<<" "<<world_coord.at<double>(3)<<endl;
    return true;

}


cv::Mat Cal3D_2D(pcl::PointXYZRGB point3D, Mat projectionMatrix, Size imageSize){
    cv::Mat point2D(2, 1, CV_16UC1, cv::Scalar(0));
    cv::Mat cur(4, 1, CV_64FC1, cv::Scalar(0));  //点云的三维齐次坐标向量4×1
    cur.at<double>(0) = point3D.x; // attention: double not float!!!!
    cur.at<double>(1) = point3D.y;
    cur.at<double>(2) = point3D.z;
    cur.at<double>(3) = 1.000000e+00;

//    cout<<"P1"<<projectionMatrix<<endl;
//    cout<<"cur"<<cur<<endl;

    cv::Mat Image_coord = projectionMatrix * cur;   //激光点云投影在图像上的齐次坐标向量3×1
    double Image_coord0 = Image_coord.at<float>(0);
    double Image_coord1 = Image_coord.at<float>(1);
    double Image_coord2 = Image_coord.at<float>(2);

    int pos_col = round(Image_coord.at<double>(0) / Image_coord.at<double>(2));  //齐次坐标转化为非齐次坐标
    int pos_row = round(Image_coord.at<double>(1) / Image_coord.at<double>(2));  //即除去一个尺度标量


//    cout<<"Image_coord"<<Image_coord<<endl;

    if (pos_col >= imageSize.width)
        pos_col = imageSize.width;
    if(pos_row >= imageSize.height)
        pos_row = imageSize.height;
    if(pos_col < 0)
        pos_col = 0;
    if(pos_row < 0)
        pos_row = 0;//去除超出图像范围的投影点云坐标
    point2D.at<int>(0) = pos_col;
    point2D.at<int>(1) = pos_row;
    return point2D;
}

//计算IoU---intersectionPercent
float intersectRect(const cv::Rect rectA, const cv::Rect rectB, cv::Rect& intersectRect) {
    if (rectA.x > rectB.x + rectB.width) { return 0.; }
    if (rectA.y > rectB.y + rectB.height) { return 0.; }
    if ((rectA.x + rectA.width) < rectB.x) { return 0.; }
    if ((rectA.y + rectA.height) < rectB.y) { return 0.; }
    float colInt = std::min(rectA.x + rectA.width, rectB.x + rectB.width) - std::max(rectA.x, rectB.x);
    float rowInt = std::min(rectA.y + rectA.height, rectB.y + rectB.height) - std::max(rectA.y, rectB.y);
    float intersection = colInt * rowInt;
    float areaA = rectA.width * rectA.height;
    float areaB = rectB.width * rectB.height;
    float intersectionPercent = intersection / (areaA + areaB - intersection);

    intersectRect.x = std::max(rectA.x, rectB.x);
    intersectRect.y = std::max(rectA.y, rectB.y);
    intersectRect.width = std::min(rectA.x + rectA.width, rectB.x + rectB.width) - intersectRect.x;
    intersectRect.height = std::min(rectA.y + rectA.height, rectB.y + rectB.height) - intersectRect.y;
    return intersectionPercent;
}

bool pick_cluster(bbox b, vector<pcl::PointCloud<pcl::PointXYZRGB>> pc_clusters, pcl::PointCloud<pcl::PointXYZRGB>& object_cluster, double object_point_num, Size imageSize, Mat projectionMatrix, Mat rectified_l, box3d& b3d)
{
    //score = num , distance, reprojection_iou
    vector<double> scores;
    double max_score = 0;
    double p_x = 0;
    double p_y = 0;
    double p_z = 0;

    for (int i=0;i<pc_clusters.size();i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB> cluster = pc_clusters[i];
        cv::Mat point2D_this(2, 1, CV_16UC1, cv::Scalar(0));
        cv::Mat point2D_min(2, 1, CV_16UC1, cv::Scalar(0));
        point2D_min.at<int>(0) = 10000;
        point2D_min.at<int>(1) = 10000;
        cv::Mat point2D_max(2, 1, CV_16UC1, cv::Scalar(0));
        point2D_max.at<int>(0) = 0;
        point2D_max.at<int>(1) = 0;
        cv::Rect rect_bbox;
        cv::Rect rect_cluster;
        cv::Rect rect_intersect;
        double pointnum = 0;
        double ave_dist= 0;
        double projection_iou=0;
        double score=0;
        double ave_x = 0;
        double ave_y = 0;
        double ave_z = 0;
        double min_z = 10000;
        for (int j=0;j<cluster.points.size();j++)
        {
            if ((cluster.points[j].x ==0) && (cluster.points[j].y ==0) && (cluster.points[j].z ==0)) continue; //have many (0,0,0)
            pointnum = pointnum +1;
            ave_dist = ave_dist + sqrt(cluster.points[j].x * cluster.points[j].x + cluster.points[j].y * cluster.points[j].y);
            ave_x = ave_x + cluster.points[j].x;
            ave_y = ave_y + cluster.points[j].y;
            ave_z = ave_z + cluster.points[j].z;
//            cout<<"ave_dist"<<typeid(ave_dist).name()<<endl;
//            cout<<"ave_dist"<<ave_dist<<endl;
            // find point2D_min and point2D_max
//            cout<<"point 3d in cluster"<<endl;
//            cout << cluster.points[j].x << "," <<cluster.points[j].y<<","<<cluster.points[j].z<<endl;
            point2D_this = Cal3D_2D(cluster.points[j], projectionMatrix, imageSize);
//            cout<<"Point2d: "<<point2D_this.at<int>(0)<<","<<point2D_this.at<int>(1)<<endl;
            if(point2D_this.at<int>(0)<=point2D_min.at<int>(0))
                point2D_min.at<int>(0) = point2D_this.at<int>(0);
            if(point2D_this.at<int>(1)<=point2D_min.at<int>(1))
                point2D_min.at<int>(1) = point2D_this.at<int>(1);
            if(point2D_this.at<int>(0)>=point2D_max.at<int>(0))
                point2D_max.at<int>(0) = point2D_this.at<int>(0);
            if(point2D_this.at<int>(1)>=point2D_max.at<int>(1))
                point2D_max.at<int>(1) = point2D_this.at<int>(1);
            if(cluster.points[j].z<min_z)
                min_z = cluster.points[j].z;
        }
        // ave_dist
        ave_dist = ave_dist/pointnum;
        ave_x = (ave_x/pointnum)/scale_offset;
        ave_y = (ave_y/pointnum)/scale_offset;
        ave_z = (ave_z/pointnum)/scale_offset;
//        cout<<"point num"<<pointnum<<endl;
        //normalize point num
        pointnum = pointnum / object_point_num;

        // calculate IoU
        rect_cluster.x = point2D_min.at<int>(0);
        rect_cluster.y = point2D_min.at<int>(1);
        rect_cluster.width  = point2D_max.at<int>(0) - point2D_min.at<int>(0);
        rect_cluster.height = point2D_max.at<int>(1) - point2D_min.at<int>(1);
        rect_bbox.x = b._xmin;
        rect_bbox.y = b._ymin;
        rect_bbox.width = b._xmax - b._xmin;
        rect_bbox.height = b._ymax - b._ymin;
        projection_iou = intersectRect(rect_bbox, rect_cluster, rect_intersect);

//        cout<<"rect_cluster"<<endl<<rect_cluster.x<<","<<rect_cluster.y<<endl
//            <<rect_cluster.width<<","<<rect_cluster.height<<endl;

        //show project result
        rectangle(rectified_l, rect_bbox, Scalar(255, 0, 0),1, LINE_8,0);
        rectangle(rectified_l, rect_cluster, Scalar(0, 255, 0),1, LINE_8,0);
//        imshow("rectified_l with projected bbox", rectified_l);
//        if (waitKey(0)==27)
//        {
//            destroyAllWindows();
//        }

        score = pointnum + w1 / ave_dist + w2 * projection_iou;
        if (pointnum==0) score = 0;
        if (ave_z < 0)
        {
            score = 0;
            continue;
        }

        cout<<"p_x: "<<ave_x<<", p_y: "<<ave_y<<", p_z: "<<min_z<<" ,socre: "<<score <<" ,pointnum: "<<pointnum<<", ave_dist: "<< ave_dist <<", IoU: "<< projection_iou<<endl;
        scores.push_back(score);
        if (score>max_score)
        {
            max_score = score;
            object_cluster = cluster;
            p_x = ave_x-offset_x;
            p_y = ave_y-offset_y;
            p_z = min_z/scale_offset-offset_z;
        }
    }
    if(max_score == 0)
    {
        cout<<"can't find the best cluster"<<endl;
        return false;
    }
    if(p_z>2.5)
    {
        cout<<"too remote!!!"<<endl;
        return false;
    }
    b3d._position_x = p_x;
    b3d._position_y = p_y;
    b3d._position_z = p_z;
    b3d._c = b._c;
    b3d._score = b._score;
    cout<<"find the best cluster with score "<<max_score<<endl;
    cout<<"p_x: "<<p_x<<", p_y: "<<p_y<<", p_z: "<<p_z<<", class: "<<b._c<<", score: "<<b._score<<endl;
    return true;
}

// 鼠标回调函数，点击视差图显示深度
void onMouse(int event, int x, int y, int flags, void *param)
{
    Point point;
    point.x = x;
    point.y = y;
    if(event == EVENT_LBUTTONDOWN)
    {
        cout << result3DImage.at<Vec3f>(point) << endl;
    }
}


bool obj_det_3d(string root_path, vector<bbox> bbox_list_l, Mat rectified_l, Mat result3DImage, vector<box3d>& pclist, int count, Size imageSize, Mat P1)
{
    string pcd_path = root_path + "pcd/";
    string frame_pc_path = pcd_path + "frame_pc/";
    string object_pc_before_cluster_path = pcd_path + "object_pc_before_cluster/";
    string object_pc_after_cluster_path = pcd_path + "object_pc_after_cluster/";
    string object_pc_path = pcd_path + "object_pc/";
    string object_pc_frame_path = pcd_path + "object_pc_frame/";
    char frame_pc_file[30];
    char object_pc_before_cluster_file[30];
    char object_pc_after_cluster_file[30];
    char object_pc_file[30];
    char object_pc_frame_file[30];

//    pcl::PointCloud<pcl::PointXYZRGBA> pc_frame;
    pcl::PointCloud<pcl::PointXYZRGB> pc_object_frame;
//    pc_frame.width    = 10000;
//    pc_frame.height   = 1;
//    pc_frame.is_dense = false;  //不是稠密型的
//    pc_frame.points.resize (pc_frame.width * pc_frame.height);  //点云总数大小
    int p_index_frame = 0;
    for (int i=0;i<bbox_list_l.size();i++)
    {
        cout<<"/////////////object"<<i<<"//////////////////"<<endl;
        double random_r = rand()%(N+1)/(float)(N+1); //random color for each cluster
        double random_g = rand()%(N+1)/(float)(N+1); //random color for each cluster
        double random_b = rand()%(N+1)/(float)(N+1); //random color for each cluster
        bbox b= bbox_list_l[i];
//        cout<< "bbox pair index"<<endl<<b._pair_index<<endl;
        if (b._pair_index == -1) continue;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc_object_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        pc_object_ptr->width    = (b._xmax - b._xmin)*(b._ymax-b._ymin);
        pc_object_ptr->height   = 1;
        pc_object_ptr->is_dense = false;  //不是稠密型的
        pc_object_ptr->points.resize (pc_object_ptr->width * pc_object_ptr->height);  //点云总数大小

//        pcl::PointCloud<pcl::PointXYZ> pc_object;
//        pc_object.width    = (b._xmax - b._xmin)*(b._ymax-b._ymin);
//        pc_object.height   = 1;
//        pc_object.is_dense = false;  //不是稠密型的
//        pc_object.points.resize (pc_object.width * pc_object.height);  //点云总数大小
        int p_index_object=0;
        for (int x = b._xmin;  x < b._xmax; x++) {
            for (int y = b._ymin; y < b._ymax; y++) {
                Point point;
                point.x = x;
                point.y = y;
//                cout << x <<"," << y << endl;
                if (result3DImage.at<Vec3f>(point)[2]<0)   //abnormal points
                {
//                    pc_frame.points[p_index_frame].x = 0; //change to (m)
//                    pc_frame.points[p_index_frame].y = 0;
//                    pc_frame.points[p_index_frame].z = 0;
//                    pc_frame.points[p_index_frame].a = i; // use intensity to hold the object index
//                    pc_frame.points[p_index_frame].r = random_r*255;
//                    pc_frame.points[p_index_frame].g = random_g*255;
//                    pc_frame.points[p_index_frame++].b = random_b*255;

                    pc_object_ptr->points[p_index_object].x = 0;
                    pc_object_ptr->points[p_index_object].y = 0;
                    pc_object_ptr->points[p_index_object++].z = 0;
//                cout<<"finish pc_object"<<endl;
                    continue;
                }
                if (result3DImage.at<Vec3f>(point)[2]>2500)
                {
//                    pc_frame.points[p_index_frame].x = 0; //change to (m)
//                    pc_frame.points[p_index_frame].y = 0;
//                    pc_frame.points[p_index_frame].z = 0;
//                    pc_frame.points[p_index_frame].a = i; // use intensity to hold the object index
//                    pc_frame.points[p_index_frame].r = random_r*255;
//                    pc_frame.points[p_index_frame].g = random_g*255;
//                    pc_frame.points[p_index_frame++].b = random_b*255;

                    pc_object_ptr->points[p_index_object].x = 0;
                    pc_object_ptr->points[p_index_object].y = 0;
                    pc_object_ptr->points[p_index_object++].z = 0;
                    continue;
                }
//                cout << result3DImage.at<Vec3f>(point)[0] << ","
//                     << result3DImage.at<Vec3f>(point)[1] << ","
//                     << result3DImage.at<Vec3f>(point)[2] << endl;

//                pc_frame.points[p_index_frame].x = result3DImage.at<Vec3f>(point)[0]/1000; //change to (m)
//                pc_frame.points[p_index_frame].y = result3DImage.at<Vec3f>(point)[1]/1000;
//                pc_frame.points[p_index_frame].z = result3DImage.at<Vec3f>(point)[2]/1000;
//                pc_frame.points[p_index_frame].a = i; // use intensity to hold the object index
//                pc_frame.points[p_index_frame].r = random_r*255;
//                pc_frame.points[p_index_frame].g = random_g*255;
//                pc_frame.points[p_index_frame++].b = random_b*255;
////                cout<<"finish pc_frame "<<endl;

                pc_object_ptr->points[p_index_object].x = result3DImage.at<Vec3f>(point)[0]/1000;
                pc_object_ptr->points[p_index_object].y = result3DImage.at<Vec3f>(point)[1]/1000;
                pc_object_ptr->points[p_index_object++].z = result3DImage.at<Vec3f>(point)[2]/1000;
//                cout<<"finish pc_object"<<endl;

            }
        }
        cout<<"pc_object size"<<pc_object_ptr->points.size()<<endl;
        // filtering cloud by passthrough
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc_object_filtered_ptr (new pcl::PointCloud<pcl::PointXYZ>);
//        pc_object_filtered_ptr->width    = 10000;
//        pc_object_filtered_ptr->height   = 1;
//        pc_object_filtered_ptr->is_dense = false;  //不是稠密型的
//        pc_object_filtered_ptr->points.resize (pc_object_filtered_ptr->width * pc_object_filtered_ptr->height);  //点云总数大小
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (pc_object_ptr);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.45, 2.3);
        //pass.setFilterLimitsNegative (true);
        pass.filter (*pc_object_filtered_ptr);
        //

        sprintf(object_pc_before_cluster_file, "frame%06d_object%02d.pcd", count, i);
        pcl::io::savePCDFileASCII (object_pc_before_cluster_path + object_pc_before_cluster_file, *pc_object_ptr); //将点云保存到PCD文件中
        // cluster the pc_object
        cout<<"start cluster"<<endl;
        vector<pcl::PointCloud<pcl::PointXYZRGB>> pc_clusters;
        auto start = std::chrono::steady_clock::now();
        cluster_pc(pc_object_filtered_ptr, pc_clusters, count, i, object_pc_after_cluster_path);
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double, std::micro> elapsed = end - start; // std::micro 表示以微秒为时间单位
        std::cout<< "cluster time for frame "<< count << " object " << i  <<": "<< elapsed.count()/1000000<< "s" << std::endl;

        // find the best cluster for the object
        int object_point_num = pc_object_ptr->points.size();
        pcl::PointCloud<pcl::PointXYZRGB> object_cluster;
        box3d b3d;
        if(pick_cluster(b, pc_clusters, object_cluster,object_point_num, imageSize, P1, rectified_l, b3d))
        {
            cout<<"find the object cluster"<<endl;
            pc_object_frame = pc_object_frame + object_cluster;
            sprintf(object_pc_file, "frame%06d_object%02d.pcd", count, i);
            pcl::io::savePCDFileASCII (object_pc_path + object_pc_file, object_cluster); //将点云保存到PCD文件中
            pclist.push_back(b3d);
        }
        else
        {
            cout<<"can not find the object cluster"<<endl;
        }
    }
//    cout<<"pc frame num:"<<pc_frame.points.size()<<endl;
//    sprintf(frame_pc_file, "%06d.pcd", count);
//    pcl::io::savePCDFileASCII (frame_pc_path + frame_pc_file, pc_frame); //将点云保存到PCD文件中
    //put best cluster in object_pc
    sprintf(object_pc_frame_file, "%06d.pcd", count);
    pcl::io::savePCDFileASCII (object_pc_frame_path + object_pc_frame_file, pc_object_frame); //将点云保存到PCD文件中
    return true;
}

