//
// Created by lucasyu on 18-11-29.
//
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64.h>
#include <opencv2/opencv.hpp> 
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "/home/ubuntu/catkin_ws/devel/include/darknet_ros_msgs/BoundingBoxes.h"
#include <random>
#include <cmath>

#include <iostream>
#include <string>
#include <fstream>
#include <array>
#include <time.h>
#include <numeric>


using namespace std;
std::vector< std::vector<float> > data_pcl;
std::vector< std::vector<float> > data_uav;
std::vector< std::vector<float> > data_label;

ros::Time timestamp;
float vel_odom = 0.0f;
float angular_odom = 0.0f;
float position_odom_x = 0.0f;
float position_odom_y = 0.0f;
float position_odom_z = 0.0f;
float position_radar_x = 0.0f;
float position_radar_y = 0.0f;
float vel_smoother = 0.0f;
float angular_smoother = 0.0f;
float vel_teleop = 0.0f;
float angular_teleop = 0.0f;
float pos_target_x = 0.0f;
float pos_target_y = 0.0f;
float yaw_target = 0.0f;
float yaw_current = 0.0f;
float yaw_current_x = 0.0f;
float yaw_current_y = 0.0f;
float yaw_delt = 0.0f;
float yaw_forward = 0.0f;
float yaw_backward = 0.0f;
float yaw_leftward = 0.0f;
float yaw_rightward = 0.0f;

// std::vector<int> v_rgb_one_line;
static bool isReceivedRGB = false;
static bool isRotated = false;
static bool isReceivedDepth = false;

std::vector<float> v_depth_one_line;
std::vector<float> v_dep_noi_one_line;

const int num_uavdata = 17;
const int num_label = 4;
const int img_width = 64;
//const int img_height = 64;
const int img_height_uplimit = 20;
const int img_height_downlimit = 4;
const int info_dim = 1;
const int resize_rgb_width = 256;
const int resize_rgb_height = 192;


//width and height before resize:
const int IMGWIDTH = 640;
const int IMGHEIGHT = 480;

const float thres_gaussian = 0.5;

// average theta for the gaussian noise 
const float theta_mean = 3.1415926 / 6;
static cv::Mat RGB_dst(resize_rgb_width, resize_rgb_height, CV_8UC1, cv::Scalar::all(0));
static cv::Mat RGB_dst_origin(resize_rgb_width, resize_rgb_height, CV_8UC1, cv::Scalar::all(0));
int counter_semantics = 0;

unsigned char semantic_labels[IMGHEIGHT][IMGWIDTH];
unsigned char semantic_labels_resized[resize_rgb_height][resize_rgb_width];

ofstream outFile_rgbdata;
ofstream outFile_rgbnoidata;
ofstream outFIle_depthdata;
ofstream outFile_uavdata;
ofstream outFile_labels;
ofstream outFile_semantics;
ofstream outFile_depnoise;

void rotate_x_y(const float& x_origin, const float& y_origin, float& x_rotate, float& y_rotate, const float& yaw_angle)
{
    x_rotate = x_origin * cos(yaw_angle) + y_origin * sin(yaw_angle);
    y_rotate = y_origin * cos(yaw_angle) - x_origin * sin(yaw_angle);
}

void ContourGaussianNoise(cv::Mat& image_m, cv::Mat& dst_depth_bg_pepper, cv::Mat& depth_image_uint8, cv::Mat& dst_depth_origin)
{
    int nr = image_m.rows;
    int nc = image_m.cols;
    cv::Mat noise_z_m_uint8(nr, nc, CV_8UC1, cv::Scalar::all(0));
    //  parameters for the gaussian noise at the contour
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0, 1.0);

    std::default_random_engine engine;
    std::uniform_real_distribution<double> uniform(0.5,2.5);

    for(int i=0; i<nr ;i++){
        float *inDepth_m = image_m.ptr<float>(i); // float
        uchar* dep_noi = noise_z_m_uint8.ptr<uchar>(i);
        for(int j=0; j<nc; j++){
            float sigma_z_m = 0.0012 + 0.0019 * (inDepth_m[j] - 0.4) * (inDepth_m[j] - 0.4);

            float randn = distribution(generator);
            float black_shift = uniform(engine);
            float rand_shift = randn * sigma_z_m + inDepth_m[j] - black_shift;

            if (rand_shift > 4.5 || rand_shift != rand_shift || rand_shift < 0){
                rand_shift = 0.0;
            //     float random_pepper = rand()%100/(double)101;
            //     if (random_pepper < 0.05f){
            //         rand_shift = 4.5f;
            //     }
            //     else{
            //         rand_shift = 0.0;
            //     }
            // }
            // else if (rand_shift < 0){
            //     rand_shift = 0;
            }
            dep_noi[j] = (uchar)floor(rand_shift * 56); // 56 = 256/4.5
        }
    }

    //  find contours by canny
    cv::Mat contours;
    cv::Canny(dst_depth_origin, contours, 20, 100, 3);

    vector<vector<cv::Point2i>> contours_bin;
    vector<cv::Vec4i> hierarchy;
    
    // transform canny contours to lists of contours
    cv::findContours(contours, contours_bin, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    //draw contours
    float noise_widths[5] = {1.0, 2.0, 3.0, 4.0, 5.0};
    int rd_indx = (int)rand() % 5;
    float noise_width = noise_widths[rd_indx];
    cv::drawContours(contours, contours_bin, -1, 255, noise_width);
    //  the contours with noise
    cv::Mat contourWiderNoiseUint8, contourNot, image_without_contour, image_with_contour_noise;

    cv::bitwise_and(noise_z_m_uint8, contours, contourWiderNoiseUint8);
    cv::bitwise_not(contours, contourNot);
    cv::bitwise_and(contourNot, dst_depth_origin, image_without_contour);
    cv::add(image_without_contour, contourWiderNoiseUint8, depth_image_uint8);
    cv::add(dst_depth_bg_pepper, depth_image_uint8, depth_image_uint8);
 
    // cv::imshow("noise_z_m_uint8", dst_depth_bg_pepper);
    // cv::waitKey();

    // cv::imshow("noise_z_m_uint8", noise_z_m_uint8);
    // cv::waitKey();

    // cv::imshow("contours", contours);
    // cv::waitKey();

    // cv::imshow("contourWiderNoiseUint8", contourWiderNoiseUint8);
    // cv::waitKey();

    // cv::imshow("image_without_contour", image_without_contour);
    // cv::waitKey();

    // cv::imshow("depth_image_uint8", depth_image_uint8);
    // cv::waitKey();

    return;
}
    

void SaltAndPepper(cv::Mat& src, float& percetage)
{
    std::default_random_engine generator;
    std::normal_distribution<double> distribution_x(0.0, resize_rgb_height/24.0);
    std::normal_distribution<double> distribution_y(0.0, resize_rgb_width/24.0);
    int nc = src.cols;
    int nr = src.rows;

    int SP_NoiseNum = int(percetage * nr * nc);
    for (int i =0; i< SP_NoiseNum; i++)
    {
        double randX; 
        randX = distribution_x(generator);
        double randY; 
        randY = distribution_y(generator);

        randX = max(randX, - resize_rgb_height / 2.0);
        randX = min(randX, resize_rgb_height / 2.0);

        randY = max(randY, - resize_rgb_width / 2.0);
        randY = min(randY, resize_rgb_width / 2.0);

        int margin_y = rand() %resize_rgb_width;
        int margin_x = rand() %resize_rgb_height;

        if (randX <= 0){
            randX = resize_rgb_height + randX;
        }
        if (randY <= 0){
            randY = resize_rgb_width + randY;
        }
        int randX_index, randY_index;
        randX_index = int(randX);
        randY_index = int(randY);

        int random_integers[3] = {0, 1, 2};
        int rand_index = rand() % 3;
        if (randX_index == src.rows || src.rows < randX_index)
        {
            cout << "error in randX_index! bigger than the row number of the image!!" << endl;
        }

        if (random_integers[rand_index] == 0 || random_integers[rand_index] == 1)
        {
            uchar *inDepth_pepper = src.ptr<uchar>(randX_index);
            inDepth_pepper[margin_y] = 0;
        }

        if (random_integers[rand() % 3] == 0 || random_integers[rand_index] == 1)
        {
            uchar *inDepth = src.ptr<uchar>(margin_x);
            inDepth[randY_index] = 0;
        }
    }
    // cv::imshow("pepper in the sides", src);
    // cv::waitKey();
    return ;
}

void writeCsv(const std::vector<std::vector<float>> vec, const string& filename)
{
    // 写文件
    ofstream outFile;
    outFile.open(filename, ios::out); // 打开模式可省略
    for(int i_vec=0; i_vec<vec.size(); ++i_vec) {
        for (int j=0; j<vec[i_vec].size(); ++j)
        {
            char c[20];
            sprintf(c, "%f", vec[i_vec][j]);
            outFile << c << ",";
        }
        outFile << endl;
    }

    outFile.close();
}

void writeLabelData(const std::vector<float> labels)
{
    char c[50];
    for (int j=0; j<labels.size(); ++j)
    {
        sprintf(c, "%f", labels[j]);
        outFile_labels << c << ",";
    }
    outFile_labels << endl;
}

void writeCsvOneLine(const std::vector<float> uav_data, const std::vector<float> labels)
{
    char c[20];
    for (int j=0; j<uav_data.size(); ++j)
    {
        sprintf(c, "%f", uav_data[j]);
        outFile_uavdata << c << ",";
    }

    outFile_uavdata << endl;

    for (int j=0; j<labels.size(); ++j)
    {
        sprintf(c, "%f", labels[j]);
        outFile_labels << c << ",";
    }
    outFile_labels << endl;
}


void writeCsvOneLineFromCVMatUint8(cv::Mat& Mat, ofstream& outFile, const int& channel_num)
{
    if (channel_num == 1)
    {
        for (int i=0; i<resize_rgb_height; i++)
        {
            for (int j=0; j<resize_rgb_width; j++)
            {
                char ch[100];
                sprintf(ch, "%d", Mat.at<unsigned char>(i, j));
                outFile << ch << ",";
            }
        }
        outFile << endl;
    }
    else if (channel_num == 3)
    {
        for (int i=0; i<resize_rgb_height; i++) {
            for (int j=0; j<resize_rgb_width; j++) {
                char c_bgr[100];
                int color_b = Mat.at<cv::Vec3b>(i, j)[0]; 
                int color_g = Mat.at<cv::Vec3b>(i, j)[1]; 
                int color_r = Mat.at<cv::Vec3b>(i, j)[2]; 
                sprintf(c_bgr, "%d", color_b);
                outFile << c_bgr << ",";
                sprintf(c_bgr, "%d", color_g);
                outFile << c_bgr << ",";
                sprintf(c_bgr, "%d", color_r);
                outFile << c_bgr << ",";
            }
        }
        outFile << endl;
    }
    
}


void imageCb_d(const sensor_msgs::ImageConstPtr& depth_msg)
{
    cv_bridge::CvImagePtr depth_ptr, depth_ptr_cp;
    isReceivedDepth = false;
    try
    {
        depth_ptr = cv_bridge::toCvCopy(depth_msg);//, sensor_msgs::image_encodings::TYPE_64FC1);
        depth_ptr_cp = cv_bridge::toCvCopy(depth_msg);//, sensor_msgs::image_encodings::TYPE_64FC1);
        cv::Mat depth_m_origin = depth_ptr->image; //original image in m
        // cv::Mat depth_img_origin = depth_ptr_cp->image;
        
        int nr = depth_m_origin.rows;
        int nc = depth_m_origin.cols;
        cv::Mat depth_int_bg_pepper(nr, nc, CV_8UC1, cv::Scalar::all(0)); // background with pepper noise
        // cv::Mat depth_int(nr, nc, CV_8UC1, cv::Scalar::all(0));
        cv::Mat depth_int_origin(nr, nc, CV_8UC1, cv::Scalar::all(0)); //original image in uint8
        // cv::Mat depth_mm(nr, nc, CV_32FC1, cv::Scalar::all(0));
        for(int i=0; i<nr ;i++)
        {
            // float *inDepth_origin = depth_img_origin.ptr<float>(i);
            // float *inDepth_mm = depth_mm.ptr<float>(i);
            // uchar* inDepth_uint = depth_int.ptr<uchar>(i);
            float *inDepth = depth_m_origin.ptr<float>(i); //original image in float
            uchar* inDepth_bg_pepper = depth_int_bg_pepper.ptr<uchar>(i);
            uchar* inDepth_uint_origin = depth_int_origin.ptr<uchar>(i);
            
            float random_thres = rand()%20 * 0.001;

            for(int j=0; j<nc; j++)
            {
                // int depth_normalized = inDepth[j];
                // cout << "value: " << inDepth[j] << endl;
                float bg_noise = 0.0f;
                if (inDepth[j] > 4.5 || inDepth[j] != inDepth[j])
                {   
                    // inDepth_origin[j] = 0.0f; // original image, without pepper in the background
                    inDepth[j] = 0.0f;
                    float random_pepper = rand()%1000/(double)1001;
                    if (random_pepper < random_thres)
                    {
                        bg_noise = 4.5f;
                    }
                    else
                    {
                        bg_noise = 0.0f;
                    }
                    // bg_noise = inDepth[j];
                }
                // else{
                //     inDepth_uint[j] = 0;
                // }
                // inDepth_uint[j] = (uchar)floor(inDepth[j] * 56); 
                inDepth_bg_pepper[j] = (uchar)floor(bg_noise * 56); // 56 = 256/4.5
                inDepth_uint_origin[j] = (uchar)floor(inDepth[j] * 56);
                // inDepth_mm[j] = inDepth[j];// * 1000.0; //m -> mm

            }
        }
        

        cv::Mat dst_depth(resize_rgb_width, resize_rgb_height, CV_8UC1, cv::Scalar::all(0));
        cv::Mat dst_depth_origin, dst_depth_bg_pepper, dst_depth_m;
        // cv::resize(depth_int, dst_depth, cv::Size(resize_rgb_width, resize_rgb_height));
        cv::resize(depth_int_bg_pepper, dst_depth_bg_pepper, cv::Size(resize_rgb_width, resize_rgb_height));
        cv::resize(depth_int_origin, dst_depth_origin, cv::Size(resize_rgb_width, resize_rgb_height));
        cv::resize(depth_m_origin, dst_depth_m, cv::Size(resize_rgb_width, resize_rgb_height), cv::INTER_LINEAR);

        double proba = ((double)rand())/RAND_MAX;

        ContourGaussianNoise(dst_depth_m, dst_depth_bg_pepper, dst_depth, dst_depth_origin);

        float pepper_percentage = ((float)rand()) / RAND_MAX / 2.0;
        SaltAndPepper(dst_depth, pepper_percentage);

        cv::Mat dst_depth_rotate = dst_depth;
        cv::Mat dst_RGB_rotate = RGB_dst;
        // cv::imshow("before translation", dst_depth_rotate);
        // cv::waitKey();

        // cv::imshow("RGB before translation", dst_depth_rotate);
        // cv::waitKey();
        // dst_rotate_no_noise = dst_depth_cp;
        isReceivedDepth = true;
        if (rand()%100 / (double)101 < 0.2f) //rotation transformation
        {
            double rotation_angles[16] = {-1.8, -1.6, -1.4, -1.2, -1.0, -0.8, -0.5, -0.3, 0.3, 0.5, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8};

            int angle_index = (int)rand() % 16;
            double rotation_angle = rotation_angles[angle_index];
         
            cv::Size src_sz = dst_depth.size();
            cv::Size dst_sz(src_sz.width, src_sz.height);
            int len = std::max(dst_depth.cols, dst_depth.rows);
            
            //rotation center
            cv::Point2f center(len / 2., len / 2.);
            
            //rotation matrix (2*3)
            cv::Mat rot_mat = cv::getRotationMatrix2D(center, rotation_angle, 1.0);

            //rotate
            cv::warpAffine(dst_depth, dst_depth_rotate, rot_mat, dst_sz);
            // cv::warpAffine(dst_depth_cp, dst_rotate_no_noise, rot_mat, dst_sz);
            cv::warpAffine(RGB_dst, dst_RGB_rotate, rot_mat, dst_sz);

            isRotated = true;

            if (rand()%100 / (double)101 < 0.5f) // translation transformation
            {
                cv::Mat dst_translation;
                cv::Mat dst_translation_RGB;
                // //translation matrix definition
                cv::Mat t_mat =cv::Mat::zeros(2, 3, CV_32FC1);
                t_mat.at<float>(0, 0) = 1;
                t_mat.at<float>(0, 2) = 0; //horizontal translation
                t_mat.at<float>(1, 1) = 1;

                // random vertical translation
                float vertical_translations[10] = {-6.0, -5.0, -4.0, -3.0, -2.0, 2.0, 3.0, 4.0, 5.0, 6.0};
                int vertical_trans_index = (int)rand() % 10;
                float vertical_trans = vertical_translations[vertical_trans_index];
                t_mat.at<float>(1, 2) = vertical_trans; //vertical translation

                cv::warpAffine(dst_depth_rotate, dst_translation, t_mat, dst_sz);
                cv::warpAffine(dst_RGB_rotate, dst_translation_RGB, t_mat, dst_sz);
                dst_depth_rotate = dst_translation;
                dst_RGB_rotate = dst_translation_RGB;
                // cout << "imshow" << endl;
            }
        }
        
        // cv::imshow("original", dst_depth_origin);
        // cv::waitKey();

        // cv::imshow("translation result", dst_depth_rotate);
        // cv::waitKey();

        // cv::imshow("RGB translation result", dst_RGB_rotate);
        // cv::waitKey();

        if (!isReceivedRGB || !isReceivedDepth)
        {
            return;
        }
        else{
            //write semantics into csv files:
            for (int i=0; i<resize_rgb_height; i++)
            {
                for (int j=0; j<resize_rgb_width; j++)
                {
                    char c_semantics[100];
                    sprintf(c_semantics, "%d", semantic_labels_resized[i][j]);
                    outFile_semantics << c_semantics << ",";
                }
            }
            outFile_semantics << endl;

            // clear the semantic data to all zeros
            counter_semantics ++;
            if (counter_semantics > 3)
            {
                for (int i=0; i<resize_rgb_height;i++){
                    for (int j=0; j<resize_rgb_width;j++){
                        semantic_labels_resized[i][j] = 0;
                    }
                }
                counter_semantics = 0;
            }
            
            //write rgb data into csv files:
            writeCsvOneLineFromCVMatUint8(RGB_dst_origin, outFile_rgbdata, 3);

            //write rgb data WITH ROTATION into csv files:
            writeCsvOneLineFromCVMatUint8(dst_RGB_rotate, outFile_rgbnoidata, 3);

            //write depth data into csv files:
            writeCsvOneLineFromCVMatUint8(dst_depth_origin, outFIle_depthdata, 1);

            //write depth data with noise into csv files:
            writeCsvOneLineFromCVMatUint8(dst_depth, outFile_depnoise, 1);

            //write label data into csv files:
            float tmp_uav[num_uavdata] = {position_odom_x, position_odom_y, vel_odom, angular_odom, position_radar_x, 
                                    position_radar_y, pos_target_x, pos_target_y, yaw_target, yaw_current, yaw_current_x, 
                                    yaw_current_y, yaw_delt, yaw_forward, yaw_backward, yaw_leftward, yaw_rightward};
            std::vector<float>data_uav_tmp;
            data_uav_tmp.insert(data_uav_tmp.begin(), tmp_uav, tmp_uav + num_uavdata);

            float tmp_label[num_label] = {vel_smoother, angular_smoother, vel_teleop, angular_teleop};
            std::vector<float>data_label_tmp;
            data_label_tmp.insert(data_label_tmp.begin(), tmp_label, tmp_label+num_label);
            writeCsvOneLine(data_uav_tmp, data_label_tmp);

            // if the depth image is rotated, the data set should be expanded:
            if (isRotated)
            {
                //write semantics into csv files:
                for (int i=0; i<resize_rgb_height; i++)
                {
                    for (int j=0; j<resize_rgb_width; j++)
                    {
                        char c_semantics[100];
                        sprintf(c_semantics, "%d", semantic_labels_resized[i][j]);
                        outFile_semantics << c_semantics << ",";
                    }
                }
                outFile_semantics << endl;

                //write depth data with noise and rotation into csv files:
                writeCsvOneLineFromCVMatUint8(dst_depth_rotate, outFile_depnoise, 1);

                //write depth data into csv files:
                writeCsvOneLineFromCVMatUint8(dst_depth_origin, outFIle_depthdata, 1);

                //write rgb data into csv files:
                writeCsvOneLineFromCVMatUint8(RGB_dst_origin, outFile_rgbdata, 3);

                //write rgb data with rotation into csv files:
                writeCsvOneLineFromCVMatUint8(dst_RGB_rotate, outFile_rgbnoidata, 3);

                //write label data into csv files:
                writeCsvOneLine(data_uav_tmp, data_label_tmp);
            }

            return;
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}


void objectsCallback(const darknet_ros_msgs::BoundingBoxes& objects)
{
    /*initialize labels*/
    for(int i = 0; i < IMGHEIGHT; i++)
    {
        for(int j = 0; j < IMGWIDTH; j++)
        {
            semantic_labels[i][j] = 0; // Set all points to common obstacles when initialzing, including NAN. NAN will not be inserted into map, so it doesn't matter.
        }
    }

    for (int i=0; i<resize_rgb_height;i++){
        for (int j=0; j<resize_rgb_width;j++){
            semantic_labels_resized[i][j] = 0;
        }
    }

    /* Set labels of each roi. If there are overlaps, only take the last input label.*/
    for (int m = 0; m < objects.bounding_boxes.size(); m++)
    {
        int label;
        /*
        0: free space
        1: unknown
        2: possible way
        3: obstacle
        4: none
        5: furniture
        6: other dynamic objects
        7: person (sometimes detected as toothbrush)
        */
        if(objects.bounding_boxes[m].Class == "person")
            label = 7;
        else if(objects.bounding_boxes[m].Class == "toothbrush")
            label = 7;
        else if(objects.bounding_boxes[m].Class == "cat")
            label = 6;
        else if(objects.bounding_boxes[m].Class == "dog")
            label = 6;
        else if(objects.bounding_boxes[m].Class == "laptop")
            label = 5;
        else if(objects.bounding_boxes[m].Class == "bed")
            label = 5;
        else if(objects.bounding_boxes[m].Class == "tvmonitor")
            label = 5;
        else if(objects.bounding_boxes[m].Class == "chair")
            label = 5;
        else if(objects.bounding_boxes[m].Class == "diningtable")
            label = 5;
        else if(objects.bounding_boxes[m].Class == "sofa")
            label = 5;
        else if(objects.bounding_boxes[m].Class == "book")
            label = 5;
        else if(objects.bounding_boxes[m].Class == "traffic light")
            label = 5;
        else 
            label = 3;

        if (label < 4)
        {
            continue;
        }
            
        unsigned int range_x_min = objects.bounding_boxes[m].xmin;
        unsigned int range_x_max = objects.bounding_boxes[m].xmax;
        unsigned int range_y_min = objects.bounding_boxes[m].ymin;
        unsigned int range_y_max = objects.bounding_boxes[m].ymax;

        if(range_x_min > IMGWIDTH - 1) range_x_min = IMGWIDTH - 1;
        if(range_x_min < 0) range_x_min = 0;
        if(range_x_max > IMGWIDTH - 1) range_x_max = IMGWIDTH - 1;
        if(range_x_max < 0) range_x_max = 0;
    
        // For object labels
        
        for(int i = range_y_min - 1; i < range_y_max ; i++)
        {
            for(int j = range_x_min - 1; j < range_x_max; j++)
            {
                semantic_labels[i][j] = floor(label * 32);
            }
        }
    }
    //resize process from C++ array to cv Mat
    cv::Mat semantic_img(IMGHEIGHT, IMGWIDTH, CV_8UC1, (unsigned char*)semantic_labels);
    cv::Mat dst_semantics;
    cv::resize(semantic_img, dst_semantics, cv::Size(resize_rgb_width, resize_rgb_height));
    int nr = dst_semantics.rows;
    int nc = dst_semantics.cols;
    // cout << "semantic resize size: " << nr << ", " << nc << endl;
    for (int i=0; i<nr; i++)
    {
        unsigned char* inSemantic = dst_semantics.ptr<unsigned char>(i);
        for(int j=0; j<nc; j++)
        {
            semantic_labels_resized[i][j] = inSemantic[j];
        }
    }
}


void callbackRGB(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr cv_ptr_origin;
    isReceivedRGB = false;
    try 
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv_ptr_origin = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        // cv::Mat dst;
        cv::resize(cv_ptr->image, RGB_dst, cv::Size(resize_rgb_width, resize_rgb_height));
        cv::resize(cv_ptr_origin->image, RGB_dst_origin, cv::Size(resize_rgb_width, resize_rgb_height));
        isReceivedRGB = true;

        // outFile_rgbdata << endl;
        return;
    }
    catch (cv_bridge::Exception& e)
    {   
         ROS_ERROR("cv_bridge exception: %s", e.what());
         return;
    }

}

void callBackOdom(const nav_msgs::OdometryConstPtr& odom)
{
    position_odom_x = odom->pose.pose.position.x;
    position_odom_y = odom->pose.pose.position.y;
    vel_odom = odom->twist.twist.linear.x / 0.8f;
    angular_odom = odom->twist.twist.angular.z / 0.8f;

}

void callBackRadar(const geometry_msgs::Point::ConstPtr& data)
{
    pos_target_x = data->x;
    pos_target_y = data->y;
}

void callBackTeleop(const geometry_msgs::Twist::ConstPtr& data)
{
    vel_teleop = data->linear.x / 0.8f;
    angular_teleop = data->angular.z / 0.8f;
}

void callBackCmdMobileBase(const geometry_msgs::Twist::ConstPtr& data)
{
    vel_smoother = data->linear.x / 0.8f;
    angular_smoother = data->angular.z / 0.8f;
}

void callBackTargetPos(const geometry_msgs::Point::ConstPtr& data)
{
    pos_target_x = data->x;
    pos_target_y = data->y;
}

void callBackTargetYaw(const std_msgs::Float64::ConstPtr& data)
{
    yaw_target = data->data / 3.15f;
}

void callBackCurrentYaw(const std_msgs::Float64::ConstPtr& data)
{
    yaw_current = data->data / 3.15f;
    yaw_current_x = cos(data->data);
    yaw_current_y = sin(data->data);
}

void callBackDeltYaw(const std_msgs::Float64::ConstPtr& data)
{
    yaw_delt = data->data;
    if (-3.15f/4.f < yaw_delt && yaw_delt < 3.15f/4.f){
        yaw_forward = 1.0f;
        yaw_backward = 0.0f;
        yaw_leftward = 0.0f;
        yaw_rightward = 0.0f;
    }
    else if ( 3.15f/4.f*3.f < yaw_delt || yaw_delt < -3.15f/4.f*3.f )
    {
        yaw_forward = 0.0f;
        yaw_backward = 1.0f;
        yaw_leftward = 0.0f;
        yaw_rightward = 0.0f;
    }
    else if ( 3.15f/4.f < yaw_delt && yaw_delt < 3.15f/4.f*3.f )
    {
        yaw_forward = 0.0f;
        yaw_backward = 0.0f;
        yaw_leftward = 1.0f;
        yaw_rightward = 0.0f;
    }
    else
    {
        yaw_forward = 0.0f;
        yaw_backward = 0.0f;
        yaw_leftward = 0.0f;
        yaw_rightward = 1.0f;
    }
    yaw_delt = data->data / 3.15f;

}


int main(int argc, char** argv)
{
    time_t t = time(0);
    char tmp[64];
    strftime( tmp, sizeof(tmp), "%Y_%m_%d_%X",localtime(&t) );
    
    char c_rgb_data[100];
    sprintf(c_rgb_data,"rgb_data_%s.csv",tmp);
    cout<<"----- file rgb data: "<< c_rgb_data<<endl;
    char c_rgb_data_noi[100];
    sprintf(c_rgb_data_noi,"rgb_noi_data_%s.csv",tmp);
    cout<<"----- file rgb noi data: "<< c_rgb_data_noi<<endl;
    char c_depth_data[100];
    sprintf(c_depth_data,"depth_data_%s.csv",tmp);
    cout<<"----- file depth data: "<< c_depth_data<<endl;
    char c_uav_data[100];
    sprintf(c_uav_data,"uav_data_%s.csv",tmp);
    cout<<"----- file uav data: "<< c_uav_data<<endl;
    char c_label_data[100];
    sprintf(c_label_data,"label_data_%s.csv",tmp);
    cout<<"----- file label data: "<< c_label_data<<endl;
    char c_semantics[100];
    sprintf(c_semantics, "semantics_%s.csv",tmp);
    cout<<"----- file semantics data: "<< c_semantics<<endl;
    char c_dep_noi[100];
    sprintf(c_dep_noi, "dep_noi_data_%s.csv",tmp);
    cout<<"----- file depth data with noise: "<< c_dep_noi<<endl;

    outFile_rgbdata.open(c_rgb_data, ios::out);
    outFile_rgbnoidata.open(c_rgb_data_noi, ios::out);
    outFIle_depthdata.open(c_depth_data, ios::out);
    outFile_labels.open(c_label_data, ios::out);
    outFile_semantics.open(c_semantics, ios::out);
    outFile_uavdata.open(c_uav_data, ios::out);
    outFile_depnoise.open(c_dep_noi, ios::out);

    ros::init(argc, argv, "uavdataprocessorigin");
    ros::NodeHandle nh;
    ros::Subscriber Odom_sub = nh.subscribe("/odom", 2, callBackOdom);
    ros::Subscriber Radar_sub = nh.subscribe("/radar/current_point", 2, callBackRadar);
    ros::Subscriber Cmd_sub = nh.subscribe("/mobile_base/commands/velocity", 2, callBackCmdMobileBase);
    ros::Subscriber CmdSmoother_sub = nh.subscribe("/teleop_velocity_smoother/raw_cmd_vel", 2, callBackTeleop);
    ros::Subscriber TargetPos_sub = nh.subscribe("/radar/target_point", 2, callBackTargetPos);
    ros::Subscriber TargetYaw_sub = nh.subscribe("/radar/target_yaw", 2, callBackTargetYaw);
    ros::Subscriber CurrentYaw_sub = nh.subscribe("/radar/current_yaw", 2, callBackCurrentYaw);
    ros::Subscriber DeltYaw_sub = nh.subscribe("/radar/delt_yaw", 2, callBackDeltYaw);
    ros::Subscriber rgbImage_sub = nh.subscribe("/camera/rgb/image_raw", 2, callbackRGB); //rgb images
    ros::Subscriber depthImage_sub = nh.subscribe("/camera/depth/image_raw", 1, imageCb_d); //depth images
    ros::Subscriber detection_sub = nh.subscribe("/darknet_ros/bounding_boxes", 2, objectsCallback); //semantics 
    ros::spin();
    outFile_labels.close();
    outFile_rgbdata.close();
    outFile_rgbnoidata.close();
    outFIle_depthdata.close();
    outFile_depnoise.close();
    return 0;
}