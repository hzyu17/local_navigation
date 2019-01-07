#include <ros/ros.h> 
#include <std_msgs/Float64MultiArray.h> 
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Float64.h> 
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#define random(x) (rand()%x)

template<class T>
int length(T& arr)
{
    return sizeof(arr) / sizeof(arr[0]);
}


using namespace cv;
using namespace std;
/* Values about map: crossing points in the map */
const double points[17][2] = {{-19, -10.25}, {-1.5, -10.25}, {7.5, -10.25}, {18.5, -10.25}, {-19, 0.0}, {-1.5, 0.0}, 
						{7.5, 0.0}, {18.5, 0.0}, {-19, 6.25}, {-8.75, 6.25}, {-1.5, 6.25}, {7.5, 6.25}, {18.5, 6.25},\
						{-19, 11},{-8.75, 11},{7.5, 11},{18.5, 11}};

std::vector<std::vector<float> > all_points;

int cur_position_index = 4;
int last_position_index = cur_position_index;


const double close_dist[17] = {1.75, 1.75, 1.75, 1.75, 
								1.5, 1.6, 1.6, 1.6, 
								1.5, 1.5, 1.6, 1.6, 1.6, 
								1.2, 1.5, 1.6, 1.6};


/* Variables */
double position[3]={0.0, 0.0, 0.0};
double angle[3] = {0.0, 0.0, 0.0};


void drawArrow(cv::Mat& img, cv::Point pStart, cv::Point pEnd, int len, int alpha, cv::Scalar color, int thickness, int lineType)
{
    const double PI = 3.1415926;
    Point arrow;
    double angle = atan2((double)(pStart.y - pEnd.y), (double)(pStart.x - pEnd.x));
    line(img, pStart, pEnd, color, thickness, lineType);
    arrow.x = pEnd.x + len * cos(angle + PI * alpha / 180);

    arrow.y = pEnd.y + len * sin(angle + PI * alpha / 180);

    line(img, pEnd, arrow, color, thickness, lineType);

    arrow.x = pEnd.x + len * cos(angle - PI * alpha / 180);

    arrow.y = pEnd.y + len * sin(angle - PI * alpha / 180);

    line(img, pEnd, arrow, color, thickness, lineType);
}

void groundtruthCallback(const gazebo_msgs::ModelStates& msg)
{
	int num_model = msg.name.size();
	if (std::strcmp(msg.name[num_model-1].c_str(),"mobile_base")){
		cout << "Attention! Got the wrong object from model_states!! not the mobile base!!"<<endl 
			 << "Object got from model_states: " << msg.name[num_model-1].c_str() << endl;
	}

	position[0] = msg.pose[num_model-1].position.x;
	position[1] = msg.pose[num_model-1].position.y;
	position[2] = msg.pose[num_model-1].position.z;

	float q0 = msg.pose[num_model-1].orientation.x;
	float q1 = msg.pose[num_model-1].orientation.y;
	float q2 = msg.pose[num_model-1].orientation.z;
	float q3 = msg.pose[num_model-1].orientation.w;

	/* Pitch roll may be needed for MAVs */
	angle[2] = atan2(2*q3*q2 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1);  // Yaw

}

void init_adj_map(const std::vector<std::vector<int> >& arrays, map<int, vector<int> >& adj_map,
                  map< int, std::vector<int> >& adj_map_times_of_visited)
{
    cout<<"begin map initialization.."<<endl;
    int i = 0;
    for (auto & item_array : arrays){
        std::vector<int> tmp_record;
        adj_map[i] = item_array;
        for (int i=0; i<item_array.size();i++){
            tmp_record.push_back(0);
        }   
        adj_map_times_of_visited[i] = tmp_record;
        i++;
    }
}

std::vector<float> generate_global_target(int& cur_position_index, int& last_position_index,
                            map< int, std::vector<int> >& adj_map, vector< vector<float> >& all_points,
                            map< int, std::vector<int> >& adj_map_times_of_visited )
{
    int num_adj = adj_map[cur_position_index].size();
    int min_visited_index = 0;
    int min_visit_time = 9999;
    for (int i=0;i<adj_map[cur_position_index].size();i++){
        if(adj_map[cur_position_index][i] != last_position_index && 
            adj_map_times_of_visited[cur_position_index][i]<min_visit_time){
            min_visit_time = adj_map_times_of_visited[cur_position_index][i];
            min_visited_index = i;
        }
    }
    
    
    int next_position_index = adj_map[cur_position_index][min_visited_index];
    adj_map_times_of_visited[cur_position_index][min_visited_index] += 1;
    
    last_position_index = cur_position_index;
    vector<float> next_target = all_points[next_position_index];
    cur_position_index = next_position_index;
    
    return next_target;
}

int main(int argc, char **argv)
{ 
	for (auto& pt : points)
	{
		std::vector<float> v;
		v.push_back(pt[0]);
		v.push_back(pt[1]);
    	all_points.push_back(v);
	}

    map<int, std::vector<int> > adj_map;
    map< int, std::vector<int> > adj_map_times_of_visited;

    std::vector<std::vector<int> > adj_vecs;

    //Set adjacent points for each point in the point set:
    adj_vecs.push_back({1, 4});
    adj_vecs.push_back({0, 2, 5});
    adj_vecs.push_back({1, 3, 6});
    adj_vecs.push_back({2, 7});
    adj_vecs.push_back({0, 5, 8});
    adj_vecs.push_back({1, 4, 6, 10});
    adj_vecs.push_back({2, 5, 7, 11});
    adj_vecs.push_back({3, 6, 12});
    adj_vecs.push_back({4, 9, 13});
    adj_vecs.push_back({8, 10, 14});
    adj_vecs.push_back({5, 9, 11});
    adj_vecs.push_back({6, 10, 12, 15});
    adj_vecs.push_back({7, 11, 16});
    adj_vecs.push_back({8, 14});
    adj_vecs.push_back({9, 13, 15});
    adj_vecs.push_back({11, 14, 16});
    adj_vecs.push_back({12, 15});

    init_adj_map(adj_vecs, adj_map);
    
    for (int i=0; i<adj_map[cur_position_index].size(); i++)
    {
    	cout<<"check : adjacent indexes: "<< adj_map[cur_position_index][i]<<", ";
    }
    cout<<endl;
    for (int i=0; i<adj_map[cur_position_index].size(); i++)
    {
    	cout<<"check : adjacent pos: "<< all_points[adj_map[cur_position_index][i]][0]<<", " <<all_points[adj_map[cur_position_index][i]][1]<<", ";
    }
    cout<<endl;

    std::vector<float> next_target_pt;
    
	ros::init(argc,argv,"global_path"); 
	ros::NodeHandle n; 

	// ros::Subscriber yaw_sub= n.subscribe("/odom",1,odometryCallback);
	ros::Subscriber yaw_sub= n.subscribe("/gazebo/model_states",1,groundtruthCallback); 

	namedWindow( "Compass", CV_WINDOW_AUTOSIZE );
	namedWindow( "Command", CV_WINDOW_AUTOSIZE );

	ros::Publisher target_pos_pub = n.advertise<geometry_msgs::Point>("/radar/target_point", 1);  // Gloabl coordinate, not robot odom coord
	ros::Publisher current_pos_pub = n.advertise<geometry_msgs::Point>("/radar/current_point", 1); // Gloabl coordinate, not robot odom coord
	ros::Publisher target_yaw_pub = n.advertise<std_msgs::Float64>("/radar/target_yaw", 1); // Gloabl coordinate, same with robot odom coord
	ros::Publisher current_yaw_pub = n.advertise<std_msgs::Float64>("/radar/current_yaw", 1);
	ros::Publisher delt_yaw_pub = n.advertise<std_msgs::Float64>("/radar/delt_yaw", 1);
    ros::Publisher direction_pub = n.advertise<std_msgs::Float64MultiArray>("/radar/direction", 1);

	std_msgs::Float64 target_yaw;
	std_msgs::Float64 current_yaw;
	std_msgs::Float64 delt_yaw;
    std_msgs::Float64MultiArray direction;

	geometry_msgs::Point target_point;
	geometry_msgs::Point current_point;


	double target_x = all_points[cur_position_index][0];
	double target_y = all_points[cur_position_index][1];
	cout<<"initial target position: "<<"x: "<<target_x<<", y: "<<target_y<<endl;
    ros::Rate loop_rate(20);

    while(ros::ok())
    {
    	/* Close detection */
    	double dist_x = sqrt((target_x - position[0])*(target_x - position[0]) + (target_y - position[1])*(target_y - position[1]));
    	if(dist_x < close_dist[cur_position_index]) 
    	{
    		cout<< " current close distance: "<< close_dist[cur_position_index]<<endl;
    		cout << "You reached the target position!" << endl;

    		for (int i=0; i<adj_map[cur_position_index].size(); i++)
		    {
		    	cout<<"check : adjacent indexes: "<< adj_map[cur_position_index][i]<<endl
		    		<<"check : adjacent pos: "<< all_points[adj_map[cur_position_index][i]][0]<<", " 
		    		<<all_points[adj_map[cur_position_index][i]][1]<<endl;
		    }

		    cout << "current target position: " << cur_position_index << ", last target position: " << last_position_index << endl;
    		next_target_pt = generate_global_target(cur_position_index, last_position_index, 
                                                adj_map,all_points,adj_map_times_of_visited);
    		target_x = next_target_pt[0];
    		target_y = next_target_pt[1];

    		cout << "new target position: " << "x: " << target_x << ", y: " << target_y << endl;
    		
    	}

    	/* Calculate target yaw */
    	double delt_x = target_x - position[0];
    	double delt_y = target_y - position[1];

    	double yaw_t = atan2(delt_y, delt_x);

    	/* Calculate delt yaw */
    	double delt_yaw_value = 0.0;

        double delt_yaw_direct = yaw_t - angle[2];
        double delt_yaw_direct_abs = std::fabs(delt_yaw_direct);
        double sup_yaw_direct_abs = 2*M_PI - delt_yaw_direct_abs;

        if(delt_yaw_direct_abs < sup_yaw_direct_abs)
            delt_yaw_value = delt_yaw_direct;
        else
            delt_yaw_value = - sup_yaw_direct_abs * delt_yaw_direct / delt_yaw_direct_abs;

        /* Calculate diraction */
        direction.data.clear();
        int command_type = 0;
        if(delt_yaw_value > -M_PI/6.0 && delt_yaw_value < M_PI/6.0)  // Move forward
        {
            direction.data.push_back(1.0);
            direction.data.push_back(0.0);
            direction.data.push_back(0.0);
            direction.data.push_back(0.0);
            command_type = 0;
        }
        else if(delt_yaw_value >= -5*M_PI/6.0 && delt_yaw_value <= -M_PI/6.0)  // Turn right
        {
            direction.data.push_back(0.0);
            direction.data.push_back(0.0);
            direction.data.push_back(0.0);
            direction.data.push_back(1.0);
            command_type = 3;
        }
        else if(delt_yaw_value >= M_PI/6.0 && delt_yaw_value <= 5*M_PI/6.0) // Turn left
        {
            direction.data.push_back(0.0);
            direction.data.push_back(0.0);
            direction.data.push_back(1.0);
            direction.data.push_back(0.0);
            command_type = 2;
        }
        else  // Move backward
        {
            direction.data.push_back(0.0);
            direction.data.push_back(1.0);
            direction.data.push_back(0.0);
            direction.data.push_back(0.0);
            command_type = 1;
        }
        direction_pub.publish(direction);

    	/* Update and publish*/
    	target_point.x = target_x;
    	target_point.y = target_y;
    	target_point.z = 0.0;
    	target_pos_pub.publish(target_point);

    	current_point.x = position[0];
    	current_point.y = position[1];
    	current_point.z = 0.0;
    	current_pos_pub.publish(current_point);

    	target_yaw.data = yaw_t;
    	target_yaw_pub.publish(target_yaw);

    	current_yaw.data = angle[2];
    	current_yaw_pub.publish(current_yaw);

    	delt_yaw.data = delt_yaw_value;
    	delt_yaw_pub.publish(delt_yaw);


    	/* Convert to body coordinate */
    	double suggested_body_x = delt_x * cos(angle[2]) + delt_y * sin(angle[2]); 
    	double suggested_body_y = -delt_x * sin(angle[2]) + delt_y * cos(angle[2]);

    	/* Draw radar */
    	Mat img(300, 300, CV_8UC3, Scalar(0,0,0));
    	Point p(150, 150);
    	circle(img, p, 60, Scalar(0, 255, 0), 10);
    	circle(img, p, 5, Scalar(0, 0, 255), 3);
    	line(img, Point(150, 270), Point(150, 30), Scalar(255, 20, 0), 3);
    	line(img, Point(140, 40), Point(150, 30), Scalar(255, 20, 0), 3);
    	line(img, Point(160, 40), Point(150, 30), Scalar(255, 20, 0), 3);

    	Point p_dsr( -suggested_body_y* 140 + 150 , -suggested_body_x * 140 + 150);
    	line(img, p, p_dsr, Scalar(0, 0, 255), 4);

    	imshow("Compass", img);

    	/* Draw command */
    	Mat img2(300, 300, CV_8UC3, Scalar(0,0,0));
    	Point pStart(150, 150);
    	Point pEnd;
    	if(command_type == 0){
    		pEnd.x = 150;
    		pEnd.y = 50;
    	}
    	else if(command_type == 1){
    		pEnd.x = 150;
    		pEnd.y = 250;
    	}
    	else if(command_type == 2){
    		pEnd.x = 50;
    		pEnd.y = 150;
    	}
    	else{
    		pEnd.x = 250;
    		pEnd.y = 150;
    	}
		drawArrow(img2, pStart, pEnd, 25, 30, Scalar(0, 0, 255), 3, 4); 	

    	imshow("Command", img2);
    	waitKey(5);


    	ros::spinOnce(); 
    	loop_rate.sleep();
    }

	return 0;
} 

