//
// Created by lucasyu on 18-12-13.
//
#include <map>
#include<stdio.h>
#include<stdlib.h>
#include <vector>
#include <iostream>
#define random(x) (rand()%x)

using namespace std;


const double points[17][2] = {{-19, -10.25}, {-1.5, -10.25}, {7.5, -10.25}, {18.5, -10.25}, {-19, 0.0}, {-1.5, 0.0}, 
                        {7.5, 0.0}, {18.5, 0.0}, {-19, 6.25}, {-8.75, 6.25}, {-1.5, 6.25}, {7.5, 6.25}, {18.5, 6.25},\
                        {-19, 11},{-8.75, 11},{7.5, 11},{18.5, 11}};

std::vector<std::vector<float> > all_points;

int cur_position_index = 4;
int last_position_index = cur_position_index;


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


int main()
{
    for (auto& pt : points)
    {
        std::vector<float> v;
        v.push_back(pt[0]);
        v.push_back(pt[1]);
        all_points.push_back(v);
    }

    map<int, std::vector<int> > adj_map;

    std::vector<std::vector<int> > adj_vecs;
    map< int, std::vector<int> > adj_map_times_of_visited;

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

    init_adj_map(adj_vecs, adj_map, adj_map_times_of_visited);

    
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


    for (int i=0;i<500;++i)
    {
        next_target_pt = generate_global_target(cur_position_index, last_position_index, 
                                                adj_map,all_points,adj_map_times_of_visited);

        cout<<"iteration: "<<i<<"("<<next_target_pt[0]<<", "<<next_target_pt[1]<<")"<<endl;
        
    }
    for(int i_pt=0;i_pt<all_points.size();i_pt++)
        {
            cout << "point number: " << i_pt << endl;
            for (int jj=0;jj<adj_map_times_of_visited[i_pt].size();jj++)
            {
                int tmp_cout = adj_map_times_of_visited[i_pt][jj];
                cout << "adjacent index: " << adj_map[i_pt][jj] 
                     <<", number of visit: " << tmp_cout <<endl;
            }
            
        }

    return 0;
}