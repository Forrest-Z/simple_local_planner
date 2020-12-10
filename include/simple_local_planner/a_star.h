#ifndef A_STAR_H
#define A_STAR_H
 
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <nav_msgs/GetPlan.h>
#include <iostream>
#include <cmath>
#include <algorithm>

#include <path_optimization.h>

namespace simple_local_planner
{
  struct node
  {
    std::pair<int,int> coordinate; //对应map的坐标，不是world下的坐标 (x,y)，对应node_map_vector_中的(行，列)
    uint8_t cost; //对应代价值
    double F;    //F = G + H;
    double G;    //G：从起点 A 移动到指定方格的移动代价，沿着到达该方格而生成的路径
    double H;    //H：从指定的方格移动到终点的估算成本
    bool in_open_vector;  //在开放列表中为1，不在为0
    bool in_close_vector; //在关闭列表中为1，不在为0
    node* father_node; //父节点
  };


  class A_star_planner
  {
  public:
    A_star_planner(costmap_2d::Costmap2DROS* cmap);
    ~A_star_planner();

    void printNodeMap();
    bool getPath(geometry_msgs::PoseStamped rec_goal, std::vector<geometry_msgs::PoseStamped>& path);
    void setParam(int smooth_param, double wp_tolerance);

  private:
    void getAroundFreePoint(geometry_msgs::PoseStamped in_p,geometry_msgs::PoseStamped& out_p);
    void generatePath();
    void transformPlan(std::vector<node>& node_plan);
    void mapToNodeMap();
    void updateNodeMapVector(node& tem_node);
    void checkAroubdNode(node& tep_node);
    void makePlan();
    bool inVector(std::vector<node> tem_vector, node tem_node, int& index);
    void countNodeG(node& tem_node);
    void countNodeH(node& tem_node);
    void countNodeF(node& tem_node);
    void insertOpenList(node& tem_node);
    void insertCloseList(node& tem_node);
    void getMinFNodeFromOpenList(int& index);
    void clearCache();

  private:
    double a_star_wp_tolerance_;
    int a_star_smooth_param_;
    uint32_t plan_seq_;
    unsigned int map_size_x_;
    unsigned int map_size_y_;
    std::vector<geometry_msgs::PoseStamped> path_;
    path_optimization* path_opt_;
    geometry_msgs::PoseStamped goal_poseStamped_;
    geometry_msgs::PoseStamped start_poseStamped_;
    costmap_2d::Costmap2DROS* cost_mapROS_;
    costmap_2d::Costmap2D* cost_map2D_;
    std::vector<node> open_vector_;
    std::vector<node> close_vector_;
    std::vector< std::vector<node> > node_map_vector_;
    node start_node_;
    node goal_node_;

    boost::mutex configuration_mutex_;

  protected:
    static 	bool compF(const node &a, const node &b)
    {
      return a.F < b.F;
    }

    static 	bool compG(const node &a, const node &b)
    {
      return a.G < b.G;
    }

  };

}

#endif
