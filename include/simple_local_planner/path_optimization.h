#ifndef PATH_OPTIMIZATION_H
#define PATH_OPTIMIZATION_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>

namespace simple_local_planner
{

class path_optimization
{
public:
  path_optimization();

  ~path_optimization();

  void optimizationOrientation(std::vector<geometry_msgs::PoseStamped> &plan);

  //路径平滑
  void planPathSmooth(std::vector<geometry_msgs::PoseStamped>& out);

  //路径平滑3点描线
  void planPathSmooth3(std::vector<geometry_msgs::PoseStamped>& out);

  //路径平滑5点描线
  void planPathSmooth5(std::vector<geometry_msgs::PoseStamped>& out);

  //路径平滑7点描线
  void planPathSmooth7(std::vector<geometry_msgs::PoseStamped>& out);

  //远离障碍物 <待继续完善>
  void awayFromObstacles(costmap_2d::Costmap2D* costmap, std::vector<geometry_msgs::PoseStamped> &plan, double safe_distance = 0.1);

  //角度限制优化
  int angleLimit(std::vector<geometry_msgs::PoseStamped>& plan,double movement_angle_range = M_PI/4);

  //路径拉直
  void cutPathPoint(costmap_2d::Costmap2D* costmap, std::vector<geometry_msgs::PoseStamped> &plan);

  //路径插值
  void insertPointForPath(std::vector<geometry_msgs::PoseStamped> &plan, double distance = 0.05);


private:
  //边准化角度
  double inline normalizeAngle(double val,double min = -M_PI,double max = M_PI);

  //计算两点间的距离
  double inline distance(double x1,double y1,double x2,double y2);

  //检查点是否为障碍物
  bool collision(costmap_2d::Costmap2D* costmap, const double x, const double y);

  //检测直线上是否有障碍物
  bool isLineFree(costmap_2d::Costmap2D* costmap, const geometry_msgs::PoseStamped pose1, const geometry_msgs::PoseStamped pose2);

};

}


#endif // PATH_OPTIMIZATION_H
