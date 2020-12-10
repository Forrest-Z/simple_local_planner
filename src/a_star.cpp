#include "a_star.h"

namespace simple_local_planner
{

  A_star_planner::A_star_planner(costmap_2d::Costmap2DROS *cmap): path_opt_(nullptr)
  {
    this->cost_mapROS_ = cmap;
    this->cost_map2D_ = this->cost_mapROS_->getCostmap();
    this->mapToNodeMap(); //如果是静态地图只需要执行一次即可
    if(this->node_map_vector_.empty())
    {
      ROS_WARN("mapToNodeMap() ERROR!!");
      return;
    }

    this->map_size_x_ = cost_map2D_->getSizeInCellsX();
    this->map_size_y_ = cost_map2D_->getSizeInCellsY();

    path_opt_ = new path_optimization();

    setParam(1, 0.5);
  }

  A_star_planner::~A_star_planner()
  {
    if(path_opt_ != nullptr)
    {
      delete path_opt_;
      path_opt_ = nullptr;
    }
  }

  void A_star_planner::printNodeMap()
  {
    if(node_map_vector_.size() == 0)
    {
      ROS_ERROR("node map ERROR");
      return;
    }

    int node_map_rows = this->node_map_vector_.size();
    int node_map_cols = this->node_map_vector_.at(0).size();

    std::cout << "node_map = " << std::endl;
    std::cout << "node_map.rows = " << node_map_rows << std::endl;
    std::cout << "node_map.cols = " << node_map_cols << std::endl;

    for (int i = node_map_rows-1; i >= 0; i--)
    {
      for(int j = node_map_cols-1; j >= 0; j--)
      {
        if(node_map_vector_.at(i).at(j).cost == costmap_2d::FREE_SPACE)
          std::cout << 0;
        else if(node_map_vector_.at(i).at(j).cost == 2)
          std::cout << 2;
        else
          std::cout << 1;
      }
      std::cout << std::endl;
    }
    std::cout << "print NodeMap done" << std::endl;
  }

  void A_star_planner::setParam(int smooth_param, double wp_tolerance)
  {
    boost::mutex::scoped_lock l(configuration_mutex_);

    if(smooth_param >= 0 && smooth_param <= 3)
      this->a_star_smooth_param_ = smooth_param;
    else
      ROS_ERROR("set local A star planner param ERROR!");

     this->a_star_wp_tolerance_ = wp_tolerance;
  }

  bool A_star_planner::getPath(geometry_msgs::PoseStamped rec_goal, std::vector<geometry_msgs::PoseStamped>& path)
  {
    path.clear();

    this->path_.clear();

    if(rec_goal.header.frame_id != this->cost_mapROS_->getGlobalFrameID())
    {
      using namespace std;
      ROS_WARN("A star received goal FrameID != GlobalFrameID. planner ERROR!!");
      cout << "goal FrameID  = " << rec_goal.header.frame_id << endl;
      cout << "GlobalFrameID = " << this->cost_mapROS_->getBaseFrameID() << endl;
      return false;
    }

    getAroundFreePoint(rec_goal,this->goal_poseStamped_);

    this->generatePath();

    if(this->path_.empty())
      return false;
    else
    {
      /********************
       * 路径优化待完善,添加Eband路径优化
       *******************/
      this->path_opt_->cutPathPoint(this->cost_map2D_, this->path_);
      this->path_opt_->insertPointForPath(this->path_);
      this->path_opt_->optimizationOrientation(this->path_);
      path = this->path_;
    }
    return true;
  }

  void A_star_planner::generatePath()
  {
    this->cost_mapROS_->updateMap();
    this->cost_map2D_ = this->cost_mapROS_->getCostmap();
    this->cost_map2D_->getMutex()->lock();
    this->mapToNodeMap(); //如果是静态地图只需要执行一次即可
    this->cost_map2D_->getMutex()->unlock();
    if(this->node_map_vector_.empty())
    {
      ROS_WARN("mapToNodeMap() ERROR!!");
      return;
    }

    geometry_msgs::PoseStamped start;
    this->cost_mapROS_->getRobotPose(start);

    geometry_msgs::PoseStamped start_around_free;
    this->getAroundFreePoint(start, start_around_free);

    unsigned int mx = 0,my = 0;
    this->cost_map2D_->worldToMap(start_around_free.pose.position.x,start_around_free.pose.position.y,mx,my);
    mx = mx >= this->map_size_x_?this->map_size_x_-1:mx;
    my = my >= this->map_size_y_?this->map_size_y_-1:my;
    this->start_node_.coordinate.first = mx;
    this->start_node_.coordinate.second = my;
    this->start_node_.cost = this->node_map_vector_.at(mx).at(my).cost;
    this->start_node_.in_open_vector = false;
    this->start_node_.in_close_vector = false;
    this->start_node_.father_node = nullptr;
    this->countNodeG(this->start_node_);
    this->countNodeH(this->start_node_);
    this->countNodeF(this->start_node_);

    this->cost_map2D_->worldToMap(this->goal_poseStamped_.pose.position.x,this->goal_poseStamped_.pose.position.y,mx,my);
    mx = mx >= this->map_size_x_?this->map_size_x_-1:mx;
    my = my >= this->map_size_y_?this->map_size_y_-1:my;
    this->goal_node_.coordinate.first = mx;
    this->goal_node_.coordinate.second = my;
    this->goal_node_.cost = this->node_map_vector_.at(mx).at(my).cost;
    this->goal_node_.in_open_vector = false;
    this->goal_node_.in_close_vector = false;
    this->goal_node_.father_node = nullptr;
    this->countNodeG(this->goal_node_);
    this->countNodeH(this->goal_node_);
    this->countNodeF(this->goal_node_);

    this->start_poseStamped_ = start_around_free;

    this->makePlan();
  }

  void A_star_planner::getAroundFreePoint(geometry_msgs::PoseStamped in_p,geometry_msgs::PoseStamped& out_p)
  {
    out_p.header = in_p.header;
    out_p.pose.orientation = in_p.pose.orientation;
    out_p.pose.position.z = 0.0;
    unsigned int mx = 0,my = 0;
    this->cost_map2D_->worldToMap(in_p.pose.position.x,in_p.pose.position.y,mx,my);
    if(this->cost_map2D_->getCost(mx,my) == costmap_2d::FREE_SPACE)
    {
      out_p.pose.position = in_p.pose.position;
      return;
    }

    unsigned int range = static_cast<unsigned int>(a_star_wp_tolerance_/cost_map2D_->getResolution());
    unsigned int size_in_cells_x = cost_map2D_->getSizeInCellsX();
    unsigned int size_in_cells_y = cost_map2D_->getSizeInCellsY();
    unsigned int index_x_begin  = 0,index_x_stop  = 0;
    unsigned int index_y_begin  = 0,index_y_stop  = 0;
    unsigned int err = 0;
    for(err = 1;err <= range;err++)
    {
      index_x_begin = mx>=err?mx-err:0;
      index_x_stop  = mx+err>=size_in_cells_x?size_in_cells_x:mx+err;
      index_y_begin = my>=err?my-err:0;
      index_y_stop  = my+err>=size_in_cells_y?size_in_cells_y:my+err;
      for(unsigned int i = index_x_begin;i <= index_x_stop;i++)
      {
        for(unsigned int j = index_y_begin;j <= index_y_stop;j++)
        {
          double wx = 0;
          double wy = 0;
          if(this->cost_map2D_->getCost(i,j) == costmap_2d::FREE_SPACE)
          {
            this->cost_map2D_->mapToWorld(i,j,wx,wy);
            out_p.pose.position.x = wx;
            out_p.pose.position.y = wy;
            return;
          }
        }
      }
    }
    out_p.pose.position = in_p.pose.position;
  }

  void A_star_planner::makePlan()
  {
    std::vector<node> node_plan;
    int index = 0;
    node current_node;
    this->insertOpenList(this->start_node_);
    while (current_node.coordinate != this->goal_node_.coordinate && open_vector_.size() > 0)
    {
      //this->getMinFNodeFromOpenList(index);
      current_node = this->open_vector_.at(index);
      this->open_vector_.erase(open_vector_.begin() + index);
      current_node.in_open_vector = false;
      current_node.in_close_vector = true;
      this->insertCloseList(current_node);
      this->checkAroubdNode(current_node);
    }
    if (this->open_vector_.empty())
    {
      std::cout << __FILE__ << " line: " << __LINE__ << std::endl;
      ROS_INFO("A star open vector is empty.");
      return;
    }
    else
    {
      while (current_node.coordinate != this->start_node_.coordinate)
      {
        node_plan.push_back(current_node);
        current_node = *(current_node.father_node);
      }
      node_plan.push_back(current_node); //顺序为从终点到起点

      this->transformPlan(node_plan);
    }
    this->clearCache();
  }

  void A_star_planner::transformPlan(std::vector<node>& node_plan)
  {
    std::vector<geometry_msgs::PoseStamped> PoseStamped_vector;
    size_t plan_size = node_plan.size();
    geometry_msgs::PoseStamped current_PoseStamped;
    for(int i = plan_size-1;i >= 0;i--)
    {
      if(i == plan_size-1 || i == 0)
      {
        if(i == plan_size-1)
        {
          current_PoseStamped = this->start_poseStamped_;
        }
        else
        {
          current_PoseStamped = this->goal_poseStamped_;
        }
        goal_poseStamped_.header.stamp = ros::Time::now();
        goal_poseStamped_.header.frame_id = this->cost_mapROS_->getGlobalFrameID();
      }
      else
      {
        double wx = 0,wy = 0;
        unsigned int mx = 0,my = 0;
        mx = node_plan.at(i).coordinate.first;
        my = node_plan.at(i).coordinate.second;
        this->cost_map2D_->mapToWorld(mx,my,wx,wy);
        current_PoseStamped.header.stamp = ros::Time::now();
        current_PoseStamped.header.frame_id = this->cost_mapROS_->getGlobalFrameID();
        current_PoseStamped.pose.position.x = wx;
        current_PoseStamped.pose.position.y = wy;
        current_PoseStamped.pose.position.z = 0;
      }
      PoseStamped_vector.push_back(current_PoseStamped);
    }
    if(plan_size == 1)
      PoseStamped_vector.push_back(this->goal_poseStamped_);

    this->path_.clear();
    size_t point_size = PoseStamped_vector.size();
    this->path_.resize(point_size);
    for(size_t i=0;i<point_size;i++)
    {
      this->path_[i] = PoseStamped_vector[i];
    }
  }

  void A_star_planner::mapToNodeMap()
  {
    int x_SizeInCells = this->cost_map2D_->getSizeInCellsX();
    int y_SizeInCells = this->cost_map2D_->getSizeInCellsY();
    this->node_map_vector_.clear();
    for(int i_x = 0;i_x < x_SizeInCells;i_x++)
    {
      std::vector<node> temp_node_vec;
      for(int i_y = 0;i_y < y_SizeInCells;i_y++)
      {
        node temp_node;
        temp_node.coordinate.first = i_x;
        temp_node.coordinate.second = i_y;
        temp_node.cost = this->cost_map2D_->getCost(i_x,i_y);
        temp_node.father_node = nullptr;
        temp_node.F = 0;
        temp_node.G = 0;
        temp_node.H = 0;
        temp_node.in_close_vector = false;
        temp_node.in_open_vector = false;
        temp_node_vec.push_back(temp_node);
      }
      this->node_map_vector_.push_back(temp_node_vec);
    }
  }

  void A_star_planner::updateNodeMapVector(node& tem_node)
  {
    this->node_map_vector_.at(tem_node.coordinate.first).at(tem_node.coordinate.second) = tem_node;
  }

  void A_star_planner::countNodeG(node& tem_node)
  {
    if (tem_node.father_node == nullptr)
      tem_node.G = 0;
    else
    {
      if (tem_node.coordinate.first - tem_node.father_node->coordinate.first == 0
          || tem_node.coordinate.second - tem_node.father_node->coordinate.second == 0)
      {
        tem_node.G = tem_node.father_node->G + 10;
      }
      else
      {
        tem_node.G = tem_node.father_node->G + 14;
      }
    }
  }

  void A_star_planner::countNodeH(node& tem_node)
  {
    tem_node.H = fabs(tem_node.coordinate.first - this->goal_node_.coordinate.first) * 10 +
      fabs(tem_node.coordinate.second - this->goal_node_.coordinate.second) * 10;
  }

  void A_star_planner::countNodeF(node& tem_node)
  {
    tem_node.F = tem_node.G + tem_node.H;
  }

  void A_star_planner::insertCloseList(node& tem_node)
  {
    if (tem_node.in_close_vector == false)
    {
      tem_node.in_close_vector = true;
      this->close_vector_.push_back(tem_node);
    }
    this->updateNodeMapVector(tem_node);
  }

  void A_star_planner::insertOpenList(node& tem_node)
  {
    if (tem_node.cost == costmap_2d::FREE_SPACE && tem_node.in_open_vector == false)
    {
      tem_node.in_open_vector = true;
      this->open_vector_.push_back(tem_node);
    }
    this->updateNodeMapVector(tem_node);
  }

  bool A_star_planner::inVector(std::vector<node> tem_vector,node tem_node,int& index)
  {
    for (int i = 0; i < tem_vector.size(); i++)
    {
      if (tem_vector.at(i).coordinate == tem_node.coordinate)
      {
        index = i;
        return true;
      }
    }
    return false;
  }

  void A_star_planner::getMinFNodeFromOpenList(int& index)
  {
    double min_F = 99999999999999.0;
    for (int i = 0; i < this->open_vector_.size(); i++)
    {
      if (min_F > this->open_vector_.at(i).F)
      {
        min_F = this->open_vector_.at(i).F;
        index = i;
      }
    }
  }

  void A_star_planner::clearCache()
  {
    this->open_vector_.clear();
    this->close_vector_.clear();
  }

  void A_star_planner::checkAroubdNode(node& tem_node)
  {
    node around_node;
    for (int i = -1; i <= 1; i++)
    {
      for (int j = -1; j <= 1; j++)
      {
        if(tem_node.coordinate.first + i > this->map_size_x_ - 1 || tem_node.coordinate.second + j > this->map_size_y_ - 1)
          continue;
        if(tem_node.coordinate.first - i > this->map_size_x_ - 1 || tem_node.coordinate.second - j > this->map_size_y_ - 1)
          continue;
        if(tem_node.coordinate.first + i < 0  || tem_node.coordinate.second + j < 0)
          continue;
        around_node = this->node_map_vector_.at(tem_node.coordinate.first + i).at(tem_node.coordinate.second + j);
        if (i == 0 && j == 0)
          continue;
        else
        {
          if (around_node.cost != costmap_2d::FREE_SPACE || around_node.in_close_vector == true)
          {
            this->insertCloseList(tem_node);
          }
          else
          {
            if(abs(i) == 1 && abs(j) == 1)
            {
              if(around_node.coordinate.first - i < this->map_size_x_ - 1 )
              {
                if(this->node_map_vector_.at(around_node.coordinate.first - i).at(around_node.coordinate.second).cost != 0)
                  continue;
              }
              if(around_node.coordinate.second - j < this->map_size_y_ - 1)
              {
                 if(this->node_map_vector_.at(around_node.coordinate.first).at(around_node.coordinate.second - j).cost != 0)
                  continue;
              }
            }
            if (around_node.in_open_vector == false)
            {
              around_node.father_node = &(this->node_map_vector_.at(tem_node.coordinate.first).at(tem_node.coordinate.second));
              this->countNodeG(around_node);
              this->countNodeH(around_node);
              this->countNodeF(around_node);
              this->insertOpenList(around_node);
            }
            else
            {
              double tem_G = 0;
              if (i == 0 || j == 0)
              {
                tem_G = tem_node.G + 10;
              }
              else
              {
                tem_G = tem_node.G + 14;
              }
              if (around_node.G > tem_G)
              {
                int index = 0;
                this->inVector(this->open_vector_,around_node,index);
                this->open_vector_.erase(this->open_vector_.begin() + index);
                around_node.father_node = &(this->node_map_vector_.at(tem_node.coordinate.first).at(tem_node.coordinate.second));
                this->countNodeG(around_node);
                this->countNodeH(around_node);
                this->countNodeF(around_node);
                this->insertOpenList(around_node);
              }
            }
          }
        }
      }
    }
    sort(this->open_vector_.begin(), this->open_vector_.end(), this->compF);
  }
}
