#include <simple_local_planner/pure_planner_ros.h>
#include <sys/time.h>
#include <boost/tokenizer.hpp>
#include <Eigen/Core>
#include <cmath>
#include <ros/console.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(simple_local_planner::PurePlannerROS, nav_core::BaseLocalPlanner)

namespace simple_local_planner
{

  void PurePlannerROS::reconfigureCB(SimpleLocalPlannerConfig &config, uint32_t level)
  {
      tc_->reconfigure(config);
  }

  PurePlannerROS::PurePlannerROS() :
    world_model_(nullptr), tc_(nullptr), dp_(nullptr),costmap_ros_(nullptr), tf_(nullptr), initialized_(false), odom_helper_("odom") {}

  PurePlannerROS::PurePlannerROS(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros):
    world_model_(nullptr), tc_(nullptr), dp_(nullptr),costmap_ros_(nullptr), tf_(nullptr), initialized_(false), odom_helper_("odom")
  {
    initialize(name, tf, costmap_ros);
  }

  PurePlannerROS::~PurePlannerROS()
  {
    if(dsrv_ != nullptr)
    {
      delete dsrv_;
      dsrv_ = nullptr;
    }

    if(tc_ != nullptr)
    {
      delete tc_;
      tc_ = nullptr;
    }

    if(world_model_ != nullptr)
    {
      delete world_model_;
      world_model_ = nullptr;
    }

    if(dp_ != nullptr)
    {
      delete dp_;
      dp_= nullptr;
    }

    if(!log_)
      log_.close();
  }

  void PurePlannerROS::initialize(std::string name,tf2_ros::Buffer* tf,costmap_2d::Costmap2DROS* costmap_ros)
  {
    if (!isInitialized())
    {
        record_ = true;
        record_log_ = false;
        if(record_)
        {
          std::stringstream ss;
          std::time_t nowtime;
          nowtime = std::time(nullptr);

          ss << nowtime;
          std::string tim_str;
          ss >> tim_str;

          std::string log_name = "/home/firefly/nav_log/";
//          std::string log_name = "/home/lhh/nav_log/";
          log_name+=tim_str;
          log_name+="_local_planner.log";
          log_.open(log_name,std::ios::out); //|std::ios::app

          if(!log_)
          {
            ROS_ERROR("open local_planner.log fail.");
          }
          else
          {
            record_log_ = true;
            RECORD_LOG("Welcome to use the smartest robot in the world.");
            std::time_t nowtime;
            struct tm* p;
            nowtime = std::time(nullptr);
            p = std::localtime(&nowtime);
            log_ << p->tm_year+1900 << "-" << p->tm_mon+1 << "-" << p->tm_mday << "," <<
            p->tm_hour << ":" << p->tm_min << ":" << p->tm_sec << std::endl;
          }
        }

        ros::NodeHandle private_nh("~/" + name);
        l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
        g_replan_pub_ = private_nh.advertise<std_msgs::Bool>("/move_base/replan", 1);

        tf_ = tf;
        costmap_ros_ = costmap_ros;

        //initialize the copy of the costmap the controller will use
        costmap_ = costmap_ros_->getCostmap();

        global_frame_ = costmap_ros_->getGlobalFrameID();
        robot_base_frame_ = costmap_ros_->getBaseFrameID();

        // Robot Configuration Parameters
        private_nh.param("max_trans_vel", max_vel_x_, 0.6);
        private_nh.param("min_trans_vel", min_vel_x_, 0.1);
        private_nh.param("max_rot_vel", max_vel_th_, 0.5);
        private_nh.param("min_rot_vel", min_vel_th_, 0.1);
        private_nh.param("max_trans_acc", max_trans_acc_, 1.0);
        private_nh.param("max_rot_acc", max_rot_acc_, 1.0);
        private_nh.param("min_in_place_rot_vel", min_in_place_vel_th_, 0.3);

        //Goal tolerance parameters
        private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
        private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.10);
        private_nh.param("wp_tolerance", wp_tolerance_, 0.5);

        private_nh.param("sim_time", sim_time_, 1.0);
        private_nh.param("sim_granularity", sim_granularity_, 0.025);
        private_nh.param("angular_sim_granularity", angular_sim_granularity_, sim_granularity_);
        private_nh.param("controller_freq", controller_freq_, 15.0);

        bool is_use_dwa;
        private_nh.param("use_dwa", is_use_dwa, false);
        world_model_ = new CostmapModel(*costmap_);
        footprint_spec_ = costmap_ros_->getRobotFootprint();
        tc_ = new PurePlanner(*world_model_, *costmap_, footprint_spec_, controller_freq_,
                              max_vel_x_, min_vel_x_, max_vel_th_, min_vel_th_,
                              min_in_place_vel_th_, max_trans_acc_, max_rot_acc_,
                              yaw_goal_tolerance_, xy_goal_tolerance_, wp_tolerance_,
                              sim_time_, sim_granularity_, angular_sim_granularity_, is_use_dwa);

        // dijkstra planner
        dp_ = new DijkstraPlanner(name,costmap_,global_frame_);

        initialized_ = true;
        //this will load the values of cfg params overwritting the read ones from the yaml file.
        dsrv_ = new dynamic_reconfigure::Server<SimpleLocalPlannerConfig>(private_nh);
        dynamic_reconfigure::Server<SimpleLocalPlannerConfig>::CallbackType cb = boost::bind(&PurePlannerROS::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
    }
    else
    {
        ROS_WARN("This planner has already been initialized, doing nothing");
    }
  }

  void PurePlannerROS::replanGlobalpath()
  {
    std_msgs::Bool data;
    data.data = true;
    this->g_replan_pub_.publish(data);
  }

  bool PurePlannerROS::checkLocalPath(const std::vector<geometry_msgs::PoseStamped> &path)
  {
    unsigned int mx = 0,my = 0;
    size_t path_size = path.size();
    for (size_t i=0;i<path_size;i++)
    {
      this->costmap_->worldToMap(path[i].pose.position.x,path[i].pose.position.y,mx,my);
      if(this->costmap_->getCost(mx,my) != costmap_2d::FREE_SPACE)
      {
        return false;
      }
    }
    return true;
  }

  bool PurePlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
  {
    if (! isInitialized())
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    if(orig_global_plan.empty())
    {
      ROS_ERROR("local planner set plan zero length");
      return false;
    }

//    if(!global_plan_.empty())
//    {
//      double goal_goal_dist =  this->computeDistance(global_plan_.back().pose.position.x,
//                                                     global_plan_.back().pose.position.y,
//                                                     orig_global_plan.back().pose.position.x,
//                                                     orig_global_plan.back().pose.position.y); //新旧目标点的距离
//      double goal_goal_aw_diff = this->tc_->normalizeAngle( (tf::getYaw(global_plan_.back().pose.orientation) -
//                                                             tf::getYaw(orig_global_plan.back().pose.orientation)),
//                                                            -M_PI,
//                                                            M_PI); //新旧目标点角度差

//      if(goal_goal_dist > 0.1 || goal_goal_aw_diff > 0.1)
//      {
        local_plan_.clear();
//      }
//    }

    //reset the global plan
    global_plan_.clear();
    global_plan_ = orig_global_plan;

    //reset the goal flag
    reached_goal_ = false;
    return true;
  }

  bool PurePlannerROS::getLocalGoal(const tf2_ros::Buffer& tf,
                                    const std::vector<geometry_msgs::PoseStamped>& global_plan,
                                    const geometry_msgs::PoseStamped& global_pose,
                                    const costmap_2d::Costmap2D& costmap,
                                    const std::string& global_frame,
                                    geometry_msgs::PoseStamped& local_goal)
  {
    if (global_plan.empty())
    {
      ROS_ERROR("Received plan with zero length");
      return false;
    }
    geometry_msgs::PoseStamped plan_pose = global_plan[0];
    plan_pose.header.stamp = ros::Time::now();
    //let's get the pose of the robot in the frame of the plan
    geometry_msgs::PoseStamped robot_pose;
    try
    {
      tf.transform(global_pose, robot_pose, plan_pose.header.frame_id, ros::Duration(1.0)); //odom->map
    }
    catch (tf2::ExtrapolationException ex)
    {
      ROS_ERROR("PurePlannerROS::getLocaGoal:%s", ex.what());
      return false;
    }
    catch (tf2::TransformException ex)
    {
      ROS_ERROR("PurePlannerROS::getLocaGoal:%s", ex.what());
      return false;
    }

    //we'll discard points on the plan that are outside the local costmap
    double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                     costmap.getSizeInCellsY() * costmap.getResolution() / 2.0) * 0.9;

    unsigned int i = 0;
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist = 0;

    //we need to loop to a point on the plan that is within a certain distance of the robot
    unsigned int plan_pose_size = static_cast<unsigned int>(global_plan.size());
    while(i < plan_pose_size)
    {
      double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;
      if (sq_dist <= sq_dist_threshold)
      {
        break;
      }
      ++i;
    }

    //now we'll transform until points are outside of our distance threshold
    while(i < plan_pose_size && sq_dist <= sq_dist_threshold)
    {
      double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;
      ++i;
    }

    if(i >= 1)
    {
      plan_pose = global_plan[i-1];
    }
    else
    {
      plan_pose = global_plan[i];
    }
    plan_pose.header.stamp = ros::Time::now();
    try
    {
      tf.transform(plan_pose, local_goal, global_frame,ros::Duration(1.0)); //map->odom
    }
    catch (tf2::ExtrapolationException ex)
    {
      ROS_ERROR("PurePlannerROS::getLocaGoal:%s", ex.what());
      return false;
    }
    catch (tf2::TransformException ex)
    {
      ROS_ERROR("PurePlannerROS::getLocaGoal:%s", ex.what());
      return false;
    }

    return true;
  }

  bool PurePlannerROS::getLocaPlan(const geometry_msgs::PoseStamped &global_pose, std::vector<geometry_msgs::PoseStamped> &local_plan)
  {
    RECORD_LOG("getLocaPlan s");
    uint32_t count = 0;
    geometry_msgs::PoseStamped local_goal;
    ros::Rate rade(10);
    while(ros::ok())
    {
      RECORD_LOG("getLocaGoal s");
      if(this->getLocaGoal(global_pose, local_goal, count))
      {
        RECORD_LOG("makePlan s");
        if(this->dp_->makePlan(global_pose, local_goal, local_plan))
        {
          RECORD_LOG("makePlan 1 d");
          return true;
        }
        else
        {
          count++;
        }
      }
      else
      {
        RECORD_LOG("getLocaGoal 0 d");
        return false;
      }
      rade.sleep();
    }
    return true;
  }

  bool PurePlannerROS::getLocaGoal(const geometry_msgs::PoseStamped &global_pose,
                                   geometry_msgs::PoseStamped &local_goal,
                                   uint32_t count)
  {
    if (this->global_plan_.empty())
    {
      ROS_ERROR("Received plan with zero length");
      return false;
    }
    geometry_msgs::PoseStamped plan_pose = global_plan_[0];
    plan_pose.header.stamp = ros::Time::now();
    //let's get the pose of the robot in the frame of the plan
    geometry_msgs::PoseStamped robot_pose;
    try
    {
      this->tf_->transform(global_pose, robot_pose, plan_pose.header.frame_id, ros::Duration(1.0)); //odom->map
    }
    catch (tf2::ExtrapolationException ex)
    {
      ROS_ERROR("PurePlannerROS::getLocaGoal:%s", ex.what());
      return false;
    }
    catch (tf2::TransformException ex)
    {
      ROS_ERROR("PurePlannerROS::getLocaGoal:%s", ex.what());
      return false;
    }

    //we'll discard points on the plan that are outside the local costmap
    double dist_threshold = std::max(costmap_->getSizeInCellsX() * costmap_->getResolution() / 2.0,
                                     costmap_->getSizeInCellsY() * costmap_->getResolution() / 2.0) * 0.9;

    dist_threshold -= (count * 0.5);

    if(dist_threshold < 1.0)
      return false;

    size_t i = 0;
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist = 0;

    //we need to loop to a point on the plan that is within a certain distance of the robot
    size_t plan_pose_size = global_plan_.size();
    while(i < plan_pose_size)
    {
      double x_diff = robot_pose.pose.position.x - global_plan_[i].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan_[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;
      if (sq_dist <= sq_dist_threshold)
      {
        break;
      }
      ++i;
    }

    //now we'll transform until points are outside of our distance threshold
    while(i < plan_pose_size && sq_dist <= sq_dist_threshold)
    {
      double x_diff = robot_pose.pose.position.x - global_plan_[i].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan_[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;
      ++i;
    }

    if(i >= 1)
    {
      plan_pose = global_plan_[i-1];
    }
    else
    {
      plan_pose = global_plan_[i];
    }
    plan_pose.header.stamp = ros::Time::now();

    try
    {
      tf_->transform(plan_pose, local_goal, global_frame_, ros::Duration(1.0)); //map->odom
    }
    catch (tf2::ExtrapolationException ex)
    {
      ROS_ERROR("PurePlannerROS::getLocaGoal:%s", ex.what());
      return false;
    }
    catch (tf2::TransformException ex)
    {
      ROS_ERROR("PurePlannerROS::getLocaGoal:%s", ex.what());
      return false;
    }

    return true;
  }

  bool PurePlannerROS::transformGlobalPlan(const tf2_ros::Buffer& tf,
                                           const std::vector<geometry_msgs::PoseStamped>& global_plan,
                                           const geometry_msgs::PoseStamped& global_pose,
                                           const costmap_2d::Costmap2D& costmap,
                                           const std::string& global_frame,
                                           std::vector<geometry_msgs::PoseStamped>& transformed_plan)
  {
    transformed_plan.clear();
    if (global_plan.empty())
    {
      ROS_ERROR("Received plan with zero length");
      return false;
    }
    const geometry_msgs::PoseStamped& plan_pose = global_plan[0];
    try
    {
      // get plan_to_global_transform from plan frame to global_frame
      geometry_msgs::TransformStamped plan_to_global_transform = tf.lookupTransform(global_frame,
                                                                                    ros::Time(),
                                                                                    plan_pose.header.frame_id,
                                                                                    plan_pose.header.stamp,
                                                                                    plan_pose.header.frame_id,
                                                                                    ros::Duration(0.5));

      //let's get the pose of the robot in the frame of the plan
      geometry_msgs::PoseStamped robot_pose;
      tf.transform(global_pose, robot_pose, plan_pose.header.frame_id);

      //we'll discard points on the plan that are outside the local costmap
      double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                       costmap.getSizeInCellsY() * costmap.getResolution() / 2.0) * 0.9;

      unsigned int i = 0;
      double sq_dist_threshold = dist_threshold * dist_threshold;
      double sq_dist = 0;

      //we need to loop to a point on the plan that is within a certain distance of the robot
      unsigned int plan_pose_size = static_cast<unsigned int>(global_plan.size());
      while(i < plan_pose_size)
      {
        double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
        double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
        sq_dist = x_diff * x_diff + y_diff * y_diff;
        if (sq_dist <= sq_dist_threshold)
        {
          break;
        }
        ++i;
      }

      geometry_msgs::PoseStamped newer_pose;
      //now we'll transform until points are outside of our distance threshold
      while(i < plan_pose_size && sq_dist <= sq_dist_threshold)
      {
        const geometry_msgs::PoseStamped& pose = global_plan[i];
        tf2::doTransform(pose, newer_pose, plan_to_global_transform);
        transformed_plan.push_back(newer_pose);
        double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
        double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
        sq_dist = x_diff * x_diff + y_diff * y_diff;
        ++i;
      }
    }
    catch(tf2::LookupException& ex)
    {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return false;
    }
    catch(tf2::ConnectivityException& ex)
    {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return false;
    }
    catch(tf2::ExtrapolationException& ex)
    {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      if (!global_plan.empty())
        ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n",
                  global_frame.c_str(),
                  static_cast<unsigned int>(global_plan.size()),
                  global_plan[0].header.frame_id.c_str());

      return false;
    }

    return true;
  }

  void PurePlannerROS::prunePlan(const geometry_msgs::PoseStamped& global_pose, std::vector<geometry_msgs::PoseStamped>& plan, std::vector<geometry_msgs::PoseStamped>& global_plan)
  {
    ROS_ASSERT(global_plan.size() >= plan.size());
    std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();
    std::vector<geometry_msgs::PoseStamped>::iterator global_it = global_plan.begin();
    while(it != plan.end())
    {
      const geometry_msgs::PoseStamped& w = *it;
      // Fixed error bound of 2 meters for now. Can reduce to a portion of the map size or based on the resolution
      double x_diff = global_pose.pose.position.x - w.pose.position.x;
      double y_diff = global_pose.pose.position.y - w.pose.position.y;
      double distance_sq = x_diff * x_diff + y_diff * y_diff;
      if(distance_sq < 1)
      {
        break;
      }
      it = plan.erase(it);
      global_it = global_plan.erase(global_it);
    }
  }

  void PurePlannerROS::prunePlan(const geometry_msgs::PoseStamped &robot_pose, std::vector<geometry_msgs::PoseStamped> &plan, const tf2_ros::Buffer& tf)
  {
    if(plan.empty())
      return;
    geometry_msgs::PoseStamped robot_pose_tf;
    if(plan[0].header.frame_id == robot_pose.header.frame_id)
    {
      robot_pose_tf = robot_pose;
    }
    else
    {
      try
      {
        tf.transform(robot_pose, robot_pose_tf, plan[0].header.frame_id, ros::Duration(1.0));
      }
      catch(tf2::LookupException& ex)
      {
        ROS_ERROR("No Transform available Error: %s\n", ex.what());
        return;
      }
      catch(tf2::ConnectivityException& ex)
      {
        ROS_ERROR("Connectivity Error: %s\n", ex.what());
        return;
      }
      catch(tf2::ExtrapolationException& ex)
      {
        ROS_ERROR("Extrapolation Error: %s\n", ex.what());
        return;
      }
    }
    std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();
    while(it != plan.end())
    {
      const geometry_msgs::PoseStamped& w = *it;
      // Fixed error bound of 2 meters for now. Can reduce to a portion of the map size or based on the resolution
      double x_diff = robot_pose_tf.pose.position.x - w.pose.position.x;
      double y_diff = robot_pose_tf.pose.position.y - w.pose.position.y;
      double distance_sq = x_diff * x_diff + y_diff * y_diff;
      if(distance_sq < 1)
      {
        break;
      }
      it = plan.erase(it);
    }
  }

  bool PurePlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
  {
    if (! isInitialized())
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    RECORD_LOG(std::endl);
    RECORD_LOG("start compute");

    if(replan_timer<this->controller_freq_*10)
      replan_timer++;

    geometry_msgs::PoseStamped robot_pose;
    if (!costmap_ros_->getRobotPose(robot_pose))
    {
      return false;
    }

/*********************************
    double distance_local_goal = 0;
    if(!local_plan_.empty())
    {
      distance_local_goal = computeDistance(robot_pose.pose.position.x,
                                            robot_pose.pose.position.y,
                                            local_plan_.back().pose.position.x,
                                            local_plan_.back().pose.position.y);
    }
    if(distance_local_goal < 1.0)
    {
      double distance_l_goal_g_goal = 1.0;
      if(!local_plan_.empty() && !global_plan_.empty())
      {
        geometry_msgs::PoseStamped global_goal;
        geometry_msgs::PoseStamped global_goal_map = global_plan_.back();
        global_goal_map.header.stamp = ros::Time::now();
        try
        {
          this->tf_->transform(global_goal_map, global_goal, this->global_frame_,ros::Duration(1.0));
        }
        catch (tf2::ExtrapolationException ex)
        {
          ROS_ERROR("PurePlannerROS::computeVelocityCommands:%s", ex.what());
          return false;
        }
        catch (tf2::TransformException ex)
        {
          ROS_ERROR("PurePlannerROS::computeVelocityCommands:%s", ex.what());
          return false;
        }
        distance_l_goal_g_goal = computeDistance(global_goal.pose.position.x,
                                                 global_goal.pose.position.y,
                                                 local_plan_.back().pose.position.x,
                                                 local_plan_.back().pose.position.y);
      }

      if(distance_l_goal_g_goal > 0.3 || (!local_plan_.empty() && !this->checkLocalPath(local_plan_)))
      {
        RECORD_LOG("start plan 1");
        if(!getLocaPlan(robot_pose, this->local_plan_))
        {
          RECORD_LOG("Could not get local plan 1");
          if( replan_timer > this->controller_freq_*2)
          {
            this->replanGlobalpath();
            replan_timer = 0;
          }
          return false;
        }
        this->prunePlan(robot_pose, global_plan_,*tf_);
        RECORD_LOG("prunePlan ok 1");

        //if the local plan passed in is empty... we won't do anything
        if(local_plan_.empty())
          return false;

        //将局部路径传给速度计算模块
        tc_->updatePlan(local_plan_);
        RECORD_LOG("updatePlan ok 1");

        //发布局部路径
        this->publishPlan(local_plan_, l_plan_pub_);
      }
    }
    else if(!this->checkLocalPath(local_plan_))
    {
      RECORD_LOG("start plan 2");
      if(!getLocaPlan(robot_pose, this->local_plan_))
      {
        RECORD_LOG("Could not get local plan 2");
        if( replan_timer > this->controller_freq_*2)
        {
          this->replanGlobalpath();
          replan_timer = 0;
        }
        return false;
      }
      this->prunePlan(robot_pose, global_plan_,*tf_); //剪切全局路径
      RECORD_LOG("prunePlan ok 2");

      //if the local plan passed in is empty... we won't do anything
      if(local_plan_.empty())
        return false;

      //将局部路径传给速度计算模块
      tc_->updatePlan(local_plan_);
      RECORD_LOG("updatePlan ok 2");

      //发布局部路径
      this->publishPlan(local_plan_, l_plan_pub_);
    }
    **********************/

    if(!getLocaPlan(robot_pose, this->local_plan_))
    {
      RECORD_LOG("Could not get local plan 3");
      if( replan_timer > this->controller_freq_*2)
      {
        this->replanGlobalpath();
        replan_timer = 0;
      }
      return false;
    }
    this->prunePlan(robot_pose, global_plan_,*tf_); //剪切全局路径
    RECORD_LOG("prunePlan ok 3");

    //if the local plan passed in is empty... we won't do anything
    if(local_plan_.empty())
      return false;

    //将局部路径传给速度计算模块
    tc_->updatePlan(local_plan_);
    RECORD_LOG("updatePlan ok 3");

    //发布局部路径
    this->publishPlan(local_plan_, l_plan_pub_);

    //获得机器人当前速度
    tf::Stamped<tf::Pose> robot_vel;
    odom_helper_.getRobotVel(robot_vel);
    RECORD_LOG("getRobotVel ok");

    //根据当前机器人速度和局部路径计算速度指令
    bool ok = tc_->findBestAction(robot_pose, robot_vel, cmd_vel);
    RECORD_LOG("findBestAction ok");

    if(!ok)
    {
      return false;
    }
    return true;
  }

  void PurePlannerROS::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub)
  {
    //given an empty path we won't do anything
    if(path.empty())
      return;

    //create a path message
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());
    gui_path.header.frame_id = path[0].header.frame_id;
    gui_path.header.stamp = path[0].header.stamp;

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(unsigned int i=0; i < path.size(); i++)
    {
      gui_path.poses[i] = path[i];
    }

    pub.publish(gui_path);
  }

  bool PurePlannerROS::isGoalReached()
  {
    if (! isInitialized())
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    return tc_->isGoalReached();
  }

};

