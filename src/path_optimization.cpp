#include <path_optimization.h>

namespace simple_local_planner
{

path_optimization::path_optimization()
{

}

path_optimization::~path_optimization()
{

}

void path_optimization::optimizationOrientation(std::vector<geometry_msgs::PoseStamped> &plan)
{
  size_t num = plan.size()-1;
  if(num < 1)
    return;
  for(size_t i=0;i<num;i++)
  {
    plan[i].pose.orientation = tf::createQuaternionMsgFromYaw( atan2( plan[i+1].pose.position.y - plan[i].pose.position.y,
                                                               plan[i+1].pose.position.x - plan[i].pose.position.x ) );
  }
}

void path_optimization::planPathSmooth(std::vector<geometry_msgs::PoseStamped>& out)
{
  std::vector<geometry_msgs::PoseStamped> tem_vec;
  tem_vec = out;

  double alpha = 0.1;
  double beta = 0.1;

  double x = 0;
  double y = 0;

  double xi = 0;
  double xi_before = 0;
  double xi_next = 0;
  double yi = 0;
  double yi_before = 0;
  double yi_next = 0;

   for(size_t k = 0; k < 5; k++) //迭代
   {
      for(size_t i = 1;i < out.size() - 1; i++)//不优化起始点和终点
      {
        x = tem_vec.at(i).pose.position.x;
        y = tem_vec.at(i).pose.position.y;

        xi = out.at(i).pose.position.x;
        xi_before = out.at(i-1).pose.position.x;
        xi_next = out.at(i+1).pose.position.x;

        yi = out.at(i).pose.position.y;
        yi_before = out.at(i-1).pose.position.y;
        yi_next = out.at(i+1).pose.position.y;

        out.at(i).pose.position.x = xi + alpha*(x - xi) + beta*(xi_before - 2*xi + xi_next);
        out.at(i).pose.position.y = yi + alpha*(y - yi) + beta*(yi_before - 2*yi + yi_next);
      }
   }
}

void path_optimization::planPathSmooth3 (std::vector<geometry_msgs::PoseStamped>& out)
{
    std::vector<geometry_msgs::PoseStamped> in = out;
    int i = 0;
    int N = out.size();
    if ( N < 3 )
    {
        return;
    }
    else
    {
        out[0].pose.position.x = ( 5.0 * in[0].pose.position.x + 2.0 * in[1].pose.position.x - in[2].pose.position.x ) / 6.0;
        out[0].pose.position.y = ( 5.0 * in[0].pose.position.y + 2.0 * in[1].pose.position.y - in[2].pose.position.y ) / 6.0;

        for ( i = 1; i <= N - 2; i++ )
        {
            out[i].pose.position.x = ( in[i - 1].pose.position.x + in[i].pose.position.x + in[i + 1].pose.position.x ) / 3.0;
            out[i].pose.position.y = ( in[i - 1].pose.position.y + in[i].pose.position.y + in[i + 1].pose.position.y ) / 3.0;
        }

        out[N - 1].pose.position.x = ( 5.0 * in[N - 1].pose.position.x + 2.0 * in[N - 2].pose.position.x - in[N - 3].pose.position.x ) / 6.0;
        out[N - 1].pose.position.y = ( 5.0 * in[N - 1].pose.position.y + 2.0 * in[N - 2].pose.position.y - in[N - 3].pose.position.y ) / 6.0;
    }
}

void path_optimization::planPathSmooth5 (std::vector<geometry_msgs::PoseStamped>& out)
{
  std::vector<geometry_msgs::PoseStamped> in = out;
  int i = 0;
  int N = out.size();
    if ( N < 5 )
    {
        return;
    }
    else
    {
        out[0].pose.position.x = ( 3.0 * in[0].pose.position.x + 2.0 * in[1].pose.position.x + in[2].pose.position.x - in[4].pose.position.x ) / 5.0;
        out[0].pose.position.y = ( 3.0 * in[0].pose.position.y + 2.0 * in[1].pose.position.y + in[2].pose.position.y - in[4].pose.position.y ) / 5.0;

        out[1].pose.position.x = ( 4.0 * in[0].pose.position.x + 3.0 * in[1].pose.position.x + 2 * in[2].pose.position.x + in[3].pose.position.x ) / 10.0;
        out[1].pose.position.y = ( 4.0 * in[0].pose.position.y + 3.0 * in[1].pose.position.y + 2 * in[2].pose.position.y + in[3].pose.position.y ) / 10.0;

        for ( i = 2; i <= N - 3; i++ )
        {
            out[i].pose.position.x = ( in[i - 2].pose.position.x + in[i - 1].pose.position.x + in[i].pose.position.x + in[i + 1].pose.position.x + in[i + 2].pose.position.x ) / 5.0;
            out[i].pose.position.y = ( in[i - 2].pose.position.y + in[i - 1].pose.position.y + in[i].pose.position.y + in[i + 1].pose.position.y + in[i + 2].pose.position.y ) / 5.0;
        }
        out[N - 2].pose.position.x = ( 4.0 * in[N - 1].pose.position.x + 3.0 * in[N - 2].pose.position.x + 2 * in[N - 3].pose.position.x + in[N - 4].pose.position.x ) / 10.0;
        out[N - 2].pose.position.y = ( 4.0 * in[N - 1].pose.position.y + 3.0 * in[N - 2].pose.position.y + 2 * in[N - 3].pose.position.y + in[N - 4].pose.position.y ) / 10.0;

        out[N - 1].pose.position.x = ( 3.0 * in[N - 1].pose.position.x + 2.0 * in[N - 2].pose.position.x + in[N - 3].pose.position.x - in[N - 5].pose.position.x ) / 5.0;
        out[N - 1].pose.position.y = ( 3.0 * in[N - 1].pose.position.y + 2.0 * in[N - 2].pose.position.y + in[N - 3].pose.position.y - in[N - 5].pose.position.y ) / 5.0;
    }
}

void path_optimization::planPathSmooth7 ( std::vector<geometry_msgs::PoseStamped>& out )
{
    std::vector<geometry_msgs::PoseStamped> in = out;
    int i = 0;
    int N = out.size();
    if ( N < 7 )
    {
        return;
    }
    else
    {
        out[0].pose.position.x = ( 13.0 * in[0].pose.position.x + 10.0 * in[1].pose.position.x +
            7.0 * in[2].pose.position.x + 4.0 * in[3].pose.position.x +in[4].pose.position.x -
            2.0 * in[5].pose.position.x - 5.0 * in[6].pose.position.x ) / 28.0;
        out[0].pose.position.y = ( 13.0 * in[0].pose.position.y + 10.0 * in[1].pose.position.y +
            7.0 * in[2].pose.position.y + 4.0 * in[3].pose.position.y +in[4].pose.position.y -
            2.0 * in[5].pose.position.y - 5.0 * in[6].pose.position.y ) / 28.0;

        out[1].pose.position.x = ( 5.0 * in[0].pose.position.x + 4.0 * in[1].pose.position.x +
            3 * in[2].pose.position.x + 2 * in[3].pose.position.x + in[4].pose.position.x -
            in[6].pose.position.x ) / 14.0;
        out[1].pose.position.y = ( 5.0 * in[0].pose.position.y + 4.0 * in[1].pose.position.y +
            3 * in[2].pose.position.y + 2 * in[3].pose.position.y + in[4].pose.position.y -
            in[6].pose.position.y ) / 14.0;

        out[2].pose.position.x = ( 7.0 * in[0].pose.position.x + 6.0 * in [1].pose.position.x +
            5.0 * in[2].pose.position.x + 4.0 * in[3].pose.position.x + 3.0 * in[4].pose.position.x +
            2.0 * in[5].pose.position.x + in[6].pose.position.x ) / 28.0;
        out[2].pose.position.y = ( 7.0 * in[0].pose.position.y + 6.0 * in [1].pose.position.y +
            5.0 * in[2].pose.position.y + 4.0 * in[3].pose.position.y + 3.0 * in[4].pose.position.y +
            2.0 * in[5].pose.position.y + in[6].pose.position.y ) / 28.0;

        for ( i = 3; i <= N - 4; i++ )
        {
            out[i].pose.position.x = ( in[i - 3].pose.position.x + in[i - 2].pose.position.x +
                in[i - 1].pose.position.x + in[i].pose.position.x + in[i + 1].pose.position.x +
                in[i + 2].pose.position.x + in[i + 3].pose.position.x ) / 7.0;
            out[i].pose.position.y = ( in[i - 3].pose.position.y + in[i - 2].pose.position.y +
                in[i - 1].pose.position.y + in[i].pose.position.y + in[i + 1].pose.position.y +
                in[i + 2].pose.position.y + in[i + 3].pose.position.y ) / 7.0;
        }

        out[N - 3].pose.position.x = ( 7.0 * in[N - 1].pose.position.x + 6.0 * in[N - 2].pose.position.x +
            5.0 * in[N - 3].pose.position.x + 4.0 * in[N - 4].pose.position.x + 3.0 * in[N - 5].pose.position.x +
            2.0 * in[N - 6].pose.position.x + in[N - 7].pose.position.x ) / 28.0;
        out[N - 3].pose.position.y = ( 7.0 * in[N - 1].pose.position.y + 6.0 * in[N - 2].pose.position.y +
            5.0 * in[N - 3].pose.position.y + 4.0 * in[N - 4].pose.position.y + 3.0 * in[N - 5].pose.position.y +
            2.0 * in[N - 6].pose.position.y + in[N - 7].pose.position.y ) / 28.0;

        out[N - 2].pose.position.x = ( 5.0 * in[N - 1].pose.position.x + 4.0 * in[N - 2].pose.position.x +
            3.0 * in[N - 3].pose.position.x + 2.0 * in[N - 4].pose.position.x + in[N - 5].pose.position.x -
            in[N - 7].pose.position.x ) / 14.0;
        out[N - 2].pose.position.y = ( 5.0 * in[N - 1].pose.position.y + 4.0 * in[N - 2].pose.position.y +
            3.0 * in[N - 3].pose.position.y + 2.0 * in[N - 4].pose.position.y + in[N - 5].pose.position.y -
            in[N - 7].pose.position.y ) / 14.0;

        out[N - 1].pose.position.x = ( 13.0 * in[N - 1].pose.position.x + 10.0 * in[N - 2].pose.position.x +
            7.0 * in[N - 3].pose.position.x + 4 * in[N - 4].pose.position.x + in[N - 5].pose.position.x -
            2 * in[N - 6].pose.position.x - 5 * in[N - 7].pose.position.x ) / 28.0;
        out[N - 1].pose.position.y = ( 13.0 * in[N - 1].pose.position.y + 10.0 * in[N - 2].pose.position.y +
            7.0 * in[N - 3].pose.position.y + 4 * in[N - 4].pose.position.y + in[N - 5].pose.position.y -
            2 * in[N - 6].pose.position.y - 5 * in[N - 7].pose.position.y ) / 28.0;
    }
}

//远离障碍物 <待继续完善>
void path_optimization::awayFromObstacles(costmap_2d::Costmap2D* costmap, std::vector<geometry_msgs::PoseStamped> &plan, double safe_distance)
{
  if(plan.empty())
  {
    ROS_INFO("PurePlannerROS::filtePath: plan is empty.");
    return;
  }
  int safe_cell = static_cast<int>( safe_distance / costmap->getResolution() );
  if(safe_cell < 1)
  {
    ROS_INFO("The safety distance is too small.");
    return;
  }
  size_t point_size = plan.size();
  geometry_msgs::PoseStamped tem_point;
  geometry_msgs::PoseStamped before_point;
  geometry_msgs::PoseStamped next_point;
  geometry_msgs::PoseStamped nearest_obstacle;
  unsigned int mx_min,mx_max,my_min,my_max,mx,my;
  for(size_t i=0;i<point_size;i++)
  {
    tem_point = plan[i];
    before_point = i>0?plan[i-1]:plan[i];
    next_point   = i<point_size-1?plan[i+1]:plan[i];

    costmap->worldToMap(tem_point.pose.position.x,tem_point.pose.position.y,mx,my);
    mx_min = mx>safe_cell?mx-safe_cell:mx;
    mx_max = mx+safe_cell<costmap->getSizeInCellsX()?mx+safe_cell:mx;
    my_min = my>safe_cell?my-safe_cell:my;
    my_max = my+safe_cell<costmap->getSizeInCellsY()?my+safe_cell:my;
    std::vector<geometry_msgs::Point> obstacle_vec;
    geometry_msgs::Point obstacle;
    obstacle_vec.clear();
    for(unsigned int j=mx_min;j<mx_max;j++) //Find all obstacles within a safe distance.
    {
      for(unsigned int k=my_min;k<my_max;k++)
      {
        if(costmap->getCost(j,k) != costmap_2d::FREE_SPACE)
        {
          costmap->mapToWorld(j,k,obstacle.x,obstacle.y);
          obstacle_vec.push_back(obstacle);
        }
      }
    }

    if(obstacle_vec.empty() != true)
    {
       //Check if the points are on the same side.
      bool same_side_flag = false;
      if(next_point.pose.position.x != before_point.pose.position.x)
      {
        double lk = 0,lb = 0,ly = 0,num = 0;
        lk = (next_point.pose.position.y-before_point.pose.position.y) / (next_point.pose.position.x-before_point.pose.position.x);
        lb = next_point.pose.position.y - lk * next_point.pose.position.x;

        for(size_t m=0;m<obstacle_vec.size();m++)
        {
          ly = lk * obstacle_vec[m].x + lb;
          if(ly != 0)
            break;
        }

        for(size_t m=0;m<obstacle_vec.size();m++)
        {
          num = ly*(lk * obstacle_vec[m].x + lb);
          if(num < 0)
          {
            same_side_flag = true;
            break;
          }
        }
      }
      else
      {
        double const_x = next_point.pose.position.x;
        double err = 0,num = 0;
        for(size_t m=0;m<obstacle_vec.size();m++)
        {
          err = const_x - obstacle_vec[m].x;
          if(err != 0)
            break;
        }
        for(size_t m=0;m<obstacle_vec.size();m++)
        {
          num = err*(const_x - obstacle_vec[m].x);
          if(num < 0)
          {
            same_side_flag = true;
            break;
          }
        }
      }

      if(same_side_flag == true)
      {
        ROS_INFO("These points are not on the same side.");
        continue;
      }

      double distance=0,min_distance_obst = 1000.0;
      size_t min_obst_index = 0;
      double diff_x,diff_y;
      for(size_t l=0;l<obstacle_vec.size();l++) //find nearest obstacle
      {
        diff_x = obstacle_vec[l].x - tem_point.pose.position.x;
        diff_y = obstacle_vec[l].y - tem_point.pose.position.y;
        distance = sqrt(diff_x*diff_x+diff_y*diff_y);
        if(min_distance_obst > distance)
        {
          min_distance_obst = distance;
          min_obst_index = l;
        }
      }

      if(safe_distance - min_distance_obst < 0.0)
      {
        continue;
      }

      nearest_obstacle.pose.position.x = obstacle_vec[min_obst_index].x;
      nearest_obstacle.pose.position.y = obstacle_vec[min_obst_index].y;

      distance =  safe_distance - min_distance_obst;
      double err_x,err_y,theta,finally_x,finally_y;
      theta = atan2(tem_point.pose.position.y-nearest_obstacle.pose.position.y,tem_point.pose.position.x-nearest_obstacle.pose.position.x);
      err_x = distance*cos(theta);
      err_y = distance*sin(theta);
      finally_x = tem_point.pose.position.x + err_x;
      finally_y = tem_point.pose.position.y + err_y;
      costmap->worldToMap(finally_x,finally_y,mx,my);
      if(costmap->getCost(mx,my) == costmap_2d::FREE_SPACE)
      {
        plan[i].pose.position.x = finally_x;
        plan[i].pose.position.y = finally_y;
      }
    }
  }
}

int path_optimization::angleLimit(std::vector<geometry_msgs::PoseStamped>& plan,double movement_angle_range)
{
  if(plan.empty())
    return 0;
  size_t pose_size = plan.size() - 1;
  double px,py,cx,cy,nx,ny,a_p,a_n;
  bool is_run = false;
  int ci = 0;
  for(ci=0;ci<1000;ci++) //最大迭代次数
  {
    is_run = false;
    for(size_t i=1;i<pose_size;i++)
    {
      px = plan[i-1].pose.position.x;
      py = plan[i-1].pose.position.y;

      cx = plan[i].pose.position.x;
      cy = plan[i].pose.position.y;

      nx = plan[i+1].pose.position.x;
      ny = plan[i+1].pose.position.y;

      a_p = normalizeAngle(atan2(cy-py,cx-px),0,2*M_PI);
      a_n = normalizeAngle(atan2(ny-cy,nx-cx),0,2*M_PI);

      if(std::max(a_p,a_n)-std::min(a_p,a_n) > movement_angle_range)
      {
        plan[i].pose.position.x = (px + nx)/2;
        plan[i].pose.position.y = (py + ny)/2;
        is_run = true;
      }
    }
    if(!is_run)
      return ci;
  }
  return ci;
}

bool path_optimization::collision(costmap_2d::Costmap2D* costmap, const double x, const double y)
{
  unsigned int mx,my;
  if(!costmap->worldToMap(x, y, mx, my))
    return true;
  if ((mx >= costmap->getSizeInCellsX()) || (my >= costmap->getSizeInCellsY()))
    return true;
//  if (!this->isAroundFree(mx, my))
  if (costmap->getCost(mx, my) != costmap_2d::FREE_SPACE)
    return true;
  return false;
}

bool path_optimization::isLineFree(costmap_2d::Costmap2D* costmap, const geometry_msgs::PoseStamped pose1, const geometry_msgs::PoseStamped pose2)
{
  std::pair<double, double> p1;
  p1.first = pose1.pose.position.x;
  p1.second = pose1.pose.position.y;

  std::pair<double, double> p2;
  p2.first = pose2.pose.position.x;
  p2.second = pose2.pose.position.y;

  double resolution = costmap->getResolution();

  std::pair<double, double> ptmp;
  ptmp.first = 0.0;
  ptmp.second = 0.0;

  double dist = sqrt( (p2.second-p1.second) * (p2.second-p1.second) +
                      (p2.first-p1.first) * (p2.first-p1.first) );
  if (dist < resolution)
  {
      return true;
  }
  else
  {
    int value = int(floor(dist/resolution));
    double theta = atan2( (p2.second - p1.second), (p2.first - p1.first) );
    int n = 1;
    for (int i = 0;i < value; i++)
    {
      ptmp.first = p1.first + resolution*cos(theta) * n;
      ptmp.second = p1.second + resolution*sin(theta) * n;
      if (collision(costmap, ptmp.first, ptmp.second))
        return false;
      n++;
    }
    return true;
  }
}

void path_optimization::cutPathPoint(costmap_2d::Costmap2D* costmap, std::vector<geometry_msgs::PoseStamped> &plan)
{
  size_t current_index = 0;
  size_t check_index = current_index+2;
  while(ros::ok())
  {
    if( current_index >= plan.size()-2 )
      return;
    if( this->isLineFree(costmap, plan[current_index], plan[check_index]) ) //点之间无障碍物
    {
      std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin() + static_cast<int>(current_index + 1) ;
      if(check_index-current_index-1 == 1)
      {
        plan.erase(it);
      }
      else
      {
        plan.erase(it,it+static_cast<int>( check_index-current_index-1) );
        check_index = current_index + 2;
      }
    }
    else
    {
      if(check_index < plan.size()-1 )
        check_index++;
      else
      {
        current_index++;
        check_index = current_index + 2;
      }
    }
  }
}

void path_optimization::insertPointForPath(std::vector<geometry_msgs::PoseStamped> &plan, double distance)
{
  std::vector<geometry_msgs::PoseStamped> path_out;
  if(plan.size() < 2)
    return;
  size_t size = plan.size() - 1;
  geometry_msgs::PoseStamped point = plan[0];
  double pp_dist = distance;
  for(size_t i=0;i<size;i++)
  {
    double theta = atan2(plan[i+1].pose.position.y - plan[i].pose.position.y,
                         plan[i+1].pose.position.x - plan[i].pose.position.x);
    size_t insert_size = static_cast<size_t>(this->distance(plan[i+1].pose.position.x, plan[i+1].pose.position.y,
                                                            plan[i].pose.position.x,plan[i].pose.position.y) / pp_dist + 0.5);
    for(size_t j=0;j<insert_size;j++)
    {
      point.pose.position.x = plan[i].pose.position.x + j * pp_dist * cos(theta);
      point.pose.position.y = plan[i].pose.position.y + j * pp_dist * sin(theta);
      path_out.push_back(point);
    }
  }
  path_out.push_back( plan.back() );
  plan.clear();
  size = path_out.size();
  plan.resize(size);
  for(size_t i=0;i<size;i++)
  {
    plan[i] = path_out[i];
  }
}

double path_optimization::distance(double x1,double y1,double x2,double y2)
{
  return sqrt( (x1-x2) * (x1-x2) + (y1-y2) * (y1-y2) );
}

double path_optimization::normalizeAngle(double val,double min,double max)
{
  float norm = 0.0;
  if (val >= min)
    norm = min + fmod((val - min), (max-min));
  else
    norm = max - fmod((min - val), (max-min));

  return norm;
}

}
