#ifndef SIMPLE_TRAJECTORY_H
#define SIMPLE_TRAJECTORY_H

#include <vector>

namespace simple_local_planner {
  /**
   * @class Trajectory
   * @brief Holds a trajectory generated by considering an x, y, and theta velocity
   */
  class Trajectory {
    public:
      /**
       * @brief  Default constructor
       */
      Trajectory();

      /**
       * @brief  Constructs a trajectory
       * @param xv The x velocity used to seed the trajectory
       * @param yv The y velocity used to seed the trajectory
       * @param thetav The theta velocity used to seed the trajectory
       * @param num_pts The expected number of points for a trajectory
       */
      Trajectory(double xv, double yv, double thetav, double time_delta, unsigned int num_pts);

      double xv_, yv_, thetav_; ///< @brief The x, y, and theta velocities of the trajectory

      double cost_; ///< @brief The cost/score of the trajectory

      double time_delta_; ///< @brief The time gap between points

      /**
       * @brief  Get a point within the trajectory
       * @param index The index of the point to get
       * @param x Will be set to the x position of the point
       * @param y Will be set to the y position of the point
       * @param th Will be set to the theta position of the point
       */
      void getPoint(unsigned int index, double& x, double& y, double& th) const;

      /**
       * @brief  Set a point within the trajectory
       * @param index The index of the point to set
       * @param x The x position
       * @param y The y position
       * @param th The theta position
       */
      void setPoint(unsigned int index, double x, double y, double th);

      /**
       * @brief  Add a point to the end of a trajectory
       * @param x The x position
       * @param y The y position
       * @param th The theta position
       */
      void addPoint(double x, double y, double th);

      /**
       * @brief  Get the last point of the trajectory
       * @param x Will be set to the x position of the point
       * @param y Will be set to the y position of the point
       * @param th Will be set to the theta position of the point
       */
      void getEndpoint(double& x, double& y, double& th) const;

      /**
       * @brief  Clear the trajectory's points
       */
      void resetPoints();

      /**
       * @brief  Return the number of points in the trajectory
       * @return The number of points in the trajectory
       */
      unsigned int getPointsSize() const;

    private:
      std::vector<double> x_pts_; ///< @brief The x points in the trajectory
      std::vector<double> y_pts_; ///< @brief The y points in the trajectory
      std::vector<double> th_pts_; ///< @brief The theta points in the trajectory

  };
};
#endif
