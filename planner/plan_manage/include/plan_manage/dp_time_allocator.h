#ifndef _DP_TIME_ALLOCATOR_H_
#define _DP_TIME_ALLOCATOR_H_

#include <Eigen/Eigen>
#include <vector>
#include <iostream>
#include <cmath>

namespace ego_planner
{
  class DPTimeAllocator
  {
  public:
    DPTimeAllocator(double v_max = 2.0, double a_max = 1.0, double payload_mass = 0.255);
    ~DPTimeAllocator() {}

    // Main API: taking a spatial path and returning an optimal velocity profile and total time
    bool computeOptimalTime(const std::vector<Eigen::Vector3d>& waypoints, 
                            double& best_total_energy, 
                            double& total_time, 
                            std::vector<double>& optimal_velocities);

    void setParam(double v_max, double a_max, double payload_mass);

    double getVMax() const { return v_max_; }
    double getAMax() const { return a_max_; }
    double getPayloadMass() const { return payload_mass_; }

  private:
    double v_max_;
    double a_max_;
    double payload_mass_;

    double DRONE_BASE_MASS = 2.614;

    // Alyassi model coefficients
    const double BETA[10] = {
      -4.6322,   // v_xy
       1.6071,   // a_xy
      -1.7923,   // v_xy * a_xy
      37.3983,   // v_z_up
      32.8667,   // v_z_down
       2.2051,   // a_z
      -1.1254,   // v_z_up * a_z
      -7.7656,   // v_z_down * a_z
     479.1578,   // mass (drone + payload)
    -597.2565    // intercept
    };

    double alyassi_power(double speed, double ax, double ay, double az, double payload_mass, double vz);
  };
} // namespace ego_planner

#endif
