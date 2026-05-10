#include <plan_manage/dp_time_allocator.h>
#include <iostream>

using namespace std;

namespace ego_planner
{

  DPTimeAllocator::DPTimeAllocator(double v_max, double a_max, double payload_mass)
      : v_max_(v_max), a_max_(a_max), payload_mass_(payload_mass)
  {
  }

  void DPTimeAllocator::setParam(double v_max, double a_max, double payload_mass)
  {
    v_max_ = v_max;
    a_max_ = a_max;
    payload_mass_ = payload_mass;
  }

  double DPTimeAllocator::alyassi_power(double speed, double ax, double ay, double az, double payload_mass, double vz)
  {
    double v_xy = std::abs(speed);
    double a_xy = std::sqrt(ax * ax + ay * ay);
    double v_z_up = std::max(vz, 0.0);
    double v_z_down = std::max(-vz, 0.0);
    double a_z = std::abs(az);
    double mass = DRONE_BASE_MASS + payload_mass;

    double p = 
        BETA[0] * v_xy +
        BETA[1] * a_xy +
        BETA[2] * v_xy * a_xy +
        BETA[3] * v_z_up +
        BETA[4] * v_z_down +
        BETA[5] * a_z +
        BETA[6] * v_z_up * a_z +
        BETA[7] * v_z_down * a_z +
        BETA[8] * mass +
        BETA[9] * 1.0;

    return std::max(p, 0.0);
  }

  bool DPTimeAllocator::computeOptimalTime(const vector<Eigen::Vector3d> &waypoints,
                                           double &best_total_energy,
                                           double &total_time,
                                           vector<double> &optimal_velocities)
  {
    int N = waypoints.size();
    if (N < 2)
      return false;

    struct Segment
    {
      double len;
      Eigen::Vector3d dir;
    };

    vector<Segment> segments;
    for (int i = 0; i < N - 1; i++)
    {
      Eigen::Vector3d diff = waypoints[i + 1] - waypoints[i];
      double seg_len = diff.norm();
      if (seg_len < 1e-6)
        continue;
      segments.push_back({seg_len, diff / seg_len});
    }

    int M = segments.size();
    if (M == 0)
      return false;

    int v_num = 30; // default value per python script
    double v_min_grid = 0.1;

    vector<double> v_grid(v_num);
    for (int i = 0; i < v_num; i++)
    {
      v_grid[i] = v_min_grid + (v_max_ - v_min_grid) * i / std::max(1, v_num - 1);
    }

    const double INF = 1e18;
    vector<vector<double>> cost(M + 1, vector<double>(v_num, INF));
    vector<vector<int>> parent(M + 1, vector<int>(v_num, -1));

    // First segment
    for (int j = 0; j < v_num; j++)
    {
      double v_exit = v_grid[j];
      double v_avg = v_exit / 2.0;

      if (v_avg < 1e-6)
        v_avg = 1e-6;

      double seg_time = segments[0].len / v_avg;
      double a_mag = v_exit / seg_time;

      if (a_mag > a_max_ * 1.5)
        continue;

      Eigen::Vector3d dir = segments[0].dir;
      double ax = a_mag * dir(0);
      double ay = a_mag * dir(1);
      double az = a_mag * dir(2);
      double vz = v_avg * dir(2);

      double P = alyassi_power(v_avg, ax, ay, az, payload_mass_, vz);
      double energy = P * seg_time;

      cost[1][j] = energy;
      parent[1][j] = 0;
    }

    // DP iterations
    for (int i = 1; i < M; i++)
    {
      for (int j_prev = 0; j_prev < v_num; j_prev++)
      {
        if (cost[i][j_prev] >= INF)
          continue;

        double v_entry = v_grid[j_prev];

        for (int j_next = 0; j_next < v_num; j_next++)
        {
          double v_exit = v_grid[j_next];
          double v_avg = (v_entry + v_exit) / 2.0;
          if (v_avg < 1e-6)
            v_avg = 1e-6;

          double seg_time = segments[i].len / v_avg;
          double a_mag = std::abs(v_exit - v_entry) / seg_time;

          if (a_mag > a_max_ * 1.5)
            continue;

          Eigen::Vector3d dir = segments[i].dir;
          double ax = a_mag * dir(0);
          double ay = a_mag * dir(1);
          double az = a_mag * dir(2);
          double vz = v_avg * dir(2);

          double P = alyassi_power(v_avg, ax, ay, az, payload_mass_, vz);
          double energy = P * seg_time;
          double new_cost = cost[i][j_prev] + energy;

          if (new_cost < cost[i + 1][j_next])
          {
            cost[i + 1][j_next] = new_cost;
            parent[i + 1][j_next] = j_prev;
          }
        }
      }
    }

    // Backtrack
    best_total_energy = INF;
    int best_j = 0;

    for (int j = 0; j < v_num; j++)
    {
      if (cost[M][j] >= INF)
        continue;

      double v_final = v_grid[j];
      double decel_time = a_max_ > 0 ? v_final / a_max_ : 0;
      double P_decel = alyassi_power(v_final / 2.0, 0, 0, 0, payload_mass_, 0);
      double decel_energy = P_decel * decel_time;

      double current_total = cost[M][j] + decel_energy;
      if (current_total < best_total_energy)
      {
        best_total_energy = current_total;
        best_j = j;
      }
    }

    if (best_total_energy >= INF)
      return false;

    optimal_velocities.clear();
    int idx = best_j;
    optimal_velocities.push_back(v_grid[best_j]);

    for (int i = M; i > 1; i--)
    {
      idx = parent[i][idx];
      optimal_velocities.push_back(v_grid[idx]);
    }
    std::reverse(optimal_velocities.begin(), optimal_velocities.end());

    total_time = 0.0;
    for (int i = 0; i < M; i++)
    {
      double v_entry = i < (int)optimal_velocities.size() ? optimal_velocities[i] : v_grid[0];
      double v_exit = i + 1 < (int)optimal_velocities.size() ? optimal_velocities[i + 1] : 0.1;
      double v_avg = (v_entry + v_exit) / 2.0;

      if (v_avg < 1e-6)
        v_avg = 0.1;

      total_time += segments[i].len / v_avg;
    }
    
    // Deceleration time
    total_time += v_grid[best_j] / a_max_;

    return true;
  }

} // namespace ego_planner
