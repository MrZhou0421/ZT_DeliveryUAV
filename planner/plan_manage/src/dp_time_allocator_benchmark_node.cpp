#include <plan_manage/dp_time_allocator.h>
#include <Eigen/Eigen>
#include <ros/ros.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <string>
#include <vector>

using ego_planner::DPTimeAllocator;

namespace
{
struct BenchConfig
{
  std::string name;
  double sample_dist;
  int vel_levels;
};

struct BenchResult
{
  BenchConfig config;
  int iterations;
  int waypoint_count;
  double path_length;
  double mean_ms;
  double std_ms;
  double min_ms;
  double max_ms;
};

std::vector<Eigen::Vector3d> makeControlPath()
{
  return {
      {5.0, -5.0, 1.5},
      {8.0, -5.0, 1.5},
      {10.0, -8.0, 1.5},
      {14.0, -8.0, 1.5},
      {10.0, -4.0, 2.4},
      {4.0, -3.5, 3.2},
      {-2.0, 0.0, 4.0},
      {-6.0, 2.0, 5.1},
      {-10.0, -1.0, 6.0},
      {-14.0, -3.0, 6.5},
  };
}

std::vector<Eigen::Vector3d> samplePath(const std::vector<Eigen::Vector3d> &control_path,
                                        double sample_dist,
                                        double &path_length)
{
  std::vector<Eigen::Vector3d> waypoints;
  path_length = 0.0;
  if (control_path.empty())
    return waypoints;

  waypoints.push_back(control_path.front());
  Eigen::Vector3d last = control_path.front();
  double carry = 0.0;

  for (size_t i = 0; i + 1 < control_path.size(); ++i)
  {
    Eigen::Vector3d a = control_path[i];
    Eigen::Vector3d b = control_path[i + 1];
    Eigen::Vector3d diff = b - a;
    double len = diff.norm();
    if (len < 1e-9)
      continue;
    path_length += len;
    Eigen::Vector3d dir = diff / len;
    double s = sample_dist - carry;
    while (s <= len + 1e-9)
    {
      Eigen::Vector3d p = a + dir * s;
      if ((p - last).norm() > 1e-6)
      {
        waypoints.push_back(p);
        last = p;
      }
      s += sample_dist;
    }
    carry = (len - std::fmod(len - carry + sample_dist, sample_dist));
    carry = std::fmod(carry + sample_dist, sample_dist);
  }

  if ((control_path.back() - waypoints.back()).norm() > 1e-6)
    waypoints.push_back(control_path.back());
  return waypoints;
}

BenchResult runConfig(const BenchConfig &config, int iterations, int warmup,
                      double max_vel, double max_acc, double payload_mass)
{
  double path_length = 0.0;
  std::vector<Eigen::Vector3d> waypoints = samplePath(makeControlPath(), config.sample_dist, path_length);
  DPTimeAllocator allocator(max_vel, max_acc, payload_mass, config.vel_levels);

  std::vector<double> samples;
  samples.reserve(iterations);
  for (int i = 0; i < warmup + iterations; ++i)
  {
    double energy = 0.0;
    double total_time = 0.0;
    std::vector<double> velocities;
    auto start = std::chrono::high_resolution_clock::now();
    bool ok = allocator.computeOptimalTime(waypoints, energy, total_time, velocities);
    auto end = std::chrono::high_resolution_clock::now();
    if (!ok)
      ROS_WARN("DP benchmark failed for config %s", config.name.c_str());
    if (i >= warmup)
    {
      double ms = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
      samples.push_back(ms);
    }
  }

  double mean = std::accumulate(samples.begin(), samples.end(), 0.0) / std::max<size_t>(1, samples.size());
  double sq = 0.0;
  for (double v : samples)
    sq += (v - mean) * (v - mean);
  double std_ms = samples.size() > 1 ? std::sqrt(sq / (samples.size() - 1)) : 0.0;

  return {config,
          static_cast<int>(samples.size()),
          static_cast<int>(waypoints.size()),
          path_length,
          mean,
          std_ms,
          *std::min_element(samples.begin(), samples.end()),
          *std::max_element(samples.begin(), samples.end())};
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dp_time_allocator_benchmark_node");
  ros::NodeHandle nh("~");

  int iterations = 100;
  int warmup = 10;
  double max_vel = 2.0;
  double max_acc = 1.0;
  double payload_mass = 0.255;
  std::string output_csv;
  nh.param("iterations", iterations, iterations);
  nh.param("warmup", warmup, warmup);
  nh.param("max_vel", max_vel, max_vel);
  nh.param("max_acc", max_acc, max_acc);
  nh.param("payload_mass", payload_mass, payload_mass);
  nh.param<std::string>("output_csv", output_csv, "");

  std::vector<BenchConfig> configs = {
      {"release_default", 0.20, 40},
      {"dense_ds", 0.10, 40},
      {"dense_ds_high_k", 0.10, 60},
  };

  std::vector<BenchResult> results;
  for (const auto &config : configs)
    results.push_back(runConfig(config, iterations, warmup, max_vel, max_acc, payload_mass));

  ROS_INFO("======================================================");
  ROS_INFO(" DPTimeAllocator benchmark (%d runs, warmup %d)", iterations, warmup);
  ROS_INFO("======================================================");
  for (const auto &r : results)
  {
    ROS_INFO(" %-16s ds=%.2fm K=%d waypoints=%d length=%.2fm mean=%.3f ms std=%.3f min=%.3f max=%.3f",
             r.config.name.c_str(), r.config.sample_dist, r.config.vel_levels,
             r.waypoint_count, r.path_length, r.mean_ms, r.std_ms, r.min_ms, r.max_ms);
  }

  if (!output_csv.empty())
  {
    std::ofstream out(output_csv);
    out << "config,path_sample_dist_m,vel_levels,waypoints,path_length_m,iterations,mean_ms,std_ms,min_ms,max_ms\n";
    out << std::fixed << std::setprecision(6);
    for (const auto &r : results)
    {
      out << r.config.name << ","
          << r.config.sample_dist << ","
          << r.config.vel_levels << ","
          << r.waypoint_count << ","
          << r.path_length << ","
          << r.iterations << ","
          << r.mean_ms << ","
          << r.std_ms << ","
          << r.min_ms << ","
          << r.max_ms << "\n";
    }
    ROS_INFO("CSV written to %s", output_csv.c_str());
  }

  return 0;
}

