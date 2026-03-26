#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <memory>
#include <mutex>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include "lilocbench_ros/tum_logger.hpp"
#include "adapt_mcl/likelihood_field.hpp"
#include "adapt_mcl/motion_model.hpp"
#include "adapt_mcl/particle_filter.hpp"
#include "adapt_mcl/soft_em_model.hpp"

namespace lilocbench_ros {

// Sensor offset: position and heading of a scanner in base_link frame.
struct SensorConfig {
  float sx{0.0f};   // x offset [m]
  float sy{0.0f};   // y offset [m]
  float st{0.0f};   // heading offset [rad]
  float z_min{0.05f};
  float z_max{30.0f};
};

class LocalizerNode {
 public:
  explicit LocalizerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh) {
    load_params(pnh);

    map_sub_ = nh_.subscribe(map_topic_, 1, &LocalizerNode::on_map, this);

    pose_pub_  = nh_.advertise<geometry_msgs::PoseStamped>("/pose_estimate", 10);
    cloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/particle_cloud", 10);

    init_pose_sub_ = nh_.subscribe(
        "/initialpose", 1, &LocalizerNode::on_initial_pose, this);
    vpr_candidates_sub_ = nh_.subscribe(
        "/vpr_candidates", 1, &LocalizerNode::on_vpr_candidates, this);

    // Independent callbacks — no synchronizer timing jitter.
    // Front scan drives PF updates; rear scan and odom are cached.
    odom_sub_       = nh_.subscribe(odom_topic_,       50, &LocalizerNode::on_odom,       this);
    rear_scan_sub_  = nh_.subscribe(scan_rear_topic_,  50, &LocalizerNode::on_rear_scan,  this);
    front_scan_sub_ = nh_.subscribe(scan_front_topic_, 50, &LocalizerNode::on_front_scan, this);

    ROS_INFO("LocalizerNode ready. Waiting for %s ...", map_topic_.c_str());
  }

  ~LocalizerNode() {
    if (logger_) {
      logger_->flush();
      TumLogger::make_run_symlinks(output_dir_ + "/" + sequence_name_ + "/run_1.txt");
      ROS_INFO("Trajectory flushed to %s/%s/run_1.txt",
               output_dir_.c_str(), sequence_name_.c_str());
    }
  }

 private:
  // ------------------------------------------------------------------ params
  void load_params(ros::NodeHandle& pnh) {
    pnh.param<std::string>("scan_front_topic", scan_front_topic_,
                           "/laser_scan_front/scan");
    pnh.param<std::string>("scan_rear_topic",  scan_rear_topic_,
                           "/laser_scan_rear/scan");
    pnh.param<std::string>("odom_topic",       odom_topic_,
                           "/dingo_velocity_controller/odom");
    pnh.param<std::string>("map_topic",        map_topic_, "/map");

    pnh.param<std::string>("sequence_name", sequence_name_, "static_0");
    pnh.param<std::string>("output_dir",    output_dir_,    "/output/results");

    pnh.param("scan_latency_correction_s", scan_latency_correction_, 0.0);

    // Sensor offsets
    pnh.param("scan_front_sx", front_.sx,  0.304559f);
    pnh.param("scan_front_sy", front_.sy, -0.005977f);
    pnh.param("scan_front_st", front_.st,  0.019208f);
    pnh.param("scan_rear_sx",  rear_.sx,  -0.302722f);
    pnh.param("scan_rear_sy",  rear_.sy,  -0.001625f);
    pnh.param("scan_rear_st",  rear_.st,  -3.130751f);
    pnh.param("z_min", front_.z_min, 0.05f); rear_.z_min = front_.z_min;
    pnh.param("z_max", front_.z_max, 30.0f); rear_.z_max = front_.z_max;

    // PF params
    pnh.param("n_particles",            pf_params_.n_particles,            1000);
    pnh.param("ess_resample_threshold", pf_params_.ess_resample_threshold, 0.5f);
    pnh.param("ess_recovery_threshold", pf_params_.ess_recovery_threshold, 0.1f);
    pnh.param("recovery_fraction",      pf_params_.recovery_fraction,      0.2f);
    pnh.param("roughening_pos_m",       pf_params_.roughening_pos_m,       0.005f);
    pnh.param("roughening_angle_rad",   pf_params_.roughening_angle_rad,   0.01f);
    pnh.param("init_spread_pos_m",      pf_params_.init_spread_pos_m,      0.10f);
    pnh.param("init_spread_angle_rad",  pf_params_.init_spread_angle_rad,  0.10f);
    pnh.param("init_random_fraction",   pf_params_.init_random_fraction,   0.05f);
    pnh.param<bool>("use_kld_sampling",  pf_params_.use_kld_sampling,  false);
    pnh.param("kld_bin_size_m",    pf_params_.kld_bin_size_m,    0.20f);
    pnh.param("kld_bin_size_rad",  pf_params_.kld_bin_size_rad,  0.20f);
    pnh.param("kld_epsilon",       pf_params_.kld_epsilon,       0.05f);
    pnh.param("kld_min_particles", pf_params_.kld_min_particles, 200);
    pnh.param("kld_max_particles", pf_params_.kld_max_particles, 5000);

    // Motion model
    pnh.param("alpha1", motion_params_.alpha1, 0.05f);
    pnh.param("alpha2", motion_params_.alpha2, 0.10f);
    pnh.param("alpha3", motion_params_.alpha3, 0.05f);
    pnh.param("alpha4", motion_params_.alpha4, 0.10f);
    pnh.param("alpha5", motion_params_.alpha5, 0.05f);

    // Sensor model
    pnh.param("n_rays",      em_params_.n_rays,      30);
    pnh.param("p_uniform",   em_params_.p_uniform,   0.033f);
    pnh.param("alpha_prior", em_params_.alpha_prior, 8.0f);
    pnh.param("beta_prior",  em_params_.beta_prior,  2.0f);
    pnh.param("em_iters",    em_params_.em_iters,    2);
    pnh.param("norm_exponent", em_params_.norm_exponent, 0.5f);

    double sigma_hit = 0.03;
    pnh.param("sigma_hit", sigma_hit, sigma_hit);
    sigma_hit_ = static_cast<float>(sigma_hit);

    // Odometry-gated updates
    pnh.param("update_min_d", update_min_d_, 0.0f);
    pnh.param("update_min_a", update_min_a_, 0.0f);

    // Global-init phase: augmented MCL injection
    pnh.param("global_init_steps",          pf_params_.global_init_steps,          0);
    pnh.param("global_init_injection_frac", pf_params_.global_init_injection_frac, 0.05f);

    // Scan-match seed for global localization
    pnh.param<bool>("use_scan_match_seed",       pf_params_.use_scan_match_seed,        false);
    pnh.param("scan_match_pos_step_m",           pf_params_.scan_match_pos_step_m,      0.4f);
    pnh.param("scan_match_angle_bins",           pf_params_.scan_match_angle_bins,      48);
    pnh.param("scan_match_top_k",                pf_params_.scan_match_top_k,           20);
    pnh.param("scan_match_candidate_frac",       pf_params_.scan_match_candidate_frac,  0.5f);

    // CAER re-ranking for global localization
    pnh.param<bool>("use_caer_rerank",       pf_params_.use_caer_rerank,       false);
    pnh.param("caer_rerank_top_n",           pf_params_.caer_rerank_top_n,     200);
    pnh.param("caer_rerank_n_rays",          pf_params_.caer_rerank_n_rays,    200);
    pnh.param("caer_inlier_threshold",       pf_params_.caer_inlier_threshold, 0.30f);

    // CPD re-ranking for global localization (outlier-aware GMM scoring)
    pnh.param<bool>("use_cpd_rerank",    pf_params_.use_cpd_rerank,    false);
    pnh.param("cpd_rerank_top_n",        pf_params_.cpd_rerank_top_n,  200);
    pnh.param("cpd_rerank_n_rays",       pf_params_.cpd_rerank_n_rays, 200);
    pnh.param("cpd_sigma",               pf_params_.cpd_sigma,         0.10f);
    pnh.param("cpd_w",                   pf_params_.cpd_w,             0.20f);

    // w_slow/w_fast adaptive recovery
    pnh.param<bool>("use_map_estimate",     pf_params_.use_map_estimate,      false);
    pnh.param<bool>("use_wslow_wfast",      pf_params_.use_wslow_wfast,       false);
    pnh.param("wslow_alpha",                pf_params_.wslow_alpha,           0.001f);
    pnh.param("wfast_alpha",                pf_params_.wfast_alpha,           0.1f);
    pnh.param("wfast_wslow_threshold",      pf_params_.wfast_wslow_threshold, 0.5f);

    // z_short sensor component
    pnh.param<bool>("use_z_short",  em_params_.use_z_short,  false);
    pnh.param("lambda_short",       em_params_.lambda_short,  2.0f);
    pnh.param("gamma_prior",        em_params_.gamma_prior,   1.0f);

    // CPD precomputed field for ongoing weight updates
    pnh.param<bool>("use_cpd_field", em_params_.use_cpd_field, false);

    // Temperature annealing for global localization convergence
    pnh.param("init_temperature_steps", pf_params_.init_temperature_steps, 0);
    pnh.param("init_temperature",       pf_params_.init_temperature,       0.05f);

    // Trajectory-consistent global init: accumulate N scans before committing.
    // Each top-K candidate is dead-reckoned independently; best total CPD score wins.
    pnh.param("trajectory_init_scans", trajectory_init_scans_, 0);
    pnh.param("trajectory_n_rays",     trajectory_n_rays_,     100);

    // Ray-swept free-space: subsample rays per scan, penalise blocked intermediate cells.
    pnh.param("trajectory_subsample",  trajectory_subsample_,  50);
    pnh.param("free_space_weight",     free_space_weight_,     0.3f);
    pnh.param("free_space_penalty",    free_space_penalty_,    1.0f);

    // Motion noise floor
    pnh.param("min_trans_noise", motion_params_.min_trans_noise, 0.0f);
    pnh.param("min_rot_noise",   motion_params_.min_rot_noise,   0.0f);

    // VPR candidates file: if set, read candidates from file after map loads
    pnh.param<std::string>("vpr_candidates_file", vpr_candidates_file_, "");
  }

  // ------------------------------------------------------------------ map
  void on_map(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    if (likelihood_field_) {
      ROS_WARN("Duplicate /map received — ignored (map already built).");
      return;
    }
    ROS_INFO("Map received: %dx%d @ %.3fm/cell",
             msg->info.width, msg->info.height, msg->info.resolution);

    adapt_mcl::LikelihoodField::MapInfo info;
    info.origin_x   = static_cast<float>(msg->info.origin.position.x);
    info.origin_y   = static_cast<float>(msg->info.origin.position.y);
    info.resolution = msg->info.resolution;
    info.width      = static_cast<int>(msg->info.width);
    info.height     = static_cast<int>(msg->info.height);

    std::vector<int8_t> data(msg->data.begin(), msg->data.end());
    // Build CPD field if needed by ongoing weight updates or trajectory scoring.
    bool need_cpd = em_params_.use_cpd_field || pf_params_.use_scan_match_seed;
    float cpd_sigma_field = need_cpd ? pf_params_.cpd_sigma : 0.0f;
    float cpd_w_field     = need_cpd ? pf_params_.cpd_w     : 0.0f;
    likelihood_field_ = std::make_shared<adapt_mcl::LikelihoodField>(
        info, data, sigma_hit_, cpd_sigma_field, cpd_w_field);

    pf_ = std::make_shared<adapt_mcl::ParticleFilter>(
        pf_params_, motion_params_, em_params_);
    pf_->initialize(*likelihood_field_, 0.0f, 0.0f, 0.0f);

    // VPR candidates file: seed PF from external candidates (overrides other init)
    if (!vpr_candidates_file_.empty()) {
      std::ifstream fin(vpr_candidates_file_);
      if (fin.is_open()) {
        std::vector<adapt_mcl::ParticleFilter::ExternalCandidate> cands;
        std::string line;
        while (std::getline(fin, line)) {
          if (line.empty() || line[0] == '#') continue;
          std::istringstream iss(line);
          float x, y, theta, weight;
          if (iss >> x >> y >> theta >> weight) {
            cands.push_back({x, y, theta, weight});
          }
        }
        if (!cands.empty()) {
          pf_->initialize_from_candidates(cands);
          est_x_ = cands[0].x; est_y_ = cands[0].y; est_theta_ = cands[0].theta;
          ROS_INFO("VPR file init: %zu candidates from %s",
                   cands.size(), vpr_candidates_file_.c_str());
          for (size_t i = 0; i < cands.size(); ++i) {
            ROS_INFO("  %zu. (%.2f, %.2f, %.1f°) w=%.3f",
                     i + 1, cands[i].x, cands[i].y,
                     cands[i].theta * 180.0f / M_PI, cands[i].weight);
          }
        } else {
          ROS_WARN("VPR file %s: no valid candidates found", vpr_candidates_file_.c_str());
        }
      } else {
        ROS_WARN("VPR file %s: cannot open", vpr_candidates_file_.c_str());
      }
    } else if (pf_params_.use_scan_match_seed) {
      global_init_pending_ = true;
      ROS_INFO("Scan-match seed enabled — will re-init on first scan.");
    }

    std::string tum_path = output_dir_ + "/" + sequence_name_ + "/run_1.txt";
    logger_ = std::make_unique<TumLogger>(tum_path);

    ROS_INFO("PF initialized.");
  }

  // ------------------------------------------------------------------ initialpose
  void on_initial_pose(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (!likelihood_field_) {
      ROS_WARN("/initialpose received before map — ignored");
      return;
    }
    const auto& p = msg->pose.pose;
    float rx = static_cast<float>(p.position.x);
    float ry = static_cast<float>(p.position.y);
    float siny = 2.0f * (p.orientation.w * p.orientation.z
                         + p.orientation.x * p.orientation.y);
    float cosy = 1.0f - 2.0f * (p.orientation.y * p.orientation.y
                                 + p.orientation.z * p.orientation.z);
    float rtheta = std::atan2(siny, cosy);
    pf_->initialize(*likelihood_field_, rx, ry, rtheta);
    est_x_ = rx; est_y_ = ry; est_theta_ = rtheta;
    ROS_INFO("Re-initialized at /initialpose (%.3f, %.3f, %.1f°)",
             rx, ry, rtheta * 180.0f / M_PI);
  }

  // ------------------------------------------------------------------ VPR candidates
  void on_vpr_candidates(const geometry_msgs::PoseArray::ConstPtr& msg) {
    if (!pf_ || !likelihood_field_) {
      ROS_WARN("/vpr_candidates received before map — ignored");
      return;
    }

    std::vector<adapt_mcl::ParticleFilter::ExternalCandidate> candidates;
    candidates.reserve(msg->poses.size());
    for (const auto& p : msg->poses) {
      float rx = static_cast<float>(p.position.x);
      float ry = static_cast<float>(p.position.y);
      float siny = 2.0f * (p.orientation.w * p.orientation.z
                            + p.orientation.x * p.orientation.y);
      float cosy = 1.0f - 2.0f * (p.orientation.y * p.orientation.y
                                    + p.orientation.z * p.orientation.z);
      float rtheta = std::atan2(siny, cosy);
      // Use position.z as weight (piggyback on unused field)
      float weight = static_cast<float>(p.position.z);
      if (weight <= 0.0f) weight = 1.0f;
      candidates.push_back({rx, ry, rtheta, weight});
    }

    if (candidates.empty()) {
      ROS_WARN("/vpr_candidates: empty message — ignored");
      return;
    }

    pf_->initialize_from_candidates(candidates);
    est_x_ = candidates[0].x;
    est_y_ = candidates[0].y;
    est_theta_ = candidates[0].theta;
    ROS_INFO("VPR init: %zu candidates, top=(%.2f, %.2f, %.1f°)",
             candidates.size(), est_x_, est_y_,
             est_theta_ * 180.0f / M_PI);
  }

  // ------------------------------------------------------------------ odom / rear scan (cached)
  void on_odom(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(state_mutex_);
    latest_odom_ = msg;
  }

  void on_rear_scan(const sensor_msgs::LaserScan::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(state_mutex_);
    latest_rear_scan_ = msg;
  }

  // ------------------------------------------------------------------ front scan → PF update
  void on_front_scan(const sensor_msgs::LaserScan::ConstPtr& front_scan) {
    if (!pf_ || !likelihood_field_) return;

    sensor_msgs::LaserScan::ConstPtr rear_scan;
    nav_msgs::Odometry::ConstPtr     odom;
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      rear_scan = latest_rear_scan_;
      odom      = latest_odom_;
    }

    if (!odom) return;  // no odom yet

    // Apply latency correction.
    double scan_time = front_scan->header.stamp.toSec() + scan_latency_correction_;

    // Extract 2D odom pose.
    const auto& pos = odom->pose.pose.position;
    const auto& ori = odom->pose.pose.orientation;
    float ox = static_cast<float>(pos.x);
    float oy = static_cast<float>(pos.y);
    float siny_o = 2.0f * (ori.w * ori.z + ori.x * ori.y);
    float cosy_o = 1.0f - 2.0f * (ori.y * ori.y + ori.z * ori.z);
    float otheta = std::atan2(siny_o, cosy_o);

    if (!odom_initialized_) {
      prev_ox_ = ox; prev_oy_ = oy; prev_otheta_ = otheta;
      odom_initialized_ = true;
    }

    // Body-frame velocity for scan undistortion.
    float vx    = static_cast<float>(odom->twist.twist.linear.x);
    float vy    = static_cast<float>(odom->twist.twist.linear.y);
    float omega = static_cast<float>(odom->twist.twist.angular.z);

    // Build endpoints from front scan, and rear scan if fresh (within 100ms).
    std::vector<std::array<float, 2>> endpoints_bl;
    endpoints_bl.reserve(front_scan->ranges.size() +
                         (rear_scan ? rear_scan->ranges.size() : 0));
    scan_to_endpoints(front_scan, front_, vx, vy, omega, endpoints_bl);
    if (rear_scan) {
      double dt_rear = std::abs(front_scan->header.stamp.toSec()
                                - rear_scan->header.stamp.toSec());
      if (dt_rear < 0.10) {
        scan_to_endpoints(rear_scan, rear_, vx, vy, omega, endpoints_bl);
      }
    }

    if (endpoints_bl.empty()) return;

    // On first scan with scan-match seed: overwrite uniform init with scan-seeded init.
    // Then skip the update for this scan so particles maintain the seeded distribution.
    if (global_init_pending_) {
      ROS_INFO("Global init: CPD search on first scan...");
      auto [ix, iy, it] = pf_->initialize_global(*likelihood_field_, endpoints_bl);
      global_init_pending_ = false;
      est_x_ = ix; est_y_ = iy; est_theta_ = it;

      if (trajectory_init_scans_ > 0) {
        // Start trajectory-consistent init: dead-reckon all top-K candidates
        // independently for N scans, pick the one with best accumulated score.
        const auto& cands = pf_->last_global_candidates();
        int nc = static_cast<int>(cands.size());
        traj_pred_poses_.resize(nc);
        traj_log_scores_.assign(nc, 0.0f);
        for (int i = 0; i < nc; ++i)
          traj_pred_poses_[i] = {cands[i].x, cands[i].y, cands[i].theta};
        trajectory_init_pending_ = true;
        trajectory_scan_count_   = 0;
        local_dx_ = 0.0f; local_dy_ = 0.0f; local_dt_ = 0.0f;
        local_ray_pairs_.clear();
        ROS_INFO("Trajectory init: accumulating %d scans across %d candidates.",
                 trajectory_init_scans_, nc);
      } else {
        ROS_INFO("Global init complete — top candidate (%.3f, %.3f, %.1f°), "
                 "particles spread across top-%d candidates, LF will select on next scan.",
                 est_x_, est_y_, est_theta_ * 180.0f / M_PI,
                 pf_params_.scan_match_top_k);
      }

      if (logger_) logger_->log(scan_time, est_x_, est_y_, est_theta_);
      publish_pose(front_scan->header.stamp, est_x_, est_y_, est_theta_, ox, oy, otheta);
      if (++pub_counter_ % 5 == 0) publish_cloud(front_scan->header.stamp);
      prev_ox_ = ox; prev_oy_ = oy; prev_otheta_ = otheta;
      return;
    }

    // Trajectory-consistent init: dead-reckon candidates, accumulate scores.
    if (trajectory_init_pending_) {
      const int n_total = static_cast<int>(endpoints_bl.size());
      const int nr = std::min(trajectory_n_rays_, n_total);

      // Differential-drive dead-reckoning deltas from odom.
      float ddx = ox - prev_ox_;
      float ddy = oy - prev_oy_;
      float trans = std::sqrt(ddx*ddx + ddy*ddy);
      float rot1  = (trans > 1e-4f)
                    ? adapt_mcl::normalize_angle(std::atan2(ddy, ddx) - prev_otheta_)
                    : 0.0f;
      float rot2  = adapt_mcl::normalize_angle(
                        adapt_mcl::normalize_angle(otheta - prev_otheta_) - rot1);

      // Update shared local dead-reckoning frame (starts at 0,0,0 at init time).
      local_dx_ += trans * std::cos(local_dt_ + rot1);
      local_dy_ += trans * std::sin(local_dt_ + rot1);
      local_dt_ = adapt_mcl::normalize_angle(local_dt_ + rot1 + rot2);

      // Subsample endpoints and transform to local frame, append to ray buffer.
      {
        const int ns = std::min(trajectory_subsample_, n_total);
        float lcos = std::cos(local_dt_), lsin = std::sin(local_dt_);
        for (int ri = 0; ri < ns; ++ri) {
          int idx = static_cast<int>(static_cast<float>(ri) / ns * n_total);
          const auto& ep = endpoints_bl[idx];
          float lx_ep = local_dx_ + lcos * ep[0] - lsin * ep[1];
          float ly_ep = local_dy_ + lsin * ep[0] + lcos * ep[1];
          local_ray_pairs_.push_back({{local_dx_, local_dy_}, {lx_ep, ly_ep}});
        }
      }

      int nc = static_cast<int>(traj_pred_poses_.size());
      for (int ci = 0; ci < nc; ++ci) {
        auto& [cx, cy, ct] = traj_pred_poses_[ci];
        // Apply motion deterministically (no noise).
        cx += trans * std::cos(ct + rot1);
        cy += trans * std::sin(ct + rot1);
        ct = adapt_mcl::normalize_angle(ct + rot1 + rot2);

        // Score current scan at this candidate's predicted pose using CPD field.
        float log_s = 0.0f;
        float ccos = std::cos(ct), csin = std::sin(ct);
        for (int ri = 0; ri < nr; ++ri) {
          int idx = static_cast<int>(static_cast<float>(ri) / nr * n_total);
          const auto& ep = endpoints_bl[idx];
          float ex = cx + ccos * ep[0] - csin * ep[1];
          float ey = cy + csin * ep[0] + ccos * ep[1];
          log_s += std::log(std::max(likelihood_field_->get_cpd_likelihood(ex, ey), 1e-6f));
        }
        traj_log_scores_[ci] += log_s;
      }

      prev_ox_ = ox; prev_oy_ = oy; prev_otheta_ = otheta;

      int best;
      if (++trajectory_scan_count_ >= trajectory_init_scans_) {
        // Final scan: compute combined CPD scan score + ray-swept free-space score.
        const auto& init_cands = pf_->last_global_candidates();
        std::vector<float> combined(nc);
        for (int ci = 0; ci < nc; ++ci) {
          float cx0 = init_cands[ci].x, cy0 = init_cands[ci].y, ct0 = init_cands[ci].theta;
          float cos_ct = std::cos(ct0), sin_ct = std::sin(ct0);
          float scan_score = 0.0f, ray_score = 0.0f;
          for (const auto& rp : local_ray_pairs_) {
            // Endpoint CPD likelihood in global frame.
            float gx_ep = cx0 + cos_ct * rp.endpoint[0] - sin_ct * rp.endpoint[1];
            float gy_ep = cy0 + sin_ct * rp.endpoint[0] + cos_ct * rp.endpoint[1];
            scan_score += std::log(std::max(
                likelihood_field_->get_cpd_likelihood(gx_ep, gy_ep), 1e-6f));
            // Ray march from robot to endpoint; count blocked intermediate cells.
            float gx_r = cx0 + cos_ct * rp.robot[0] - sin_ct * rp.robot[1];
            float gy_r = cy0 + sin_ct * rp.robot[0] + cos_ct * rp.robot[1];
            float dx = gx_ep - gx_r, dy = gy_ep - gy_r;
            float dist = std::sqrt(dx*dx + dy*dy);
            int n_steps = static_cast<int>(dist / ray_step_m_);
            if (n_steps > 1) {
              float ux = dx / dist, uy = dy / dist;
              for (int s = 1; s < n_steps; ++s) {
                float x = gx_r + s * ray_step_m_ * ux;
                float y = gy_r + s * ray_step_m_ * uy;
                if (likelihood_field_->is_occupied(x, y))
                  ray_score -= free_space_penalty_;
              }
            }
          }
          combined[ci] = (1.0f - free_space_weight_) * scan_score
                       + free_space_weight_ * ray_score;
        }
        best = static_cast<int>(
            std::max_element(combined.begin(), combined.end()) - combined.begin());
        est_x_ = traj_pred_poses_[best][0];
        est_y_ = traj_pred_poses_[best][1];
        est_theta_ = traj_pred_poses_[best][2];
        // Log top-3 candidates for diagnostics.
        {
          std::vector<int> order(nc);
          std::iota(order.begin(), order.end(), 0);
          std::partial_sort(order.begin(), order.begin() + std::min(3, nc), order.end(),
                            [&](int a, int b){ return combined[a] > combined[b]; });
          for (int k = 0; k < std::min(3, nc); ++k) {
            int ci = order[k];
            const auto& ic = init_cands[ci];
            ROS_INFO("  traj rank%d: init=(%.2f,%.2f,%.1f°) combined=%.1f",
                     k+1, ic.x, ic.y, ic.theta * 180.0f / M_PI, combined[ci]);
          }
        }
        ROS_INFO("Trajectory init done — committed to candidate %d "
                 "(%.3f, %.3f, %.1f°) after %d scans, %zu ray-pairs.",
                 best, est_x_, est_y_, est_theta_ * 180.0f / M_PI,
                 trajectory_init_scans_, local_ray_pairs_.size());
        pf_->initialize_tracking(*likelihood_field_, est_x_, est_y_, est_theta_);
        trajectory_init_pending_ = false;
        local_ray_pairs_.clear();
      } else {
        // Intermediate: use incremental CPD score for display.
        best = static_cast<int>(
            std::max_element(traj_log_scores_.begin(), traj_log_scores_.end())
            - traj_log_scores_.begin());
        est_x_ = traj_pred_poses_[best][0];
        est_y_ = traj_pred_poses_[best][1];
        est_theta_ = traj_pred_poses_[best][2];
      }

      if (logger_) logger_->log(scan_time, est_x_, est_y_, est_theta_);
      publish_pose(front_scan->header.stamp, est_x_, est_y_, est_theta_, ox, oy, otheta);
      if (++pub_counter_ % 5 == 0) publish_cloud(front_scan->header.stamp);
      return;
    }

    // Odom-gated update.
    float ddx = ox - prev_ox_;
    float ddy = oy - prev_oy_;
    float dda = std::abs(adapt_mcl::normalize_angle(otheta - prev_otheta_));
    if (std::sqrt(ddx*ddx + ddy*ddy) >= update_min_d_ || dda >= update_min_a_) {
      auto [ex, ey, et] = pf_->update(
          *likelihood_field_, endpoints_bl,
          prev_ox_, prev_oy_, prev_otheta_,
          ox, oy, otheta);
      est_x_ = ex; est_y_ = ey; est_theta_ = et;
      prev_ox_ = ox; prev_oy_ = oy; prev_otheta_ = otheta;
    }

    // Always log + publish using cached estimate.
    if (logger_) logger_->log(scan_time, est_x_, est_y_, est_theta_);
    publish_pose(front_scan->header.stamp, est_x_, est_y_, est_theta_, ox, oy, otheta);

    if (++pub_counter_ % 5 == 0) publish_cloud(front_scan->header.stamp);
  }

  // ------------------------------------------------------------------ helpers
  void scan_to_endpoints(
      const sensor_msgs::LaserScan::ConstPtr& scan,
      const SensorConfig& cfg,
      float vx, float vy, float omega,
      std::vector<std::array<float, 2>>& out) const {
    int   ray_idx = 0;
    float angle   = scan->angle_min;
    const int N   = static_cast<int>(scan->ranges.size());
    for (float r : scan->ranges) {
      if (std::isfinite(r) && r >= cfg.z_min && r <= cfg.z_max) {
        float phi_bl = angle + cfg.st;
        float ex = cfg.sx + r * std::cos(phi_bl);
        float ey = cfg.sy + r * std::sin(phi_bl);

        float dt   = static_cast<float>(ray_idx - (N - 1)) * scan->time_increment;
        float dth  = omega * dt;
        float cdth = std::cos(dth);
        float sdth = std::sin(dth);
        float ex0  = cdth * (ex + vx * dt) - sdth * (ey + vy * dt);
        float ey0  = sdth * (ex + vx * dt) + cdth * (ey + vy * dt);
        out.push_back({ex0, ey0});
      }
      angle += scan->angle_increment;
      ++ray_idx;
    }
  }

  void publish_pose(const ros::Time& stamp,
                    float x, float y, float theta,
                    float ox, float oy, float otheta) {
    // Publish pose estimate (map -> base_link) as a message.
    geometry_msgs::PoseStamped msg;
    msg.header.stamp    = stamp;
    msg.header.frame_id = "map";
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
    pose_pub_.publish(msg);

    // Publish map -> odom TF (not map -> base_link) so it chains with the
    // bag's odom -> base_link without conflict.
    // T_map_odom = T_map_baselink * inv(T_odom_baselink)
    float dth = theta - otheta;
    float mo_x = x - ox * std::cos(dth) + oy * std::sin(dth);
    float mo_y = y - ox * std::sin(dth) - oy * std::cos(dth);
    tf2::Quaternion q_mo;
    q_mo.setRPY(0, 0, dth);

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp    = ros::Time::now();
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id  = "odom";
    tf_msg.transform.translation.x = mo_x;
    tf_msg.transform.translation.y = mo_y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.x = q_mo.x();
    tf_msg.transform.rotation.y = q_mo.y();
    tf_msg.transform.rotation.z = q_mo.z();
    tf_msg.transform.rotation.w = q_mo.w();
    tf_broadcaster_.sendTransform(tf_msg);
  }

  void publish_cloud(const ros::Time& stamp) {
    geometry_msgs::PoseArray msg;
    msg.header.stamp    = stamp;
    msg.header.frame_id = "map";
    for (const auto& p : pf_->particles()) {
      geometry_msgs::Pose pose;
      pose.position.x = p.x;
      pose.position.y = p.y;
      tf2::Quaternion q;
      q.setRPY(0, 0, p.theta);
      pose.orientation.x = q.x();
      pose.orientation.y = q.y();
      pose.orientation.z = q.z();
      pose.orientation.w = q.w();
      msg.poses.push_back(pose);
    }
    cloud_pub_.publish(msg);
  }

  // ------------------------------------------------------------------ state
  ros::NodeHandle& nh_;

  std::string scan_front_topic_, scan_rear_topic_, odom_topic_, map_topic_;
  std::string sequence_name_, output_dir_;
  double scan_latency_correction_{0.0};
  float sigma_hit_{0.03f};

  SensorConfig front_, rear_;

  adapt_mcl::ParticleFilterParams pf_params_;
  adapt_mcl::MotionModelParams    motion_params_;
  adapt_mcl::SoftEmParams         em_params_;

  std::shared_ptr<adapt_mcl::LikelihoodField> likelihood_field_;
  std::shared_ptr<adapt_mcl::ParticleFilter>  pf_;
  std::unique_ptr<TumLogger>                logger_;

  // Cached latest messages (protected by mutex; front_scan callback reads them).
  std::mutex state_mutex_;
  sensor_msgs::LaserScan::ConstPtr latest_rear_scan_;
  nav_msgs::Odometry::ConstPtr     latest_odom_;

  float prev_ox_{0.0f}, prev_oy_{0.0f}, prev_otheta_{0.0f};
  bool  odom_initialized_{false};
  bool  global_init_pending_{false};
  int   pub_counter_{0};

  // Cached pose estimate (updated only when odom gate opens).
  float est_x_{0.0f}, est_y_{0.0f}, est_theta_{0.0f};
  float update_min_d_{0.0f};    // [m] odom gate threshold
  float update_min_a_{0.0f};    // [rad] odom gate threshold
  std::string vpr_candidates_file_;  // path to VPR candidates file (empty = disabled)

  // Trajectory-consistent global init state
  bool  trajectory_init_pending_{false};
  int   trajectory_scan_count_{0};
  int   trajectory_init_scans_{0};
  int   trajectory_n_rays_{100};
  std::vector<std::array<float, 3>> traj_pred_poses_;   // per-candidate (x, y, theta)
  std::vector<float>                traj_log_scores_;   // per-candidate accumulated score

  // Ray-swept free-space: shared local dead-reckoning frame + ray buffer.
  struct RayPair {
    std::array<float, 2> robot;     // (lx, ly) robot position in local frame
    std::array<float, 2> endpoint;  // (lx, ly) scan endpoint in local frame
  };
  std::vector<RayPair> local_ray_pairs_;   // subsampled rays from all scans in window
  float local_dx_{0.0f}, local_dy_{0.0f}, local_dt_{0.0f};  // shared local dead-reckoning
  int   trajectory_subsample_{50};    // rays to subsample per scan into ray buffer
  float free_space_weight_{0.3f};     // weight of ray-blocked score in combined score
  float free_space_penalty_{1.0f};    // nats penalty per blocked intermediate cell
  static constexpr float ray_step_m_{0.04f};  // step size for ray marching [m]

  ros::Subscriber map_sub_, init_pose_sub_, vpr_candidates_sub_;
  ros::Subscriber front_scan_sub_, rear_scan_sub_, odom_sub_;
  ros::Publisher  pose_pub_, cloud_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};

}  // namespace lilocbench_ros

int main(int argc, char** argv) {
  ros::init(argc, argv, "localizer_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  lilocbench_ros::LocalizerNode node(nh, pnh);
  ros::spin();
  return 0;
}
