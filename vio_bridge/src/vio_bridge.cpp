// rowpilot VIO bridge: D435i stereo infrared + IMU (from Gazebo) -> OpenVINS -> odometry.
//
//   vio_bridge <estimator_config.yaml> [topic_prefix=/gh_x500] [out_topic=/rowpilot/vio/odometry]
//
// Published gz.msgs.Odometry, in OpenVINS's global frame G (gravity-aligned, z up, origin and yaw
// fixed at initialisation), for the IMU (= D435i) frame:
//   pose.position     p_IinG [m]
//   pose.orientation  q_ItoG (Hamilton, x y z w)
//   twist.linear      v_IinG [m/s] (in G, not in the body frame)
// header.data "status" = "initialized" | "waiting"
//
// Measurements are fed to OpenVINS strictly in time order: a stereo pair is only processed once
// IMU data up to its timestamp has arrived. On the real drone, the same loop takes frames from
// librealsense instead of Gazebo.
#include <atomic>
#include <cmath>
#include <functional>
#include <csignal>
#include <deque>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>

#include <gz/msgs/image.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/transport/Node.hh>
#include <opencv2/core.hpp>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/State.h"
#include "types/IMU.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

namespace {
std::atomic<bool> g_run{true};
std::mutex g_mtx;
std::deque<ov_core::ImuData> g_imu;
std::map<double, cv::Mat> g_left, g_right;

double stamp(const gz::msgs::Header &h) { return h.stamp().sec() + 1e-9 * h.stamp().nsec(); }

cv::Mat to_mat(const gz::msgs::Image &m) {
  return cv::Mat(m.height(), m.width(), CV_8UC1, const_cast<char *>(m.data().data())).clone();
}
}  // namespace

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "usage: vio_bridge <estimator_config.yaml> [topic_prefix] [out_topic]\n";
    return 2;
  }
  const std::string prefix = argc > 2 ? argv[2] : "/gh_x500";
  const std::string out_topic = argc > 3 ? argv[3] : "/rowpilot/vio/odometry";
  std::signal(SIGINT, [](int) { g_run = false; });
  std::signal(SIGTERM, [](int) { g_run = false; });

  auto parser = std::make_shared<ov_core::YamlParser>(argv[1]);
  ov_msckf::VioManagerOptions params;
  params.print_and_load(parser);
  params.use_multi_threading_subs = false;
  if (!parser->successful()) {
    std::cerr << "could not load OpenVINS config " << argv[1] << "\n";
    return 1;
  }
  auto vio = std::make_shared<ov_msckf::VioManager>(params);

  gz::transport::Node node;
  std::function<void(const gz::msgs::IMU &)> on_imu = [](const gz::msgs::IMU &m) {
    ov_core::ImuData d;
    d.timestamp = stamp(m.header());
    d.wm << m.angular_velocity().x(), m.angular_velocity().y(), m.angular_velocity().z();
    d.am << m.linear_acceleration().x(), m.linear_acceleration().y(), m.linear_acceleration().z();
    std::lock_guard<std::mutex> lk(g_mtx);
    g_imu.push_back(d);
  };
  std::function<void(const gz::msgs::Image &)> on_left = [](const gz::msgs::Image &m) {
    std::lock_guard<std::mutex> lk(g_mtx);
    g_left[stamp(m.header())] = to_mat(m);
  };
  std::function<void(const gz::msgs::Image &)> on_right = [](const gz::msgs::Image &m) {
    std::lock_guard<std::mutex> lk(g_mtx);
    g_right[stamp(m.header())] = to_mat(m);
  };
  node.Subscribe(prefix + "/d435i_imu", on_imu);
  node.Subscribe(prefix + "/infra1", on_left);
  node.Subscribe(prefix + "/infra2", on_right);
  auto pub = node.Advertise<gz::msgs::Odometry>(out_topic);
  std::cout << "[vio_bridge] listening on " << prefix << "/{infra1,infra2,d435i_imu}, publishing " << out_topic
            << std::endl;

  double last_imu_t = -1, last_report = -1;
  size_t frames = 0;
  while (g_run) {
    std::deque<ov_core::ImuData> imu;
    std::vector<std::pair<double, std::pair<cv::Mat, cv::Mat>>> pairs;
    {
      std::lock_guard<std::mutex> lk(g_mtx);
      imu.swap(g_imu);
      if (!imu.empty()) last_imu_t = std::max(last_imu_t, imu.back().timestamp);
      for (auto it = g_left.begin(); it != g_left.end();) {
        auto r = g_right.find(it->first);
        if (r != g_right.end() && it->first <= last_imu_t) {
          pairs.push_back({it->first, {it->second, r->second}});
          g_right.erase(r);
          it = g_left.erase(it);
        } else {
          ++it;
        }
      }
      // drop unmatched images that are too old to ever be paired
      while (!g_left.empty() && g_left.begin()->first < last_imu_t - 1.0) g_left.erase(g_left.begin());
      while (!g_right.empty() && g_right.begin()->first < last_imu_t - 1.0) g_right.erase(g_right.begin());
    }
    for (const auto &d : imu) vio->feed_measurement_imu(d);
    for (auto &p : pairs) {
      ov_core::CameraData cam;
      cam.timestamp = p.first;
      cam.sensor_ids = {0, 1};
      cam.images = {p.second.first, p.second.second};
      cam.masks = {cv::Mat::zeros(p.second.first.size(), CV_8UC1), cv::Mat::zeros(p.second.second.size(), CV_8UC1)};
      vio->feed_measurement_camera(cam);
      frames++;

      gz::msgs::Odometry out;
      out.mutable_header()->mutable_stamp()->set_sec(static_cast<int64_t>(p.first));
      out.mutable_header()->mutable_stamp()->set_nsec(static_cast<int32_t>((p.first - std::floor(p.first)) * 1e9));
      auto *status = out.mutable_header()->add_data();
      status->set_key("status");
      // OpenVINS's initialized() stays false until the first visual update, which zero-velocity
      // updates postpone while the drone stands still. The state is valid from initialisation on
      // (its timestamp is set then), and PX4/ArduPilot need the pose before take-off.
      auto st = vio->get_state();
      if (st->_timestamp > 0) {
        Eigen::Matrix3d R_ItoG = ov_core::quat_2_Rot(st->_imu->quat()).transpose();
        Eigen::Quaterniond q(R_ItoG);
        Eigen::Vector3d p_IinG = st->_imu->pos(), v_IinG = st->_imu->vel();
        status->add_value("initialized");
        auto *pose = out.mutable_pose();
        pose->mutable_position()->set_x(p_IinG.x());
        pose->mutable_position()->set_y(p_IinG.y());
        pose->mutable_position()->set_z(p_IinG.z());
        pose->mutable_orientation()->set_w(q.w());
        pose->mutable_orientation()->set_x(q.x());
        pose->mutable_orientation()->set_y(q.y());
        pose->mutable_orientation()->set_z(q.z());
        out.mutable_twist()->mutable_linear()->set_x(v_IinG.x());
        out.mutable_twist()->mutable_linear()->set_y(v_IinG.y());
        out.mutable_twist()->mutable_linear()->set_z(v_IinG.z());
      } else {
        status->add_value("waiting");
      }
      pub.Publish(out);
      if (p.first - last_report > 5.0) {
        last_report = p.first;
        std::cout << "[vio_bridge] t=" << p.first << " frames=" << frames
                  << (vio->get_state()->_timestamp > 0 ? " initialized" : " waiting for initialisation") << std::endl;
      }
    }
    if (pairs.empty()) std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return 0;
}
