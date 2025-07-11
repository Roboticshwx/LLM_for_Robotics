#include <ik_impedance_controller/motion_generator.hpp>

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <utility>

#include <Eigen/Core>

MotionGenerator::MotionGenerator(
  double speed_factor, // 速度缩放因子
  const Vector7d& q_start,
  const Vector7d& q_goal
  ) : q_start_(q_start) { // 初始化成员变量
  assert(speed_factor > 0);
  assert(speed_factor <= 1);
  delta_q_ = q_goal - q_start; // 计算目标关节位置与起始关节位置之间的差值
  dq_max_ *= speed_factor;  // 缩放后的最大速度
  ddq_max_start_ *= speed_factor;  // 缩放后的起始阶段最大加速度
  ddq_max_goal_ *= speed_factor;   // 缩放后的结束阶段最大加速度

  calculateSynchronizedValues();  // 同步各关节运动时间参数
}

bool MotionGenerator::calculateDesiredValues(double time, Vector7d* delta_q_d) const {
  Vector7i sign_delta_q;
  sign_delta_q << delta_q_.cwiseSign().cast<int>();

  Vector7d t_d = t_2_sync_ - t_1_sync_; // 匀速阶段持续时间
  Vector7d delta_t_2_sync = t_f_sync_ - t_2_sync_;  // 减速阶段持续时间
  std::array<bool, kJoints> joint_motion_finished{};  // 各关节运动完成标志

  for (auto i = 0; i < kJoints; i++) {
    if (std::abs(delta_q_[i]) < kDeltaQMotionFinished) { // 小于一个微小角度的时候已完成
      (*delta_q_d)[i] = 0;
      joint_motion_finished.at(i) = true;
    } else {
      if (time < t_1_sync_[i]) {  // 加速阶段(三次多项式)
        (*delta_q_d)[i] = -1.0 / std::pow(t_1_sync_[i], 3.0) * dq_max_sync_[i] * sign_delta_q[i] *
                          (0.5 * time - t_1_sync_[i]) * std::pow(time, 3.0);
      } else if (time >= t_1_sync_[i] && time < t_2_sync_[i]) {  // 匀速阶段
        (*delta_q_d)[i] = q_1_[i] + (time - t_1_sync_[i]) * dq_max_sync_[i] * sign_delta_q[i];
      } else if (time >= t_2_sync_[i] && time < t_f_sync_[i]) {  // 减速阶段(三次多项式)
        (*delta_q_d)[i] =
            delta_q_[i] +
            0.5 *
                (1.0 / std::pow(delta_t_2_sync[i], 3.0) *
                     (time - t_1_sync_[i] - 2.0 * delta_t_2_sync[i] - t_d[i]) *
                      std::pow((time - t_1_sync_[i] - t_d[i]), 3.0) +
                 (2.0 * time - 2.0 * t_1_sync_[i] - delta_t_2_sync[i] - 2.0 * t_d[i])) *
                dq_max_sync_[i] * sign_delta_q[i];
      } else {  // 运动完成
        (*delta_q_d)[i] = delta_q_[i];
        joint_motion_finished.at(i) = true;
      }
    }
  }
  // 检查所有关节是否完成运动
  return std::all_of(joint_motion_finished.cbegin(), joint_motion_finished.cend(),
                      [](bool each_joint_finished) { return each_joint_finished; });
}

void MotionGenerator::calculateSynchronizedValues() {
  Vector7d dq_max_reach(dq_max_);
  Vector7d t_f = Vector7d::Zero();
  Vector7d delta_t_2 = Vector7d::Zero();
  Vector7d t_1 = Vector7d::Zero();
  Vector7d delta_t_2_sync = Vector7d::Zero();
  Vector7i sign_delta_q;
  sign_delta_q << delta_q_.cwiseSign().cast<int>();

  for (auto i = 0; i < kJoints; i++) {
    if (std::abs(delta_q_[i]) > kDeltaQMotionFinished) {
      if (std::abs(delta_q_[i]) < (3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_start_[i]) +
                                   3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_goal_[i]))) {
        dq_max_reach[i] = std::sqrt(4.0 / 3.0 * delta_q_[i] * sign_delta_q[i] *
                                    (ddq_max_start_[i] * ddq_max_goal_[i]) /
                                    (ddq_max_start_[i] + ddq_max_goal_[i]));
      }
      t_1[i] = 1.5 * dq_max_reach[i] / ddq_max_start_[i];
      delta_t_2[i] = 1.5 * dq_max_reach[i] / ddq_max_goal_[i];
      t_f[i] = t_1[i] / 2.0 + delta_t_2[i] / 2.0 + std::abs(delta_q_[i]) / dq_max_reach[i];
    }
  }
  double max_t_f = t_f.maxCoeff();
  for (auto i = 0; i < kJoints; i++) {
    if (std::abs(delta_q_[i]) > kDeltaQMotionFinished) {
      double param_a = 1.5 / 2.0 * (ddq_max_goal_[i] + ddq_max_start_[i]);
      double param_b = -1.0 * max_t_f * ddq_max_goal_[i] * ddq_max_start_[i];
      double param_c = std::abs(delta_q_[i]) * ddq_max_goal_[i] * ddq_max_start_[i];
      double delta = param_b * param_b - 4.0 * param_a * param_c;
      if (delta < 0.0) {
        delta = 0.0;
      }
      dq_max_sync_[i] = (-1.0 * param_b - std::sqrt(delta)) / (2.0 * param_a);
      t_1_sync_[i] = 1.5 * dq_max_sync_[i] / ddq_max_start_[i];
      delta_t_2_sync[i] = 1.5 * dq_max_sync_[i] / ddq_max_goal_[i];
      t_f_sync_[i] =
          (t_1_sync_)[i] / 2.0 + delta_t_2_sync[i] / 2.0 + std::abs(delta_q_[i] / dq_max_sync_[i]);
      t_2_sync_[i] = (t_f_sync_)[i] - delta_t_2_sync[i];
      q_1_[i] = (dq_max_sync_)[i] * sign_delta_q[i] * (0.5 * (t_1_sync_)[i]);
    }
  }
}

std::pair<MotionGenerator::Vector7d, bool> MotionGenerator::getDesiredJointPositions(
    const rclcpp::Duration& trajectory_time) {
  time_ = trajectory_time.seconds();  // 获取当前时间

  Vector7d delta_q_d;  // 当前时刻的关节位移增量
  bool motion_finished = calculateDesiredValues(time_, &delta_q_d); // 计算增量

  std::array<double, kJoints> joint_positions{};
  Eigen::VectorXd::Map(joint_positions.data(), kJoints) = (q_start_ + delta_q_d);
  return std::make_pair(q_start_ + delta_q_d, motion_finished);  // 计算最终关节位置
}