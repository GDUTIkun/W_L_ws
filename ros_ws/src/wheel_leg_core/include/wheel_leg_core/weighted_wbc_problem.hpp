#pragma once

#include <Eigen/Core>

#include "wheel_leg_core/nominal_wbc_model.hpp"

namespace wheel_leg {

enum class WeightedWbcProfile {
  kNominal,
  kPhase27Minimal,
  kPhase33ZetaManifold,
  kPhase34XiTracking,
  kPhase43NativeWheelRate,
  kPhase43XiAndNativeWheelRate,
  kPhase45ContactConsistentRolling,
  kPhase46HipCommonSafeRolling,
};

[[nodiscard]] NominalWbcModel::Matrix1x12 hipCommonSafeRollingMap(
    const NominalWbcModel::Matrix1x12 &map);

struct WbcReference {
  double base_x_acceleration_m_s2{0.0};
  double base_height_acceleration_m_s2{0.0};
  Eigen::Vector3d orientation_acceleration_rad_s2{Eigen::Vector3d::Zero()};
  Eigen::Vector4d leg_acceleration_rad_s2{Eigen::Vector4d::Zero()};
  Eigen::Vector2d wheel_vertical_acceleration_m_s2{Eigen::Vector2d::Zero()};
  Eigen::Vector2d wheel_longitudinal_acceleration_m_s2{Eigen::Vector2d::Zero()};
  Eigen::Vector2d wheel_joint_acceleration_rad_s2{Eigen::Vector2d::Zero()};
  // Phase45 contact observation. Each active row is the acceleration of the
  // current wheel-surface material point along the measured ground tangent:
  //   a_roll = rolling_acceleration_map * nudot + rolling_acceleration_bias.
  std::array<NominalWbcModel::Matrix1x12, 2> rolling_acceleration_map{
      NominalWbcModel::Matrix1x12::Zero(),
      NominalWbcModel::Matrix1x12::Zero()};
  Eigen::Vector2d rolling_acceleration_bias_m_s2{Eigen::Vector2d::Zero()};
  Eigen::Vector2d rolling_acceleration_m_s2{Eigen::Vector2d::Zero()};
  Eigen::Vector2d rolling_velocity_m_s{Eigen::Vector2d::Zero()};
  std::array<bool, 2> rolling_task_active{};
  Eigen::Matrix<double, 12, 1> interaction_wrench_flu{
      Eigen::Matrix<double, 12, 1>::Zero()};
};

class WeightedWbcProblem {
 public:
  static constexpr int kVariableCount = 42;
  static constexpr int kConstraintCount = 104;

  using Matrix42 = Eigen::Matrix<double, kVariableCount, kVariableCount>;
  using Vector42 = Eigen::Matrix<double, kVariableCount, 1>;
  using Matrix104x42 =
      Eigen::Matrix<double, kConstraintCount, kVariableCount>;
  using Vector104 = Eigen::Matrix<double, kConstraintCount, 1>;

  enum class Status { kOk, kModelRejected, kNonFinite };

  struct Result {
    Status status{Status::kModelRejected};
    NominalWbcModel::Status model_status{NominalWbcModel::Status::kInvalidState};
    Matrix42 h{Matrix42::Zero()};
    Vector42 g{Vector42::Zero()};
    Matrix104x42 a{Matrix104x42::Zero()};
    Vector104 lower{Vector104::Zero()};
    Vector104 upper{Vector104::Zero()};

    [[nodiscard]] bool ok() const { return status == Status::kOk; }
  };

  [[nodiscard]] Result assemble(
      const NominalWbcModel::Result &model,
      const WbcReference &reference,
      WeightedWbcProfile profile = WeightedWbcProfile::kNominal) const;
};

}  // namespace wheel_leg
