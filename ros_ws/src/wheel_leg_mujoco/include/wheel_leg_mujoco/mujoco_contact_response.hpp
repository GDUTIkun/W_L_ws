#pragma once
#include <string>
#include <vector>
#include <Eigen/Core>
#include "mujoco/mujoco.h"
#include "wheel_leg_core/nominal_wbc_model.hpp"
namespace wheel_leg_mujoco {
// WARNING: MUJOCO-DEPENDENT SIMULATION-ONLY primitive contact law.
// Never deploy this realization to hardware.
struct MujocoContactResponse {
  bool ok{}, same_snapshot{}, acceleration_lift_legal{}, primitive_contact_law{};
  bool point_force_decode{}, rank5_aggregate{}, generalized_commuting{};
  bool production_dynamics_compatible{}, active_set_consistent{};
  std::string failure, active_set_signature;
  std::vector<int> contact_rows;
  Eigen::Matrix<double,12,12> decision_nudot{Eigen::Matrix<double,12,12>::Zero()};
  Eigen::Matrix<double,12,12> decision_wrench{Eigen::Matrix<double,12,12>::Zero()};
  Eigen::Matrix<double,12,1> decision_rhs{Eigen::Matrix<double,12,1>::Zero()};
  Eigen::Matrix<double,12,1> aggregate_force0{Eigen::Matrix<double,12,1>::Zero()};
  Eigen::Matrix<double,12,12> aggregate_force_nudot{Eigen::Matrix<double,12,12>::Zero()};
  Eigen::Matrix<double,12,16> k_a_reduced{Eigen::Matrix<double,12,16>::Zero()};
  Eigen::Matrix<double,12,16> k_b_reduced{Eigen::Matrix<double,12,16>::Zero()};
  Eigen::Matrix<double,12,16> k_c_reduced{Eigen::Matrix<double,12,16>::Zero()};
  Eigen::VectorXd contact_row_force0;
  Eigen::MatrixXd contact_row_force_nudot;
  int decision_row_rank{}, current_hard_equality_rank{12};
  int hard_equality_rank_with_response{}, incremental_rank{};
  double decision_row_condition{}, same_snapshot_max_abs{};
  double raw_qacc_diagnostic_max_abs{}, acceleration_lift_residual{};
  double primitive_law_residual{}, point_force_decode_residual{};
  double rank5_projector_residual{}, generalized_commuting_residual{};
  double affine_offset_residual{}, affine_nudot_residual{};
  double static_ab_residual{}, static_bc_residual{}, static_ac_residual{};
  double historical_operator_residual{}, historical_offset_residual{};
  int dominant_nudot_column{-1}, dominant_contact_row{-1};
  int dominant_wheel{-1}, dominant_generalized_force_dof{-1};
  double production_dynamics_residual{}, minimum_predicted_contact_row_force{};
  Eigen::VectorXd predictedContactRowForce(const Eigen::Matrix<double,12,1>& nudot) const;
};
MujocoContactResponse buildMujocoContactResponse(
    const mjModel*, const mjData*, const wheel_leg::NominalWbcModel::Result&);
}  // namespace wheel_leg_mujoco
