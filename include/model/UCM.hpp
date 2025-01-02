/*
MIT License

Copyright (c) 2024 Sangjun Lee, STRADVISION

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef MODEL__UCM_HPP_
#define MODEL__UCM_HPP_

#include <string>

#include "model/base.hpp"

namespace FCA
{
namespace model
{
class UCM : public Base
{
public:
  struct Params
  {
    double alpha;
  };

  UCM(const std::string & model_name, const std::string & config_path);

  void parse() override;
  void set_sample_points(const std::vector<Eigen::Vector2d> & point2d_vec) override;
  void initialize(
    const Base::Params & common_params, const std::vector<Eigen::Vector3d> & point3d_vec,
    const std::vector<Eigen::Vector2d> & point2d_vec) override;
  Eigen::Vector2d project(const Eigen::Vector3d & point3d, bool condition) const override;
  Eigen::Vector3d unproject(const Eigen::Vector2d & point2d, bool condition) const override;
  void optimize(
    const std::vector<Eigen::Vector3d> & point3d_vec,
    const std::vector<Eigen::Vector2d> & point2d_vec, bool display_optimization_progress) override;
  void print() const override;
  void save_result(const std::string & result_path) const override;

  static bool check_proj_condition(double z, double d, double alpha);
  static bool check_unproj_condition(double r_squared, double alpha);
  void evaluate(const model::Base * const gt) override;
  const Params & get_distortion_params() const { return distortion_; };
private:
  Params distortion_;
};

class UCMAnalyticCostFunction : public ceres::SizedCostFunction<2, 5>
{
public:
  UCMAnalyticCostFunction(double gt_u, double gt_v, double obs_x, double obs_y, double obs_z)
  : gt_u_(gt_u), gt_v_(gt_v), obs_x_(obs_x), obs_y_(obs_y), obs_z_(obs_z)
  {
  }

  virtual ~UCMAnalyticCostFunction() {}

  virtual bool Evaluate(
    double const * const * parameters, double * residuals, double ** jacobians) const
  {
    const double fx = parameters[0][0];
    const double fy = parameters[0][1];
    const double cx = parameters[0][2];
    const double cy = parameters[0][3];
    const double alpha = parameters[0][4];

    const double u_cx = gt_u_ - cx;
    const double v_cy = gt_v_ - cy;
    const double d = std::sqrt(obs_x_ * obs_x_ + obs_y_ * obs_y_ + obs_z_ * obs_z_);
    const double denom = alpha * d + (1.0 - alpha) * obs_z_;

    constexpr double PRECISION = 1e-3;
    if ((denom < PRECISION) || !UCM::check_proj_condition(obs_z_, d, alpha)) {
      return false;
    }

    residuals[0] = fx * obs_x_ - u_cx * denom;
    residuals[1] = fy * obs_y_ - v_cy * denom;

    if (jacobians) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 2, 5, Eigen::RowMajor>> jacobian_eucm(jacobians[0]);
        jacobian_eucm.col(0) << obs_x_, 0.0;  // ∂residual_x / ∂fx, ∂residual_y / ∂fx
        jacobian_eucm.col(1) << 0.0, obs_y_;  // ∂residual_x / ∂fy, ∂residual_y / ∂fy
        jacobian_eucm.col(2) << denom, 0.0;   // ∂residual_x / ∂cx, ∂residual_y / ∂cx
        jacobian_eucm.col(3) << 0.0, denom;   // ∂residual_x / ∂cy, ∂residual_y / ∂cy
        jacobian_eucm.col(4) << (obs_z_ - d) * u_cx,
          (obs_z_ - d) * v_cy;  // ∂residual_x / ∂alpha, ∂residual_y / ∂alpha
      }
    }

    return true;
  }

private:
  const double gt_u_, gt_v_, obs_x_, obs_y_, obs_z_;
};

struct UCMAutoDiffCostFunctor
{
  UCMAutoDiffCostFunctor(double gt_u, double gt_v, double obs_x, double obs_y, double obs_z)
  : gt_u_(gt_u), gt_v_(gt_v), obs_x_(obs_x), obs_y_(obs_y), obs_z_(obs_z)
  {
  }

  template <typename T>
  bool operator()(const T * const parameters, T * residuals) const
  {
    T fx = parameters[0];
    T fy = parameters[1];
    T cx = parameters[2];
    T cy = parameters[3];
    T alpha = parameters[4];

    const double r_squared = obs_x_ * obs_x_ + obs_y_ * obs_y_ + obs_z_ * obs_z_;
    T u_cx = gt_u_ - cx;
    T v_cy = gt_v_ - cy;
    T d = ceres::sqrt(r_squared);
    T denom = alpha * d + (1.0 - alpha) * T(obs_z_);

    constexpr double PRECISION = 1e-3;
    if ((denom < PRECISION) || !UCM::check_proj_condition(obs_z_, d, alpha)) {
      return false;
    }
    residuals[0] = fx * T(obs_x_) - u_cx * denom;
    residuals[1] = fy * T(obs_y_) - v_cy * denom;

    return true;
  }

private:
  const double gt_u_, gt_v_, obs_x_, obs_y_, obs_z_;
};

}  // namespace model
}  // namespace FCA
#endif
