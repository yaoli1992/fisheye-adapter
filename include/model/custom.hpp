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

#ifndef MODEL__CUSTOM_HPP_
#define MODEL__CUSTOM_HPP_

#include <string>

#include "model/base.hpp"

namespace FCA
{
namespace model
{
class Custom : public Base
{
public:
  struct Params
  {
  };

  Custom(const std::string & model_name, const std::string & config_path);

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
  void evaluate(const model::Base * const gt) override;
  const Params & get_distortion_params() const { return distortion_; };
private:
  Params distortion_;
};

class CustomAnalyticCostFunction : public ceres::SizedCostFunction<2, 6>
{
public:
  CustomAnalyticCostFunction(double gt_u, double gt_v, double obs_x, double obs_y, double obs_z)
  : gt_u_(gt_u), gt_v_(gt_v), obs_x_(obs_x), obs_y_(obs_y), obs_z_(obs_z)
  {
  }

  virtual ~CustomAnalyticCostFunction() {}

  virtual bool Evaluate(
    double const * const * parameters, double * residuals, double ** jacobians) const
  {
    return true;
  }

private:
  const double gt_u_, gt_v_, obs_x_, obs_y_, obs_z_;
};

struct CustomAutoDiffCostFunctor
{
  CustomAutoDiffCostFunctor(double gt_u, double gt_v, double obs_x, double obs_y, double obs_z)
  : gt_u_(gt_u), gt_v_(gt_v), obs_x_(obs_x), obs_y_(obs_y), obs_z_(obs_z)
  {
  }

  template <typename T>
  bool operator()(const T * const parameters, T * residuals) const
  {
    return true;
  }

private:
  const double gt_u_, gt_v_, obs_x_, obs_y_, obs_z_;
};
}  // namespace model
}  // namespace FCA
#endif
