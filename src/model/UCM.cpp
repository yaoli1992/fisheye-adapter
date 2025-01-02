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

#include "model/UCM.hpp"

namespace FCA
{
namespace model
{

UCM::UCM(const std::string & model_name, const std::string & config_path)
: Base(model_name, config_path)
{
}

void UCM::parse()
{
  // Load the YAML file
  YAML::Node config;
  try {
    config = YAML::LoadFile(config_path_ + "/" + model_name_ + ".yml");
  } catch (const YAML::BadFile & e) {
    std::cerr << "Error loading YAML file: " << e.what() << std::endl;
    exit(-1);
  }

  common_params_.width = config["image"]["width"].as<int32_t>();
  common_params_.height = config["image"]["height"].as<int32_t>();

  // Read parameters
  common_params_.cx = config["parameter"]["cx"].as<double>();
  common_params_.cy = config["parameter"]["cy"].as<double>();
  common_params_.fx = config["parameter"]["fx"].as<double>();
  common_params_.fy = config["parameter"]["fy"].as<double>();
  distortion_.alpha = config["parameter"]["alpha"].as<double>();
}

void UCM::set_sample_points(const std::vector<Eigen::Vector2d> & point2d_vec){};

void UCM::initialize(
  const Base::Params & common_params, const std::vector<Eigen::Vector3d> & point3d_vec,
  const std::vector<Eigen::Vector2d> & point2d_vec)
{
  assert(point3d_vec.size() == point2d_vec.size());

  // set fx,fy,cx,cy
  common_params_ = common_params;

  // set alpha
  Eigen::MatrixXd A(point3d_vec.size() * 2, 1);
  Eigen::VectorXd b(point3d_vec.size() * 2, 1);

  for (auto i = 0U; i < point3d_vec.size(); ++i) {
    const auto & point3d = point3d_vec.at(i);
    const auto & point2d = point2d_vec.at(i);
    const double X = point3d.x();
    const double Y = point3d.y();
    const double Z = point3d.z();
    const double u = point2d.x();
    const double v = point2d.y();

    const double d = std::sqrt((X * X) + (Y * Y) + (Z * Z));
    const double u_cx = u - common_params_.cx;
    const double v_cy = v - common_params_.cy;

    A(i * 2, 0) = u_cx * (d - Z);
    A(i * 2 + 1, 0) = v_cy * (d - Z);

    b[i * 2] = (common_params_.fx * X) - (u_cx * Z);
    b[i * 2 + 1] = (common_params_.fy * Y) - (v_cy * Z);
  }

  const Eigen::VectorXd x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
  distortion_.alpha = x[0];
}

Eigen::Vector2d UCM::project(const Eigen::Vector3d & point3d, bool condition) const
{
  const double X = point3d.x();
  const double Y = point3d.y();
  const double Z = point3d.z();

  const double d = std::sqrt((X * X) + (Y * Y) + (Z * Z));
  const double denom = distortion_.alpha * d + (1.0 - distortion_.alpha) * Z;

  if (condition) {
    constexpr double PRECISION = 1e-3;
    if ((denom < PRECISION) || !check_proj_condition(Z, d, distortion_.alpha)) {
      return {-1., -1.};
    }
  }

  Eigen::Vector2d point2d;
  point2d.x() = common_params_.fx * (X / denom) + common_params_.cx;
  point2d.y() = common_params_.fy * (Y / denom) + common_params_.cy;

  return point2d;
}

Eigen::Vector3d UCM::unproject(const Eigen::Vector2d & point2d, bool condition) const
{
  const double fx = common_params_.fx;
  const double fy = common_params_.fy;
  const double cx = common_params_.cx;
  const double cy = common_params_.cy;
  const double alpha = distortion_.alpha;
  const double u = point2d.x();
  const double v = point2d.y();
  const double gamma = 1.0 - alpha;
  const double xi = alpha / gamma;
  const double mx = (u - cx) / fx * gamma;
  const double my = (v - cy) / fy * gamma;

  const double r_squared = (mx * mx) + (my * my);
  const double num = xi + std::sqrt(1.0 + (1.0 - xi * xi) * r_squared);
  const double denom = 1.0 - r_squared;

  if (condition) {
    constexpr double PRECISION = 1e-3;
    if ((denom < PRECISION) || !check_unproj_condition(r_squared, alpha)) {
      return {-1, -1, -1};
    }
  }

  const double coeff = num / denom;

  Eigen::Vector3d point3d;
  point3d = coeff * Eigen::Vector3d(mx, my, 1.0) - Eigen::Vector3d(0., 0., xi);

  return point3d;
}

bool UCM::check_proj_condition(double z, double d, double alpha)
{
  double w = (1.0 - alpha) / alpha;
  if (alpha <= 0.5) {
    w = alpha / (1.0 - alpha);
  }
  return z > -w * d;
}

bool UCM::check_unproj_condition(double r_squared, double alpha)
{
  bool condition = true;
  if (alpha > 0.5) {
    const double gamma = 1.0 - alpha;
    if (r_squared > gamma * gamma / (2 * alpha - 1.0)) {
      condition = false;
    }
  }
  return condition;
}

void UCM::optimize(
  const std::vector<Eigen::Vector3d> & point3d_vec,
  const std::vector<Eigen::Vector2d> & point2d_vec, bool display_optimization_progress)
{
  double parameters[5] = {
    common_params_.fx, common_params_.fy, common_params_.cx, common_params_.cy, distortion_.alpha};

  ceres::Problem problem;

  const auto num_pairs = point3d_vec.size();
  for (auto i = 0U; i < num_pairs; ++i) {
    const auto & point2d = point2d_vec.at(i);
    const auto & point3d = point3d_vec.at(i);
    const double gt_u = point2d.x();
    const double gt_v = point2d.y();
    const double obs_x = point3d.x();
    const double obs_y = point3d.y();
    const double obs_z = point3d.z();

    UCMAnalyticCostFunction * cost_function =
      new UCMAnalyticCostFunction(gt_u, gt_v, obs_x, obs_y, obs_z);
    problem.AddResidualBlock(cost_function, nullptr, parameters);

    // set parameters range
    constexpr double EPS = 1e-3;
    problem.SetParameterLowerBound(parameters, 4, 0.0 + EPS);  // alpha >= 0
    problem.SetParameterUpperBound(parameters, 4, 1.0 - EPS);  // alpha <= 1
  }
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  if (display_optimization_progress) {
    options.minimizer_progress_to_stdout = true;
  }

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  if (display_optimization_progress) {
    std::cout << summary.FullReport() << std::endl;
  }

  common_params_.fx = parameters[0];
  common_params_.fy = parameters[1];
  common_params_.cx = parameters[2];
  common_params_.cy = parameters[3];
  distortion_.alpha = parameters[4];
}

void UCM::print() const
{
  std::cout << model_name_ << " parameters: "
            << "fx=" << common_params_.fx << ", "
            << "fy=" << common_params_.fy << ", "
            << "cx=" << common_params_.cx << ", "
            << "cy=" << common_params_.cy << ", "
            << "alpha=" << distortion_.alpha << std::endl;
}

void UCM::save_result(const std::string & result_path) const
{
  YAML::Emitter out;

  out << YAML::BeginMap;
  out << YAML::Key << "image" << YAML::Value;
  out << YAML::BeginMap;
  out << YAML::Key << "width" << YAML::Value << common_params_.width;
  out << YAML::Key << "height" << YAML::Value << common_params_.height;
  out << YAML::EndMap;

  out << YAML::Key << "parameter" << YAML::Value;
  out << YAML::BeginMap;
  out << YAML::Key << "fx" << YAML::Value << common_params_.fx;
  out << YAML::Key << "fy" << YAML::Value << common_params_.fy;
  out << YAML::Key << "cx" << YAML::Value << common_params_.cx;
  out << YAML::Key << "cy" << YAML::Value << common_params_.cy;
  out << YAML::Key << "alpha" << YAML::Value << distortion_.alpha;
  out << YAML::EndMap;

  out << YAML::EndMap;

  std::ofstream fout(result_path + "/" + model_name_ + ".yml");
  fout << out.c_str();
  fout << std::endl;
}

void UCM::evaluate(const model::Base * const gt)
{
  const UCM * gt_model = dynamic_cast<const UCM *>(gt);

  const auto & est_pinhole_params = this->common_params_;
  const auto & gt_pinhole_params = gt_model->get_common_params();
  const auto & est_distortion_params = this->distortion_;
  const auto & gt_distortion_params = gt_model->get_distortion_params();

  const double diff_fx = est_pinhole_params.fx - gt_pinhole_params.fx;
  const double diff_fy = est_pinhole_params.fy - gt_pinhole_params.fy;
  const double diff_cx = est_pinhole_params.cx - gt_pinhole_params.cx;
  const double diff_cy = est_pinhole_params.cy - gt_pinhole_params.cy;
  const double diff_alpha = est_distortion_params.alpha - gt_distortion_params.alpha;

  const double params_diff_norm = std::sqrt(
    diff_fx * diff_fx + diff_fy * diff_fy + diff_cx * diff_cx + diff_cy * diff_cy +
    diff_alpha * diff_alpha);

  std::cout << "parameter error: " << params_diff_norm << std::endl;
}
}  // namespace model
}  // namespace FCA
