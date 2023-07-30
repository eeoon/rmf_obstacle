#include "IntersectionChecker.hpp"

#include <cmath>
#include <iostream>
#include <Eigen/Dense>

//==============================================================================
namespace IntersectionChecker {

namespace {

struct CollisionBox
{
  Eigen::Vector2d min; // min x & y
  Eigen::Vector2d max; // max x & y
};

CollisionBox make_reference_box(const CollisionGeometry& o)
{
  CollisionBox box;
  box.min = Eigen::Vector2d(-o.size_x * 0.5, -o.size_y * 0.5);
  box.max = Eigen::Vector2d(o.size_x * 0.5, o.size_y * 0.5);
  return box;
}

std::pair<CollisionBox, std::vector<Eigen::Vector3d>> make_transformed_box(
  const CollisionGeometry& to,
  const CollisionGeometry& from)
{
  Eigen::Matrix3d mat;
  const double th = to.center.theta - from.center.theta;
  mat(0, 0) = std::cos(th);
  mat(0, 1) = std::sin(-th);
  mat(0, 2) = (from.size_x - to.size_x);
  mat(1, 0) = std::sin(th);
  mat(1, 1) = std::cos(th);
  mat(1, 2) = (from.size_y - to.size_y);
  mat(2, 0) = 0;
  mat(2, 1) =  0;
  mat(2, 2) = 1.0;

  std::vector<Eigen::Vector3d> vertices;
  vertices.push_back({-from.size_x * 0.5, from.size_y * 0.5, 1.0});
  vertices.push_back({-from.size_x * 0.5, -from.size_y * 0.5, 1.0});
  vertices.push_back({from.size_x * 0.5, from.size_y * 0.5, 1.0});
  vertices.push_back({from.size_x * 0.5, -from.size_y * 0.5, 1.0});

  for (auto& v : vertices)
    v = mat * v;

  CollisionBox o2_box;
  o2_box.min = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
  o2_box.max = {std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest()};

  for (const auto& v : vertices)
  {
    for (std::size_t i = 0; i < 2; ++i)
    {
      if (v[i] < o2_box.min[i])
        o2_box.min[i] = v[i];
      if (v[i] > o2_box.max[i])
        o2_box.max[i] = v[i];
    }
  }

  return {o2_box, vertices};
}

} // anonymous namespace

bool between(const CollisionGeometry& o1, const CollisionGeometry& o2, double& how_much)
{
  auto print_geom = [](const CollisionGeometry& o, const std::string& name) {
    std::cout << name << ": {" << o.size_x << ","
              << o.size_y << "," << o.center.theta << "} [" << o.size_x
              << "," << o.size_y << "]" << std::endl;
  };

  auto print_box = [](const CollisionBox& box, const std::string& name) {
    std::cout << name << "_min: " << box.min[0] << "," << box.min[1]
              << std::endl;
    std::cout << name << "_max: " << box.max[0] << "," << box.max[1]
              << std::endl;
  };

  const auto o1_box = make_reference_box(o1);
  const auto result = make_transformed_box(o1, o2);
  const auto& o2_box = result.first;

  // TODO: Consider moving this to a separate narrowphase implementation
  // to speed up compute
  how_much = std::numeric_limits<double>::min();
  for (std::size_t i = 0; i < 2; ++i)
  {
    double dist;
    // O2 projections are on the left of O1 extremas
    if (o2_box.max[i] < o1_box.min[i])
    {
      dist = std::abs(o2_box.max[i] - o1_box.min[i]);
    }
    // O2 projections are on the right of O1 extremas
    else if (o2_box.min[i] > o1_box.max[i])
    {
      dist = std::abs(o2_box.min[i] - o1_box.max[i]);
    }
    // They intersect
    else
      continue;
    how_much = std::max(how_much, dist);
  }

  // Simple SAT theorem application
  for (std::size_t i = 0; i < 2; ++i)
  {
    if (o2_box.min[i] > o1_box.max[i] || o2_box.max[i] < o1_box.min[i])
    {
      return false;
    }
  }

  return true;
}

} // namespace IntersectionChecker
