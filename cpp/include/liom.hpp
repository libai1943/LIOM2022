#pragma once
#include <array>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

namespace liom {
using Vector = std::vector<double>;
using Matrix = std::vector<Vector>;
using Box = std::array<double, 4>; // xmin, xmax, ymin, ymax
using Boxes = std::vector<Box>;
struct Problem {
  Vector p;
  Matrix reference, grid, obstacles;
  static Problem load(const std::filesystem::path& directory);
  int n() const { return static_cast<int>(p[0]); }
  double radius() const;
  std::array<double, 2> offsets() const;
  Vector initial_vector() const;
};
class Corridors {
 public:
  explicit Corridors(const Problem& problem);
  std::pair<Boxes, Boxes> build(const Vector& z) const;
 private:
  const Problem& problem_;
  double dx_, dy_;
  bool point_free(double x, double y) const;
  bool strip_free(const Box& box) const;
  Box box(double x, double y) const;
};
std::pair<Vector, Vector> bounds(const Problem&, const Boxes&, const Boxes&);
struct Result {
  Vector solution;
  Matrix statistics, history;
  double max_bound_violation = 0, min_disc_clearance = 0;
};
Result optimize(const Problem&, const std::string& linear_solver,
                const std::string& hsl_library);
void save(const Problem&, const Result&, const std::filesystem::path& directory);
} // namespace liom
