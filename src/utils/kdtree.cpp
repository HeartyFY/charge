#include "kdtree.h"

namespace utils {
namespace kdtree {

KDTree2D::KDTree2D() : kdtree_(2, cloud_, nanoflann::KDTreeSingleIndexAdaptorParams(10)) {}

void KDTree2D::init(const std::vector<Eigen::Vector2d>& points) {
  cloud_.pts = points;
  kdtree_.buildIndex();
}
    
double KDTree2D::get_dist(const Eigen::Vector2d& point) {
  size_t ret_index;
  double out_dist_sqr;
  nanoflann::KNNResultSet<double> resultSet(1);
  resultSet.init(&ret_index, &out_dist_sqr);
  kdtree_.findNeighbors(resultSet, point.data(), nanoflann::SearchParameters(10));
  return std::sqrt(out_dist_sqr);
}
    
Eigen::Vector2d KDTree2D::get_near(const Eigen::Vector2d& point) {
  size_t ret_index;
  double out_dist_sqr;
  nanoflann::KNNResultSet<double> resultSet(1);
  resultSet.init(&ret_index, &out_dist_sqr);
  kdtree_.findNeighbors(resultSet, point.data(), nanoflann::SearchParameters(10));
  return cloud_.pts[ret_index];
}

std::pair<double, Eigen::Vector2d> KDTree2D::get_near_with_dist(const Eigen::Vector2d& point) {
  size_t ret_index;
  double out_dist_sqr;
  nanoflann::KNNResultSet<double> resultSet(1);
  resultSet.init(&ret_index, &out_dist_sqr);
  kdtree_.findNeighbors(resultSet, point.data(), nanoflann::SearchParameters(10));
  return std::make_pair(std::sqrt(out_dist_sqr), cloud_.pts[ret_index]);
}

} // namespace kdtree
} // namespace utils
