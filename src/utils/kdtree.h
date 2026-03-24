#ifndef KD_TREE_H
#define KD_TREE_H

#include <Eigen/Dense>
#include "nanoflann.hpp"

namespace utils {
namespace kdtree {

class KDTree2D {
public:
  struct PointCloud {
    std::vector<Eigen::Vector2d> pts;

    inline size_t kdtree_get_point_count() const { return pts.size(); }
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const { return pts[idx](dim); }
    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const { return false; }
  };

  using KDTree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, PointCloud>, PointCloud, 2>;

  KDTree2D();
  
  void init(const std::vector<Eigen::Vector2d>& points);
  double get_dist(const Eigen::Vector2d& point);
  Eigen::Vector2d get_near(const Eigen::Vector2d& point);
  std::pair<double, Eigen::Vector2d> get_near_with_dist(const Eigen::Vector2d& point);
    
private:
  PointCloud cloud_;
  KDTree kdtree_;
};



} // namespace kdtree
} // namespace utils

#endif // KD_TREE_H