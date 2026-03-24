#include "grid_map.h"

GridMap::GridMap() {};
GridMap::~GridMap() {};
GridMap::GridMap(const GridMeta& meta) : meta_(meta) {
  inv_resolution_ = 1.0 / meta_.resolution;
};

void GridMap::set_meta(const GridMeta& meta) {
  meta_ = meta;
  inv_resolution_ = 1.0 / meta_.resolution;
}

void GridMap::set_angle_resolution(const double angle_resolution) {
  inv_angle_resolution_ = 1.0 / angle_resolution;
  angle_width_ = static_cast<int>(360.0 / angle_resolution);
}

bool GridMap::in_grid(const int idx) {
  return (idx >= 0 && idx < meta_.width * meta_.height);
}

bool GridMap::in_grid(const Eigen::Vector2i &pt_g) {
  return (pt_g.x() >= 0 && pt_g.y() >= 0 &&
          pt_g.x() < meta_.width && pt_g.y() < meta_.height);
}

bool GridMap::in_grid(const Eigen::Vector2d &pt_w) {
  Eigen::Vector2i pt_g = world_to_grid(pt_w);
  return in_grid(pt_g);
}

bool GridMap::in_range(const Eigen::Vector2i &pt_g) {
  return (pt_g - Eigen::Vector2i(meta_.width / 2, meta_.height / 2)).norm() < meta_.width / 2;
}


int GridMap::grid_to_index(const Eigen::Vector2i &pt_g) {
  if (!in_grid(pt_g)) return -1;
  return pt_g.x() + meta_.width * (pt_g.y());
}

int GridMap::world_posi_to_index(const Eigen::Vector2d &pt_w) {
  Eigen::Vector2i pt_g = world_to_grid(pt_w);
  return grid_to_index(pt_g);
}

int GridMap::world_pose_to_index(const Eigen::Vector3d &po_w) {
  Eigen::Vector2i pt_g = world_to_grid(po_w.head(2));
  int pt_idx = grid_to_index(pt_g);
  if (pt_idx < 0) return -1;
  return pt_idx*angle_width_ + static_cast<int>(po_w.z() * inv_angle_resolution_);
}

Eigen::Vector2i GridMap::index_to_grid(const int idx) {
  return Eigen::Vector2i(idx % meta_.width, idx / meta_.width);
}

Eigen::Vector2i GridMap::world_to_grid(const Eigen::Vector2d &pt_w) {
  double dx = pt_w.x() - meta_.origin_x;
  double dy = pt_w.y() - meta_.origin_y;

  // return Eigen::Vector2i(
  //     static_cast<int>(dx * inv_resolution_),
  //     static_cast<int>(dy * inv_resolution_));

  double sin = std::sin(-meta_.rotation);
  double cos = std::cos(-meta_.rotation);
  double x_g = dx * cos - dy * sin;
  double y_g = dy * cos + dx * sin;

  return Eigen::Vector2i(
      static_cast<int>(x_g * inv_resolution_),
      static_cast<int>(y_g * inv_resolution_));
}

Eigen::Vector2d GridMap::grid_to_world(const Eigen::Vector2i &pt_g) {
  double x_g = (pt_g.x() + 0.5) * meta_.resolution;
  double y_g = (pt_g.y() + 0.5) * meta_.resolution;
  // return Eigen::Vector2d(x_g + meta_.origin_x, y_g + meta_.origin_y);
  double sin = std::sin(meta_.rotation);
  double cos = std::cos(meta_.rotation);
  double dx = x_g * cos - y_g * sin;
  double dy = x_g * sin + y_g * cos;
  return Eigen::Vector2d(dx + meta_.origin_x, dy + meta_.origin_y);
}






