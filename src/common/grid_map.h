#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <Eigen/Dense>

struct GridMeta {
  int width;
  int height;
  double resolution;
  double origin_x;
  double origin_y;
  double rotation;
};

class GridMap {
public:
  GridMap();
  GridMap(const GridMeta& meta);
  ~GridMap();

  void set_meta(const GridMeta& meta);
  // void set_origin(double x, double y, double rotation);
  // void set_size(int width, int height, double resolution);
  void set_angle_resolution(const double angle_resolution);

  double get_resolution() const { return meta_.resolution; }
  
  bool in_grid(const int idx);
  bool in_grid(const Eigen::Vector2i &pt_g);
  bool in_grid(const Eigen::Vector2d &pt_w);
  bool in_range(const Eigen::Vector2i &pt_g);

  int grid_to_index(const Eigen::Vector2i &pt_g);
  int world_posi_to_index(const Eigen::Vector2d &pt_w);
  int world_pose_to_index(const Eigen::Vector3d &po_w);
  
  Eigen::Vector2i index_to_grid(const int idx);
  Eigen::Vector2i world_to_grid(const Eigen::Vector2d &pt_w);
  Eigen::Vector2d grid_to_world(const Eigen::Vector2i &pt_g);
  
  
private:
  GridMeta meta_;
  Eigen::Vector2i center_grid_;
  double inv_resolution_;
  double inv_angle_resolution_;
  int angle_width_;


}; // class GridMap

#endif // GRID_MAP_H