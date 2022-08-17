#ifndef BOX_PLANE_SEGMENT_H_
#define BOX_PLANE_SEGMENT_H_

#include "open3d/Open3D.h"
#include <Eigen/Core>
#include <string>
#include <vector>
#include <tuple>
#include <memory>
#include <cmath>
#include <iostream>

namespace plane_segment{

class BoxPlaneSegment
{
private:
  /* data */
public:
  BoxPlaneSegment(/* args */){}
  ~BoxPlaneSegment(){}
  bool boxPlaneSegment(const std::string& filename, Eigen::Vector3d start_point, Eigen::Vector3d end_point);
  bool planeSegment(const std::string& filename); 
};

}

#endif 