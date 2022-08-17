#include "boxPlaneSegment.h"

namespace plane_segment{

bool BoxPlaneSegment::boxPlaneSegment(const std::string& filename, Eigen::Vector3d start_point, Eigen::Vector3d end_point) 
{
  // Read point clout
	auto pc = open3d::geometry::PointCloud();
	if (!open3d::io::ReadPointCloud(filename, pc)) {
		std::cout << "[ERROR] Read file failed." << std::endl;
		return false;
	}

  // Get low and up bounds of coordinate
  Eigen::Vector3d sp = start_point, ep = end_point;
  for (int i = 0; i < 3; i++) {
    if (start_point(i) > end_point(i)) {
      sp(i) = end_point(i);
      ep(i) = start_point(i);
    } else if (start_point(i) == end_point(i)) {
      std::cout << "[ERROR] start greater than end" << std::endl;
      return false;
    }
  }

  // Select points inside box
	std::vector<size_t> inside_box_indices;
  for (int i = 0; i < (int)pc.points_.size(); i++) {
    Eigen::Vector3d& p = pc.points_[i];

    for (int j = 0; j < 3; j++) {
      if (p(j) > sp(j) && p(j) < ep(j))
        inside_box_indices.push_back(i);
    }
  }
	std::shared_ptr<open3d::geometry::PointCloud> inside_box_cloud = pc.SelectByIndex(inside_box_indices);

	// Plane segment
	Eigen::Vector4d plane_model;
	std::vector<size_t> inliers;
	std::tie(plane_model, inliers) = inside_box_cloud->SegmentPlane(0.02, 3, 1000);

	// Print plane model: A,B,C,D
	std::cout << plane_model(0) << "," <<
	plane_model(1) << "," <<
	plane_model(2) << "," <<
	plane_model(3) << std::endl;

	// Get inlier cloud 
	std::shared_ptr<open3d::geometry::PointCloud> inlier_cloud = inside_box_cloud->SelectByIndex(inliers);
	std::shared_ptr<open3d::geometry::PointCloud> outlier_cloud = inside_box_cloud->SelectByIndex(inliers, true);
	const Eigen::Vector3d red_color(1.0, 0, 0);
	inlier_cloud->PaintUniformColor(red_color);

	// Get point which is near from plane
	std::vector<Eigen::Vector3d>& pts = inside_box_cloud->points_;
	std::vector<Eigen::Vector3d>& norms = inside_box_cloud->normals_;
	std::vector<size_t> plane_indices;
	for (int i = 0; i < (int)pts.size(); i++) {
		Eigen::Vector3d& p = pts[i];
		Eigen::Vector3d& n = norms[i];
		Eigen::Vector4d m = plane_model;

		Eigen::Vector4d expand_coordinate(p(0), p(1), p(2), 1);
		Eigen::Vector3d plane_normal(m(0), m(1), m(2));

		// Get the distance from point to plane
		double dist = expand_coordinate.dot(m); 
		double abs_dist = fabs(dist);

		if (abs_dist < 0.05 ) {
			plane_indices.push_back(i);
			p -= dist * plane_normal;
			n = plane_normal;
		}
	}

	// Get the plane
	std::shared_ptr<open3d::geometry::PointCloud> plane_cloud = inside_box_cloud->SelectByIndex(plane_indices);

	// Draw interested clouds
	auto ppcd = std::make_shared<open3d::geometry::PointCloud>(pc);
	if (!open3d::visualization::DrawGeometries({  plane_cloud }, "open3d", 640, 480, 50, 50, false)) {
		std::cout << "[ERROR] Draw failed." << std::endl;
		return false;
	}

	// Save plane cloud
	// if (!open3d::io::WritePointCloud("../../colord_cloud_plane.ply", *plane_cloud, {false, true, false})) {
	// 	std::cout << "[ERROR] Write file failed." << std::endl;
	// 	return false;
	// }	

  return true;
}


bool BoxPlaneSegment::planeSegment(const std::string& filename) 
{
	auto pc = open3d::geometry::PointCloud();
	if (!open3d::io::ReadPointCloud(filename, pc)) {
		std::cout << "[ERROR] Read file failed." << std::endl;
		return false;
	}
	// Plane segment
	Eigen::Vector4d plane_model;
	std::vector<size_t> inliers;
	std::tie(plane_model, inliers) = pc.SegmentPlane(0.02, 3, 1000);

	// Print plane model: A,B,C,D
	std::cout << plane_model(0) << "," <<
	plane_model(1) << "," <<
	plane_model(2) << "," <<
	plane_model(3) << std::endl;

	// Get inlier cloud 
	std::shared_ptr<open3d::geometry::PointCloud> inlier_cloud = pc.SelectByIndex(inliers);
	std::shared_ptr<open3d::geometry::PointCloud> outlier_cloud = pc.SelectByIndex(inliers, true);
	const Eigen::Vector3d red_color(1.0, 0, 0);
	inlier_cloud->PaintUniformColor(red_color);

	// Get point which is near from plane
	std::vector<Eigen::Vector3d>& pts = pc.points_;
	std::vector<Eigen::Vector3d>& norms = pc.normals_;
	std::vector<size_t> plane_indices;
	for (int i = 0; i < (int)pts.size(); i++) {
		Eigen::Vector3d& p = pts[i];
		Eigen::Vector3d& n = norms[i];
		Eigen::Vector4d m = plane_model;

		Eigen::Vector4d expand_coordinate(p(0), p(1), p(2), 1);
		Eigen::Vector3d plane_normal(m(0), m(1), m(2));

		// Get the distance from point to plane
		double dist = expand_coordinate.dot(m); 
		double abs_dist = fabs(dist);

		if (abs_dist < 0.05 ) {
			plane_indices.push_back(i);
			p -= dist * plane_normal;
			n = plane_normal;
		}
	}

	// Get the floor part and the rest of whole cloud
	std::shared_ptr<open3d::geometry::PointCloud> floor_cloud = pc.SelectByIndex(plane_indices);
	std::shared_ptr<open3d::geometry::PointCloud> non_floor_cloud = pc.SelectByIndex(plane_indices, true);
	const Eigen::Vector3d blue_color(0, 0, 1.0);
	floor_cloud->PaintUniformColor(blue_color);

	// Plane segment
	Eigen::Vector4d ceil_plane_model;
	std::vector<size_t> ceil_inliers;
	std::tie(ceil_plane_model, ceil_inliers) = non_floor_cloud->SegmentPlane(0.01, 3, 1000);
	std::shared_ptr<open3d::geometry::PointCloud> ceil_inlier_cloud = pc.SelectByIndex(ceil_inliers);
	ceil_inlier_cloud->PaintUniformColor(red_color);

	// Print plane model: A,B,C,D
	std::cout << ceil_plane_model(0) << "," <<
	ceil_plane_model(1) << "," <<
	ceil_plane_model(2) << "," <<
	ceil_plane_model(3) << std::endl;


	for (int i = 0; i < (int)pts.size(); i++) {
		Eigen::Vector3d& p = pts[i];
		Eigen::Vector3d& n = norms[i];
		Eigen::Vector4d m = ceil_plane_model;

		Eigen::Vector4d expand_coordinate(p(0), p(1), p(2), 1);
		Eigen::Vector3d plane_normal(m(0), m(1), m(2));

		// Get the distance from point to plane
		double dist = expand_coordinate.dot(m); 
		double abs_dist = fabs(dist);

		if (abs_dist < 0.08 ) {
			plane_indices.push_back(i);
			p -= dist * plane_normal;
			n = -plane_normal;
		}
	}
	std::shared_ptr<open3d::geometry::PointCloud> floor_ceil_cloud = pc.SelectByIndex(plane_indices);
	floor_ceil_cloud->PaintUniformColor(blue_color);

	// Draw interested clouds
	auto ppcd = std::make_shared<open3d::geometry::PointCloud>(pc);
	if (!open3d::visualization::DrawGeometries({  floor_ceil_cloud }, "open3d", 640, 480, 50, 50, true)) {
		std::cout << "[ERROR] Draw failed." << std::endl;
		return false;
	}

	// Save plane cloud
	// if (!open3d::io::WritePointCloud("../../colord_cloud_plane.ply", *floor_ceil_cloud, {false, true, false})) {
	// 	std::cout << "[ERROR] Write file failed." << std::endl;
	// 	return false;
	// }	
    return true;
}

}