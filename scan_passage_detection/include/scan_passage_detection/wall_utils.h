#ifndef WALL_UTILS_H
#define WALL_UTILS_H

#include <visualization_msgs/MarkerArray.h>
#include <mbzirc_comm_objs/WallList.h>

inline double squaredDistanceToSegment(double x, double y, double x1, double y1, double x2, double y2) {
	double dx = x2 - x1;
	double dy = y2 - y1;

  double t_star;
	double t_hat = (dx * (x - x1) + dy * (y - y1)) / (dx*dx + dy*dy);
	if (t_hat < 0.0) {
		t_star = 0;
	} else if (t_hat > 1.0) {
		t_star = 1;
	} else {
		t_star = t_hat;
	}

	double closest_x = x1 + t_star * dx - x;
	double closest_y = y1 + t_star * dy - y;
	return closest_x*closest_x + closest_y*closest_y;
}

inline double squaredLength(const mbzirc_comm_objs::Wall& _wall) {
    double dx = _wall.end[0] - _wall.start[0];
    double dy = _wall.end[1] - _wall.start[1];
    return dx*dx + dy*dy;
}

inline mbzirc_comm_objs::Wall closestWall(const mbzirc_comm_objs::WallList& _wall_list) {
  mbzirc_comm_objs::Wall closest;
  double min_sq_distance = 1e6;
  for (auto wall: _wall_list.walls) {
      double sq_distance = squaredDistanceToSegment(0, 0, wall.start[0], wall.start[1], wall.end[0], wall.end[1]);
      if (sq_distance < min_sq_distance) {
          min_sq_distance = sq_distance;
          closest = wall;
      }
  }
  return closest;
}

inline mbzirc_comm_objs::Wall mostCenteredWall(const mbzirc_comm_objs::WallList& _wall_list) {
  mbzirc_comm_objs::Wall most_centered;
  double min_sq_dx = 1e6;
  for (auto wall: _wall_list.walls) {
      double dx = 0.5 * (wall.start[0] + wall.end[0]);  // x-distance to mid-point
      if (dx*dx < min_sq_dx) {
          min_sq_dx = dx*dx;
          most_centered = wall;
      }
  }
  return most_centered;
}

inline mbzirc_comm_objs::Wall largestWall(const mbzirc_comm_objs::WallList& _wall_list) {
  mbzirc_comm_objs::Wall largest;
  double max_squared_length = 0;
  for (auto wall: _wall_list.walls) {
      double squared_length = squaredLength(wall);
      if (squared_length > max_squared_length) {
          max_squared_length = squared_length;
          largest = wall;
      }
  }
  return largest;
}

inline visualization_msgs::Marker getLineMarker(const mbzirc_comm_objs::Wall& _wall, const std::string& _frame_id, std_msgs::ColorRGBA _color, unsigned int _id = 0, const std::string& _ns = "wall") {
    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id = _frame_id;
    line_marker.header.stamp = ros::Time::now();
    line_marker.ns = _ns;
    line_marker.id = _id;
    line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::Marker::ADD;
    line_marker.pose.position.x = 0;
    line_marker.pose.position.y = 0;
    line_marker.pose.position.z = 0;
    line_marker.pose.orientation.x = 0.0;
    line_marker.pose.orientation.y = 0.0;
    line_marker.pose.orientation.z = 0.0;
    line_marker.pose.orientation.w = 1.0;
    line_marker.scale.x = 0.1;
    line_marker.scale.y = 0.1;
    //line_marker.scale.z = 0.1;
    geometry_msgs::Point p;
    p.x = _wall.start[0];
    p.y = _wall.start[1];
    line_marker.points.push_back(p);
    p.x = _wall.end[0];
    p.y = _wall.end[1];
    line_marker.points.push_back(p);
    line_marker.color = _color;
    line_marker.lifetime = ros::Duration(0.1);

    return line_marker;
}

inline mbzirc_comm_objs::Wall fromPoints(const geometry_msgs::Point& _start, const geometry_msgs::Point& _end) {
    mbzirc_comm_objs::Wall wall;
    wall.start[0] = _start.x;
    wall.start[1] = _start.y;
    wall.end[0] = _end.x;
    wall.end[1] = _end.y;
    return wall;
}

#endif  // WALL_UTILS_H
