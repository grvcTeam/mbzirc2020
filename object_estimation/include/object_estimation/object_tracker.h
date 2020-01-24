//------------------------------------------------------------------------------
// GRVC MBZIRC
// Author Jesus Capitan <jcapitan@us.es>
//------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2019 GRVC University of Seville
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//------------------------------------------------------------------------------

#ifndef OBJECT_TRACKER_H_
#define OBJECT_TRACKER_H_

#include <mbzirc_comm_objs/ObjectDetection.h>
#include <mbzirc_comm_objs/Object.h>
#include <object_estimation/timer.hpp>

#include <vector>
#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>

enum ObjectStatus {INACTIVE, ACTIVE, LOST, N_STATUS};
enum Factor {COLOR};
    
/** \brief This class implements a stochastic filter for an object. 

This class implements a stochastic filter for an object. The filter estimates some continuous features 
(e.g., the position and velocity) and some discrete features (e.g., shape or color). The object may be static or 
moving. 

*/

class ObjectTracker 
{
public:
	ObjectTracker(int id, int type);
	ObjectTracker(int id, int type, int subtype);
	~ObjectTracker();

	void initialize(mbzirc_comm_objs::ObjectDetection* z);
	void initialize(YAML::Node node);
	void predict(double dt);
	bool update(mbzirc_comm_objs::ObjectDetection* z);
	double getLikelihood(mbzirc_comm_objs::ObjectDetection* z);
	double getDistance(mbzirc_comm_objs::ObjectDetection* z);
	double lastUpdateTime();
	int getUpdateCount();
	std::vector<double> getPose();
	std::vector<double> getScale();
	std::vector<double> getVelocity();
	double getOrientation();
	std::vector<std::vector<double> > getCov();
	int getNumFactors();
	std::vector<double> getFactorProbs(int factor);

	int getSubtype();
	ObjectStatus getStatus();
	void setStatus(ObjectStatus status);
	int getId();
	int getColor();
	bool isStatic();
	int color_from_string(const std::string& color);
	int subtype_from_string(const std::string& subtype);
	
protected:
	Timer update_timer_;			/// Timer for last update
	int update_count_;				/// Counter with the number of updates
	int id_;						/// Target identifier
	int obj_type_;					/// Object type
	int obj_subtype_;				/// Object subtype
	bool is_static_;				/// It indicates whether the target is static/dynamic
	ObjectStatus status_;			/// Current status

	/// Factored discrete belief	
	std::vector<std::vector<double> > fact_bel_;	
	
	/// State vector: [x (m), y (m), z(m), vx (m/s), vy (m/s), vz (m/s)]
	/// Orientation as yaw
	Eigen::MatrixXd pose_;
	Eigen::MatrixXd pose_cov_;
	double yaw_;
	std::vector<double> scale_;

	/// Do not update when True
	bool fixed_pose_;
	bool fixed_color_;
	bool fixed_scale_;				

};

#endif
