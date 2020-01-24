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


#include <object_estimation/object_tracker.h>

#define VEL_NOISE_VAR 0.2 
#define COLOR_DETECTOR_PD 0.9
#define MIN_COLOR_DISTANCE 0.15

using namespace std;
using namespace mbzirc_comm_objs;

/** Constructor
\param id Identifier
\param type Type of object
*/
ObjectTracker::ObjectTracker(int id, int type)
{
	id_ = id;
	obj_type_ = type;
	obj_subtype_ = mbzirc_comm_objs::Object::SUBTYPE_UNKNOWN;

	is_static_ = true;
	fixed_pose_ = false;
		
	status_ = ACTIVE;

	fact_bel_.resize(1);
	fact_bel_[COLOR].resize(ObjectDetection::NCOLORS);

	pose_ = Eigen::MatrixXd::Zero(6,1);
	pose_cov_ = Eigen::MatrixXd::Identity(6,6);
	orientation_ = Eigen::MatrixXd::Zero(4,1);
	scale_.resize(3);
}

/** Constructor
\param id Identifier
\param type Type of object
\param subtype Subtype of object
*/
ObjectTracker::ObjectTracker(int id, int type, int subtype)
{
	id_ = id;
	obj_type_ = type;
	obj_subtype_ = subtype;

	is_static_ = true;
	fixed_pose_ = false;
		
	status_ = ACTIVE;

	fact_bel_.resize(1);
	fact_bel_[COLOR].resize(ObjectDetection::NCOLORS);

	pose_ = Eigen::MatrixXd::Zero(6,1);
	pose_cov_ = Eigen::MatrixXd::Identity(6,6);
	orientation_ = Eigen::MatrixXd::Zero(4,1);
	scale_.resize(3);
}

/// Destructor
ObjectTracker::~ObjectTracker()
{
}

/**
\brief Initialize the filter. 
\param node YAML node with initial information
*/
void ObjectTracker::initialize(YAML::Node node)
{
	if(node["sub_type"])
		obj_subtype_ = subtype_from_string(node["sub_type"].as<string>());

	if(node["status"] && node["status"].as<string>() == "active" )
	{
		status_ = ACTIVE;
	}
					
	if(node["frame_id"] && node["frame_id"].as<string>() == "arena" && node["position_x"] && node["orientation_x"])
	{
		fixed_pose_ = true;

		double x,y,z,qx,qy,qz,qw;
		x = node["position_x"].as<float>();
		y = node["position_y"].as<float>();
		z = node["position_z"].as<float>();
		qx = node["orientation_x"].as<float>();
		qy = node["orientation_y"].as<float>();
		qz = node["orientation_z"].as<float>();
		qw = node["orientation_w"].as<float>();

		// Setup state vector
		pose_.setZero(6, 1);
		pose_(0,0) = x;
		pose_(1,0) = y;
		pose_(2,0) = z;
		pose_(3,0) = 0.0;
		pose_(4,0) = 0.0;
		pose_(5,0) = 0.0;

		// Setup cov matrix
		pose_cov_.setIdentity(6, 6);
		for(int i = 0; i < 6; i++)
			for(int j = 0; j < 6; j++)
				pose_cov_(i,j) = 0.0;

		orientation_(0,0) = qx;
		orientation_(1,0) = qy;
		orientation_(2,0) = qz;
		orientation_(3,0) = qw;
	}
	else
	{
		cout << "No frame ID or postion/orientation available in config file. Frame ID should be arena" << endl;
	}

	int color;

	if(node["color"])
	{
		color = color_from_string(node["color"].as<string>());
		fixed_color_ = true;
	}
    	
	else
		color = ObjectDetection::COLOR_UNKNOWN;
		
	// Init and update factored belief 
	for(int fact = 0; fact < fact_bel_.size(); fact++)
	{
		switch(fact)
		{
			case COLOR:

			double prob_z, total_prob = 0.0;

			for(int i = 0; i < ObjectDetection::NCOLORS; i++)
			{
				if(color == ObjectDetection::COLOR_UNKNOWN)
					fact_bel_[COLOR][i] = 1.0/ObjectDetection::NCOLORS;
				else if(color == i)
					fact_bel_[COLOR][i] = 1.0;
				else
					fact_bel_[COLOR][i] = 0.0;
			}
					
			break;			
		}
	}

	if(node["scale_x"] && node["scale_y"] && node["scale_z"])
	{
		scale_.resize(3);
		scale_[0] = node["scale_x"].as<float>();
		scale_[1] = node["scale_y"].as<float>();
		scale_[2] = node["scale_z"].as<float>();
		fixed_scale_ = true;
	}

	// Update timer
	update_timer_.reset();
	update_count_ = 0;
}

/**
\brief Initialize the filter. 
\param z Initial observation
*/
void ObjectTracker::initialize(ObjectDetection* z)
{
	// Setup state vector
	pose_.setZero(6, 1);
	pose_(0,0) = z->pose.pose.position.x;
	pose_(1,0) = z->pose.pose.position.y;
	pose_(2,0) = z->pose.pose.position.z;
	pose_(3,0) = 0.0;
	pose_(4,0) = 0.0;
	pose_(5,0) = 0.0;
		
	// Setup cov matrix
	pose_cov_.setIdentity(6, 6);
	for(int i = 0; i < 3; i++)
		for(int j = 0; j < 3; j++)
			pose_cov_(i,j) = z->pose.covariance[i*6+j];
	
	pose_cov_(3,3) = VEL_NOISE_VAR;
	pose_cov_(4,4) = VEL_NOISE_VAR;
	pose_cov_(5,5) = VEL_NOISE_VAR;

	orientation_(0,0) = z->pose.pose.orientation.x;
	orientation_(1,0) = z->pose.pose.orientation.y;
	orientation_(2,0) = z->pose.pose.orientation.z;
	orientation_(3,0) = z->pose.pose.orientation.w;

	// Init and update factored belief 
	for(int fact = 0; fact < fact_bel_.size(); fact++)
	{
		switch(fact)
		{
			case COLOR:

			double prob_z, total_prob = 0.0;

			for(int i = 0; i < ObjectDetection::NCOLORS; i++)
			{
				if(z->color == ObjectDetection::COLOR_UNKNOWN)
					prob_z = 1.0;
				else if(z->color == i)
					prob_z = COLOR_DETECTOR_PD;
				else
					prob_z = (1.0 - COLOR_DETECTOR_PD)/(ObjectDetection::NCOLORS-1);

				fact_bel_[COLOR][i] = (1.0/(ObjectDetection::NCOLORS))*prob_z;
				total_prob += fact_bel_[COLOR][i];
			}
			
			// Normalize
			for(int i = 0; i < ObjectDetection::NCOLORS; i++)
			{
				fact_bel_[COLOR][i] /= total_prob;
			}	

			break;			
		}
	}

	// TODO scale

	// Update timer
	update_timer_.reset();
	update_count_ = 0;
}

/**
\brief Predict the filter.
\param dt Length in seconds of the prediction step. 
*/
void ObjectTracker::predict(double dt)
{
	// static factors do not vary. Position depending on whether it is dynamic or not.

	if(!is_static_)
	{
		// State vector prediction
		pose_(0,0) += pose_(3,0)*dt;
		pose_(1,0) += pose_(4,0)*dt;
		pose_(2,0) += pose_(5,0)*dt;
		
		// Convariance matrix prediction
		Eigen::Matrix<double, 6, 6> F;
		F.setIdentity(6, 6);
		F(0,3) = dt;
		F(1,4) = dt;
		F(2,5) = dt;
		Eigen::Matrix<double, 6, 6> Q;
		Q.setZero(6, 6);
		Q(3,3) = VEL_NOISE_VAR*dt*dt;
		Q(4,4) = VEL_NOISE_VAR*dt*dt;
		Q(5,5) = VEL_NOISE_VAR*dt*dt;
		pose_cov_ = F*pose_cov_*F.transpose() + Q;
	}
}

/**
\brief Update the filter.
\param z Observation to update. 
\return True if everything was fine
*/
bool ObjectTracker::update(ObjectDetection* z)
{
	// Update factored belief 
	for(int fact = 0; fact < fact_bel_.size(); fact++)
	{
		switch(fact)
		{
			case COLOR:

			double prob_z, total_prob = 0.0;

			for(int i = 0; i < ObjectDetection::NCOLORS; i++)
			{
				if(z->color == ObjectDetection::COLOR_UNKNOWN)
					prob_z = 1.0;
				if(z->color == i)
					prob_z = COLOR_DETECTOR_PD;
				else
					prob_z = (1.0 - COLOR_DETECTOR_PD)/(ObjectDetection::NCOLORS-1);

				fact_bel_[COLOR][i] *= prob_z;
				total_prob += fact_bel_[COLOR][i];
			}
			
			// Normalize
			for(int i = 0; i < ObjectDetection::NCOLORS; i++)
			{
				fact_bel_[COLOR][i] /= total_prob;
			}	

			break;			
		}
	}
	
	// Compute update jacobian
	Eigen::Matrix<double, 3, 6> H;
	H.setZero(3, 6);
	H(0,0) = 1.0;
	H(1,1) = 1.0;
	H(2,2) = 1.0;
		
	// Compute update noise matrix
	Eigen::Matrix<double, 3, 3> R;
	for(int i = 0; i < 3; i++)
		for(int j = 0; j < 3; j++)
			R(i,j) = z->pose.covariance[i*6+j];
		
	// Calculate innovation matrix
	Eigen::Matrix<double, 3, 3> S;
	S = H*pose_cov_*H.transpose() + R;
		
	// Calculate kalman gain
	Eigen::Matrix<double, 6, 3> K;
	K = pose_cov_*H.transpose()*S.inverse();
		
	// Calculate innovation vector
	Eigen::Matrix<double, 3, 1> y;
	y = H*pose_;
	y(0,0) = z->pose.pose.position.x - y(0,0);
	y(1,0) = z->pose.pose.position.y - y(1,0);
	y(2,0) = z->pose.pose.position.z - y(2,0);
		
	// Calculate new state vector
	pose_ = pose_ + K*y;
		
	// Calculate new cov matrix
	Eigen::Matrix<double, 6, 6> I;
	I.setIdentity(6, 6);
	pose_cov_ = (I - K*H)*pose_cov_;

	// TODO: Include orientation in the KF
	orientation_(0,0) = z->pose.pose.orientation.x;
	orientation_(1,0) = z->pose.pose.orientation.y;
	orientation_(2,0) = z->pose.pose.orientation.z;
	orientation_(3,0) = z->pose.pose.orientation.w;

	// Update timer
	update_timer_.reset();
	update_count_++;
}
    
/**
Compute the likelihood of an observation with current belief. Based on Mahalanobis distance. 
\param z Observation. 
\return Likelihood measurement
*/
double ObjectTracker::getLikelihood(ObjectDetection* z)
{
	double distance;

	// Compute update jacobian
	Eigen::Matrix<double, 3, 6> H;
	H.setZero(3, 6);
	H(0,0) = 1.0;
	H(1,1) = 1.0;
	H(2,2) = 1.0;
		
	// Compute update noise matrix
	Eigen::Matrix<double, 3, 3> R;
	for(int i = 0; i < 3; i++)
		for(int j = 0; j < 3; j++)
			R(i,j) = z->pose.covariance[i*6+j];
		
	// Calculate innovation matrix
	Eigen::Matrix<double, 3, 3> S;
	S = H*pose_cov_*H.transpose() + R;
			
	// Calculate innovation vector
	Eigen::Matrix<double, 3, 1> y;
	y = H*pose_;
	y(0,0) = z->pose.pose.position.x - y(0,0);
	y(1,0) = z->pose.pose.position.y - y(1,0);
	y(2,0) = z->pose.pose.position.z - y(2,0);

	distance = y.transpose()*S.inverse()*y;
	
	double prob_z, prob_color = 0.0;

	for(int i = 0; i < ObjectDetection::NCOLORS; i++)
	{
		if(z->color == i)
			prob_z = COLOR_DETECTOR_PD;
		else
			prob_z = (1.0 - COLOR_DETECTOR_PD)/(ObjectDetection::NCOLORS-1);

		prob_color += fact_bel_[COLOR][i]*prob_z;
	}

	if(prob_color < MIN_COLOR_DISTANCE)
		distance = -1;

	return distance;
}

/**
Compute the euclidean distance of an observation with current belief.
\param z Observation. 
\return Euclidean distance
*/
double ObjectTracker::getDistance(ObjectDetection* z)
{
	double dx, dy, dz;
	dx = pose_(0,0) - z->pose.pose.position.x;
	dy = pose_(1,0) - z->pose.pose.position.y;
	dz = pose_(2,0) - z->pose.pose.position.z;

	return sqrt(dx*dx + dy*dy + dz*dz);
}

/**
Return the time since the last observation update. 
\return Update time
*/
double ObjectTracker::lastUpdateTime()
{
	return update_timer_.elapsed();
}

/**
Return the counter of updates. 
\return Update counter
*/
int ObjectTracker::getUpdateCount()
{
	return update_count_;
}
    
/** \brief Return pose information from the target
\return Position of the target
*/
vector<double> ObjectTracker::getPose()
{
	vector<double> p;
	p.resize(3);
	p[0] = pose_(0,0);
	p[1] = pose_(1,0);
	p[2] = pose_(2,0);
}

/** \brief Return scale information from the target
\return Scale 
*/
vector<double> ObjectTracker::getScale()
{
	vector<double> s;
	s.resize(3);
	s[0] = scale_[0];
	s[1] = scale_[1];
	s[2] = scale_[2];
}

/** \brief Return velocity information from the target
\return Velocity of the target
*/
vector<double> ObjectTracker::getVelocity()
{
	vector<double> v;
	v.resize(3);
	v[0] = pose_(3,0);
	v[1] = pose_(4,0);
	v[2] = pose_(5,0);
}

/** \brief Return orientation information from the target
\return Quaterion 
*/
vector<double> ObjectTracker::getOrientation()
{
	vector<double> q;
	q.resize(4);
	q[0] = orientation_(0,0);
	q[1] = orientation_(1,0);
	q[2] = orientation_(2,0);
	q[3] = orientation_(3,0);
}

/** \brief Return covariance matrix from the target position
\return Covariance matrix
*/
vector<vector<double> > ObjectTracker::getCov()
{
	vector<vector<double> > covariance;
	covariance.resize(3);
	for(int i = 0; i < 3; i++)
		covariance[i].resize(3);
	
	for(int i = 0; i < 3; i++)
		for(int j = 0; j < 3; j++)
			covariance[i][j] = pose_cov_(i,j);
	
	return covariance;
}

/** \brief Return number of discrete factors
\return Number of discrete factors
*/
int ObjectTracker::getNumFactors()
{
	return fact_bel_.size();
}

/** \brief Return probabilities for a given factor
\param factor Factor to return
\return Probabilities for a given factor
*/
vector<double> ObjectTracker::getFactorProbs(int factor)
{
	return fact_bel_[factor];
}

/** \brief Return subtype of the target
\return Target subtype
*/
int ObjectTracker::getSubtype()
{
	return obj_subtype_;
}

/** \brief Return likeliest target status
\return Target status  
*/
ObjectStatus ObjectTracker::getStatus()
{
	return status_;
}

/** \brief Set a new target status
\param Target status  
*/
void ObjectTracker::setStatus(ObjectStatus status)
{
	status_ = status;
}

/** \brief Return target identifier
\return Target identifier  
*/
int ObjectTracker::getId()
{
	return id_;
}

/** \brief Return whether target is static or not
\return True if it is static
*/
bool ObjectTracker::isStatic()
{
	return is_static_;
}

/** \brief Return the likeliest color for the target
\return A color
*/
int ObjectTracker::getColor()
{
	double max_prob = -1.0;
	int color, color_max;

	for(int i = 0; i < ObjectDetection::NCOLORS; i++)
	{
		if(fact_bel_[COLOR][i] > max_prob)
		{
			max_prob = fact_bel_[COLOR][i];
			color_max = i;
		}
	}

	if(max_prob > 1.0 - max_prob)
		color = color_max;
	else
		color = ObjectDetection::COLOR_UNKNOWN;

	return color;
}

/** \brief Tranform string to color value
\param color String with color
*/
int ObjectTracker::color_from_string(const string& color) {
    int out_color;
    switch(color[0]) {
        case 'r':
            out_color = ObjectDetection::COLOR_RED;
            break;
        case 'g':
            out_color = ObjectDetection::COLOR_GREEN;
            break;
        case 'b':
            out_color = ObjectDetection::COLOR_BLUE;
            break;
        case 'o':
            out_color = ObjectDetection::COLOR_ORANGE;
            break;
        default:
        cout << "Unknown color " << color.c_str() << endl;
            out_color = ObjectDetection::COLOR_UNKNOWN;
    }
    return out_color;
}

/** \brief Tranform string to subtype value
\param subtype String with subtype
*/
int ObjectTracker::subtype_from_string(const string& subtype) {
    int out_subtype;
    if(subtype == "uav") 
	{
		out_subtype = Object::SUBTYPE_UAV;
	}
	else if(subtype == "ugv") 
	{
		out_subtype = Object::SUBTYPE_UGV;
	}
	else if(subtype == "infire") 
	{
		out_subtype = Object::SUBTYPE_INFIRE;
	}
	else if(subtype == "facadefire") 
	{
		out_subtype = Object::SUBTYPE_FACADEFIRE;
	}
	else if(subtype == "outfire") 
	{
		out_subtype = Object::SUBTYPE_OUTFIRE;
	}
	else if(subtype == "ground") 
	{
		out_subtype = Object::SUBTYPE_GROUND;
	}
	else if(subtype == "first") 
	{
		out_subtype = Object::SUBTYPE_FIRST;
	}
	else if(subtype == "second") 
	{
		out_subtype = Object::SUBTYPE_SECOND;
	}
	else
	{
		cout << "Unknown subtype " << subtype.c_str() << endl;
        out_subtype = Object::SUBTYPE_UNKNOWN;
	}
	 
    return out_subtype;
}


