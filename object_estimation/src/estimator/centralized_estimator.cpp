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


#include <object_estimation/centralized_estimator.h>
#include <yaml-cpp/yaml.h>

// #define DEBUG_MODE

using namespace std;

/** Constructor
\param type Object type
\param likelihood_th Likelihood threshold to associate observations
\param lost_th Time threshold to consider target lost
\param min_update_count Minimum number of updates to consider a target consistent 
*/
CentralizedEstimator::CentralizedEstimator(int type, double lkhd_th, double lost_th, int min_update_count)
{
	obj_type_ = type;
	likelihood_th_ = lkhd_th;
	lost_th_ = lost_th;
	min_update_count_ = min_update_count;
	track_id_count_ = 0;
}

/// Destructor
CentralizedEstimator::~CentralizedEstimator()
{
	for(auto it = targets_.begin(); it != targets_.end(); ++it)
	{
		delete(it->second);
	}
}

/** \brief Initialize estimator with a priori information
\param config_file File with a priori information
*/
void CentralizedEstimator::initializeAPrioriInfo(string config_file)
{
	YAML::Node yaml_config = YAML::LoadFile(config_file);
	string item;

	switch(obj_type_)
	{
		case mbzirc_comm_objs::Object::TYPE_PILE:
			item = "pile";
		break;
		case mbzirc_comm_objs::Object::TYPE_WALL:
			item = "wall";
		break;
		case mbzirc_comm_objs::Object::TYPE_PASSAGE:
			item = "passage";
		break;
		case mbzirc_comm_objs::Object::TYPE_FIRE:
			item = "fire";
		break;
		default:
		cout << "There is no a priori information for objects of this type" << endl;
	}

	if(yaml_config[item])
	{
		for (size_t i = 0; i < yaml_config[item].size(); i++) 
		{
			int new_target_id = track_id_count_++;
			targets_[new_target_id] = new ObjectTracker(new_target_id, obj_type_);
			targets_[new_target_id]->setStatus(INACTIVE);
			targets_[new_target_id]->initialize(yaml_config[item][i]);
    	}
	}

	if(yaml_config["building"])
		scenario_info_ = yaml_config["building"];
}

/** Prediction step for all targets
\param dt Length in seconds of the prediction step.
*/
void CentralizedEstimator::predict(double dt)
{
	for(auto it = targets_.begin(); it != targets_.end(); ++it)
	{
		if((it->second)->getStatus() == ACTIVE)
			(it->second)->predict(dt);
	}
}

/**
\brief Update step for a target
\param z_list List with observations to update
\return True if everything was fine
*/
bool CentralizedEstimator::update(vector<mbzirc_comm_objs::ObjectDetection*> z_list)
{
	vector<vector<double> > distances;
	vector<int> valid_targets;
	vector<int> valid_candidates;
	int n_valid_targets = 0, n_valid_candidates = 0;

	// Compute distances for each association. And count valid targets
	for(auto it = targets_.begin(); it != targets_.end(); ++it)
	{
		bool target_valid = true;

		// Try to associate detections with active and inactive targets
		if( (it->second)->getStatus() != LOST )
		{
			valid_targets.push_back((it->second)->getId());
			n_valid_targets++;
		}
		else
		{
			valid_targets.push_back(-1);
			target_valid = false;
		}

		vector<double> t_distances;
		double likelihood;

		for(int i = 0; i < z_list.size(); i++)
		{
			if(target_valid)
			{		
				likelihood = (it->second)->getLikelihood(z_list[i]);
				t_distances.push_back(likelihood);
			}
			else
				t_distances.push_back(-1.0);
		}

		distances.push_back(t_distances);
	}

	// All candidates valid initially
	for(int i = 0; i < z_list.size(); i++)
		valid_candidates.push_back(1);

	n_valid_candidates = z_list.size();

	#ifdef DEBUG_MODE
	cout << "Candidates from UAV " << endl;
	cout << "Distances: " << endl;
	for(int i = 0; i < distances.size(); i++)
	{
		for(int j = 0; j < distances[i].size(); j++)
			cout << distances[i][j] << " ";
		cout << endl;
	}
	cout << endl;	
	
	#endif

	// Look for best pairs until running out of candidates or targets
	while(n_valid_targets != 0 && n_valid_candidates != 0 )
	{
		double min_dist = -1.0;
		pair<int, int> best_pair;

		for(int t_id = 0; t_id < distances.size(); t_id++)
		{
			if(valid_targets[t_id] != -1)
			{
				for(int c_id = 0; c_id < distances[t_id].size(); c_id++)
				{
					if(valid_candidates[c_id] != -1 && distances[t_id][c_id]!= -1 && (min_dist == -1.0 || distances[t_id][c_id] < min_dist))
					{
						min_dist = distances[t_id][c_id];
						best_pair.first = t_id;
						best_pair.second = c_id;
					}		
				}
			}
		}

		// If there is no good data association, create new target
		if(min_dist != -1 && min_dist <= likelihood_th_)
		{
			#ifdef DEBUG_MODE
			cout << "Candidate " << z_list[best_pair.second]->pose.pose.position.x << "," << z_list[best_pair.second]->pose.pose.position.y << ". Associated to target " << valid_targets[best_pair.first] << ", with distance " << min_dist << endl;
			#endif

			targets_[valid_targets[best_pair.first]]->update(z_list[best_pair.second]);
			targets_[valid_targets[best_pair.first]]->setStatus(ACTIVE);		// In case it was not active
		}
		else
		{
			#ifdef DEBUG_MODE
			cout << "Candidate " << z_list[best_pair.second]->pose.pose.position.x << "," << z_list[best_pair.second]->pose.pose.position.y << ". New target " << track_id_count_ << ", with distance " << min_dist << endl;
			#endif
			int new_target_id = track_id_count_++;
			targets_[new_target_id] = new ObjectTracker(new_target_id, obj_type_);
			targets_[new_target_id]->initialize(z_list[best_pair.second]);

			// Include new target's distances
			valid_targets.push_back(new_target_id);
			n_valid_targets++;

			vector<double> t_distances;
			double likelihood;

			for(int i = 0; i < z_list.size(); i++)
			{
				if(valid_candidates[i] != -1)
				{		
					likelihood = targets_[new_target_id]->getLikelihood(z_list[i]);
					t_distances.push_back(likelihood);
				}
				else
					t_distances.push_back(-1.0);
			}

			distances.push_back(t_distances);			
		}

		valid_candidates[best_pair.second] = -1;
		n_valid_candidates--;
	}

	// Create new targets with remaining candidates
	if(n_valid_candidates)
	{
		for(int i = 0; i < z_list.size(); i++)
		{
			if(valid_candidates[i])
			{
				#ifdef DEBUG_MODE
				cout << "Candidate " << z_list[i]->location(0) << "," << z_list[i]->location(1) << ". New target " << track_id_count_ << endl;
				#endif

				targets_[track_id_count_] = new ObjectTracker(track_id_count_, obj_type_);
				targets_[track_id_count_++]->initialize(z_list[i]);
			}
		}
	}
}

/// Return number of targets
int CentralizedEstimator::getNumTargets()
{
	return targets_.size();
}

/// Return Identifiers of active targets
vector<int> CentralizedEstimator::getActiveTargets()
{
	vector<int> targets_ids;

	for(auto it = targets_.begin(); it != targets_.end(); ++it)
	{
		if( (it->second)->getStatus() == ACTIVE)
		{
			targets_ids.push_back((it->second)->getId());
		}
	}

	return targets_ids;
}

/** \brief Return information from a target
\param target_id Identifier of the target
\param position Position of the target
\param scale Scale of the target
\param Status Status of the target 
\param color Color of the target
\param subtype Subtype of the target
\return True if the target was found 
*/
bool CentralizedEstimator::getTargetInfo(int target_id, vector<double> &position, vector<double> &scale, ObjectStatus &status, int &color, int &subtype)
{
	bool found = false;

	auto it = targets_.find(target_id);
	if(it != targets_.end())
	{
		found = true;
		position = targets_[target_id]->getPose();
		scale = targets_[target_id]->getScale();
		status = targets_[target_id]->getStatus();
		color = targets_[target_id]->getColor();
		subtype = targets_[target_id]->getSubtype();
	}
	
	return found;
}

/** \brief Return position information from a target
\param target_id Identifier of the target
\param position Position of the target
\param orientation Yaw orientation of the target
\param covariance Covariance matrix for position
\return True if the target was found 
*/
bool CentralizedEstimator::getTargetInfo(int target_id, vector<double> &position, double &orientation, vector<vector<double> > &covariances)
{
	bool found = false;

	auto it = targets_.find(target_id);
	if(it != targets_.end())
	{
		found = true;
		position = targets_[target_id]->getPose();
		orientation = targets_[target_id]->getOrientation();
		covariances = targets_[target_id]->getCov();
	}
	
	return found;
}

/** \brief Set the status of a target
\param target_id Identifier of the target
\param Status Status of the target 
\return True if the target was found 
*/
bool CentralizedEstimator::setTargetStatus(int target_id, ObjectStatus status)
{
	bool found = false;

	auto it = targets_.find(target_id);
	if(it != targets_.end())
	{
		found = true;
		targets_[target_id]->setStatus(status);
	}
	
	return found;
}

/** Compute subtypes for targets
*/
void CentralizedEstimator::computeSubtypesTargets()
{
	for(auto it = targets_.begin(); it != targets_.end(); ++it)
		(it->second)->computeSubtype(scenario_info_);
}

/** Remove lost targets
*/
void CentralizedEstimator::removeLostTargets()
{
	auto it = targets_.begin();

	while(it != targets_.end())
	{		
		if( (it->second)->getStatus() == ACTIVE && (it->second)->lastUpdateTime() > lost_th_ && (it->second)->getUpdateCount() < min_update_count_) 
		{
			(it->second)->setStatus(LOST);
		}
		if( (it->second)->getStatus() == LOST )
		{
			delete(it->second);
			it = targets_.erase(it);
		}
		else
			++it;
	}
}

/** Print information from the targets for debugging
*/
void CentralizedEstimator::printTargetsInfo()
{
	cout << "****************************************************" << endl;
	cout << "Number of targets: " << targets_.size() << endl;

	for(auto it = targets_.begin(); it != targets_.end(); ++it)
	{
		cout << "Id: " << (it->second)->getId() << ". ";

		switch((it->second)->getStatus())
		{
			case INACTIVE:
			cout << "Status: " << "INACTIVE. "; 
			break;
			case ACTIVE:
			cout << "Status: " << "ACTIVE. "; 
			break;
			case LOST:
			cout << "Status: " << "LOST. "; 
			break;
			default:
			cout << "Status: " << "ERROR. ";
		}

		if( (it->second)->getStatus() == ACTIVE )
		{
			vector<double> position, scale;
			vector<vector<double> > cov;
			vector<double> color_probs;

			position = (it->second)->getPose();
			scale = (it->second)->getScale();
			cov = (it->second)->getCov();
			color_probs = (it->second)->getFactorProbs(0);

			cout << "Position: " << position[0] << "," << position[1] << "," << position[2] << ". Scale: " << scale[0] << "," << scale[1] << "," << scale[2] << ". Covariances: " << cov[0][0] << " " << cov[0][1] << " " << cov[0][2] << "; " << cov[1][0] << " " << cov[1][1] << " " << cov[1][2] << "; " << cov[2][0] << " " << cov[2][1] << " " << cov[2][2] << "." << endl;
			cout << "Color: ";
			switch((it->second)->getColor())
			{
				case mbzirc_comm_objs::ObjectDetection::COLOR_UNKNOWN:
				cout << "UNKNOWN. ";
				break;
				case mbzirc_comm_objs::ObjectDetection::COLOR_RED:
				cout << "RED. ";
				break;
				case mbzirc_comm_objs::ObjectDetection::COLOR_BLUE:
				cout << "BLUE. ";
				break;
				case mbzirc_comm_objs::ObjectDetection::COLOR_GREEN:
				cout << "GREEN. ";
				break;
				case mbzirc_comm_objs::ObjectDetection::COLOR_ORANGE:
				cout << "ORANGE. ";
				break;
				default:
				cout << "ERROR. ";
			}

			cout << "( ";
			for(int i = 0; i < color_probs.size(); i++)
				cout << color_probs[i] << " ";

			cout << "). ";

			cout << "Static? ";
			if((it->second)->isStatic())
				cout << "yes. ";
			else
				cout << "no. ";
		}
		cout << endl;
	}
}