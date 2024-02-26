/*! \file base.h
 *
 * \brief The basis of the ur_6dof
 *
 * \author Yanbing LIU
 *
 * \date 17/02/2024
 *
 * \copyright 
 *
 * License details.
 *
 */

#ifndef BASE_H
#define BASE_H

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <boost/shared_ptr.hpp>

namespace tum_ics_ur_robot_base
{

    class RobotBase
    {
    public:
        enum Mode {CREATE, INITIALIZED};
    
    private:
        Mode mode_;
        std::string name_;

    public:
        RobotBase(const std::string& name): 
            name_(name),
            mode_(CREATE) 
        {}

        virtual ~RobotBase() {} 

        // is model initialized?
        // if not, try to initialized
        bool is_Initialized(ros::NodeHandle& nh) 
        {
            if(mode_ == INITIALIZED)
                return true;

            if(init(nh)) {
                mode_ = INITIALIZED;
                return true;
            }
            return false;
        }

        // is model initialized?
        // only return state, not try
        bool get_mode() const {
            return mode_ == INITIALIZED;
        }

        std::string get_name() const 
        { 
            return name_; 
        }

    private:
        virtual bool init(ros::NodeHandle& nh) = 0;
    };





}

#endif