/*! \file base.h
 *
 * \brief The basis of the ur_model
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
        enum Mode {INIT, JOINT, CARTESIAN};

    private:
    }




    
}
