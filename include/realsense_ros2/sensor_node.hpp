/**
 * Copyright (C) 2020 Luz Martinez - luz.martinez@tum.de
 * Technische Universität München
 * Chair for Chair of Robotics, Artificial Intelligence and Real-time Systems
 * Fakultät für Informatik / I16, Boltzmannstraße 3, 85748 Garching bei München, Germany
 * https://www6.in.tum.de
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef SRC_SENSOR_INTERFACE
#define SRC_SENSOR_INTERFACE

// ROS headers
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/rate.hpp"

#include <librealsense/rs.hpp>
#include <realsense_ros2/base_realsense_node.hpp>

#include <chrono>
#include <sstream>
#include <vector>


const int DEFAULT_SENSOR_FREQUENCY = 1000;  // Hz

namespace realsense {

class SensorInterface {

public:
	SensorInterface(rclcpp::Node::SharedPtr nh);
	void setFrequency(double frequency) ;

	void run();
  
private:

	rclcpp::Node::SharedPtr _nh;
	realsense_camera::BaseNodelet base_sensor;
	rs::device * dev;
	std::vector<rs::stream> supported_streams;

	/* Parameters */
	std::string sensor_name_{""};
	double sensor_frequency_;
	int rows ;
	int cols ;
	int tile_w ;
	int tile_h ;

};


}  // namespace realsense


#endif //SRC_SENSOR_INTERFACE
