/**
 * Copyright (C) 2016 Salvatore Virga - salvo.virga@tum.de, Marco Esposito - marco.esposito@tum.de
 * Technische Universität München
 * Chair for Computer Aided Medical Procedures and Augmented Reality
 * Fakultät für Informatik / I16, Boltzmannstraße 3, 85748 Garching bei München, Germany
 * http://campar.in.tum.de
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

#include <pluginlib/class_list_macros.hpp>

#include "realsense_ros2/sensor_node.hpp"

namespace realsense {

SensorInterface::SensorInterface(rclcpp::Node::SharedPtr nh) :
  _nh (nh)
  {
    rs::context ctx;
    if (ctx.get_device_count() == 0) throw std::runtime_error("No device detected. Is it plugged in?");
    dev = ctx.get_device(0);

    configureSensor();
    setParams();

  }

bool SensorInterface::configureSensor (){

    for (int i = (int)rs::capabilities::depth; i <= (int)rs::capabilities::fish_eye; i++)
        if (dev->supports((rs::capabilities)i))
            supported_streams.push_back((rs::stream)i);

    // Configure all supported streams to run at 30 frames per second
    for (auto & stream : supported_streams)
        dev->enable_stream(stream, rs::preset::best_quality);

    // Compute field of view for each enabled stream
    for (auto & stream : supported_streams)
    {
        if (!dev->is_stream_enabled(stream)) continue;
        auto intrin = dev->get_stream_intrinsics(stream);
        std::cout << "Capturing " << stream << " at " << intrin.width << " x " << intrin.height;
        // std::cout << std::setprecision(1) << std::fixed << ", fov = " << intrin.hfov() << " x " << intrin.vfov() << ", distortion = " << intrin.model() << std::endl;
    }

    // Start our device
    dev->start();

    return true;
}

void SensorInterface::setParams (){
    // rows = tiles_map.at(supported_streams.size()).second;
    // cols = tiles_map.at(supported_streams.size()).first;
    tile_w = 640; // pixels
    tile_h = 480; // pixels

}


void SensorInterface::setFrequency(double frequency) {
  sensor_frequency_ = frequency;
}

void SensorInterface::run() {
  // period = p;
}



}  // namespace realsense
