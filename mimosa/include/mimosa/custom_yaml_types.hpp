// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <yaml-cpp/yaml.h>

namespace YAML
{
template <>
struct convert<gtsam::Pose3>
{
  // The Pose3 is shown as x, y, z, x, y, z, w

  static Node encode(const gtsam::Pose3 & rhs)
  {
    Node node;
    node.push_back(rhs.translation().x());
    node.push_back(rhs.translation().y());
    node.push_back(rhs.translation().z());
    node.push_back(rhs.rotation().toQuaternion().x());
    node.push_back(rhs.rotation().toQuaternion().y());
    node.push_back(rhs.rotation().toQuaternion().z());
    node.push_back(rhs.rotation().toQuaternion().w());
    return node;
  }

  static bool decode(const Node & node, gtsam::Pose3 & rhs)
  {
    if (!node.IsSequence() || node.size() != 7) {
      return false;
    }

    rhs = gtsam::Pose3(
      gtsam::Rot3(gtsam::Quaternion(
        node[6].as<double>(), node[3].as<double>(), node[4].as<double>(), node[5].as<double>())),
      gtsam::Point3(node[0].as<double>(), node[1].as<double>(), node[2].as<double>()));
    return true;
  }
};
}  // namespace YAML
