#pragma once

#include <map>
#include <string>
#include <vector>

namespace open_manipulator_controller
{

// Gripper joint positions (radians).
// Both gripper_left_joint and gripper_right_joint use the same positive value.
constexpr double GRIPPER_OPEN   =  0.01;   // ~fully open
constexpr double GRIPPER_CLOSED =  0.0;    // fingers touching

// Default trajectory duration when the client sends 0
constexpr double DEFAULT_DURATION = 2.0;

// Joint order: [joint1, joint2, joint3, joint4]  (radians)
//   joint1 – base rotation
//   joint2 – shoulder
//   joint3 – elbow
//   joint4 – wrist
inline const std::map<std::string, std::vector<double>> & predefined_positions()
{
  static const std::map<std::string, std::vector<double>> positions = {
    {"home",        { 0.00, -1.05,  0.35,  0.70}},
    {"zero",        { 0.00,  0.00,  0.00,  0.00}},
    {"safe",        { 0.00, -0.80,  0.50,  0.30}},
    {"pick_front",  { 0.00, -0.30,  0.80, -0.50}},
    {"pick_left",   { 1.57, -0.30,  0.80, -0.50}},
    {"pick_right",  {-1.57, -0.30,  0.80, -0.50}},
    {"place_front", { 0.00, -0.60,  0.40,  0.20}},
    {"inspect",     { 0.00, -0.50,  0.90, -0.40}},
    {"stow",        { 0.00, -1.50,  1.00,  0.50}},
  };
  return positions;
}

}  // namespace open_manipulator_controller
