/**
 * \file  calibration.cc
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 * Copyright (C) 2012, Austin Robot Technology,
 *                     The University of Texas at Austin
 *
 * License: Modified BSD License
 */

#include "velodyne_decoder/calibration.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <yaml-cpp/yaml.h>

namespace YAML {
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template <typename T> void operator>>(const YAML::Node &node, T &i) { i = node.as<T>(); }
} // namespace YAML

namespace velodyne_decoder {

constexpr auto NUM_LASERS                  = "num_lasers";
constexpr auto DISTANCE_RESOLUTION         = "distance_resolution";
constexpr auto LASERS                      = "lasers";
constexpr auto LASER_ID                    = "laser_id";
constexpr auto ROT_CORRECTION              = "rot_correction";
constexpr auto VERT_CORRECTION             = "vert_correction";
constexpr auto DIST_CORRECTION             = "dist_correction";
constexpr auto TWO_PT_CORRECTION_AVAILABLE = "two_pt_correction_available";
constexpr auto DIST_CORRECTION_X           = "dist_correction_x";
constexpr auto DIST_CORRECTION_Y           = "dist_correction_y";
constexpr auto VERT_OFFSET_CORRECTION      = "vert_offset_correction";
constexpr auto HORIZ_OFFSET_CORRECTION     = "horiz_offset_correction";
constexpr auto MAX_INTENSITY               = "max_intensity";
constexpr auto MIN_INTENSITY               = "min_intensity";
constexpr auto FOCAL_DISTANCE              = "focal_distance";
constexpr auto FOCAL_SLOPE                 = "focal_slope";

/** Read calibration for a single laser. */
void operator>>(const YAML::Node &node, std::pair<int, LaserCorrection> &correction) {
  node[LASER_ID] >> correction.first;
  node[ROT_CORRECTION] >> correction.second.rot_correction;
  node[VERT_CORRECTION] >> correction.second.vert_correction;
  node[DIST_CORRECTION] >> correction.second.dist_correction;
  if (node[TWO_PT_CORRECTION_AVAILABLE])
    node[TWO_PT_CORRECTION_AVAILABLE] >> correction.second.two_pt_correction_available;
  else
    correction.second.two_pt_correction_available = false;
  node[DIST_CORRECTION_X] >> correction.second.dist_correction_x;
  node[DIST_CORRECTION_Y] >> correction.second.dist_correction_y;
  node[VERT_OFFSET_CORRECTION] >> correction.second.vert_offset_correction;
  if (node[HORIZ_OFFSET_CORRECTION])
    node[HORIZ_OFFSET_CORRECTION] >> correction.second.horiz_offset_correction;
  else
    correction.second.horiz_offset_correction = 0;

  std::optional<YAML::Node> max_intensity_node;
  if (node[MAX_INTENSITY]) {
    *max_intensity_node = node[MAX_INTENSITY];
  }
  if (max_intensity_node) {
    float max_intensity_float;
    *max_intensity_node >> max_intensity_float;
    correction.second.max_intensity = std::floor(max_intensity_float);
  } else {
    correction.second.max_intensity = 255;
  }

  std::optional<YAML::Node> min_intensity_node;
  if (node[MIN_INTENSITY]) {
    *min_intensity_node = node[MIN_INTENSITY];
  }
  if (min_intensity_node) {
    float min_intensity_float;
    *min_intensity_node >> min_intensity_float;
    correction.second.min_intensity = std::floor(min_intensity_float);
  } else {
    correction.second.min_intensity = 0;
  }
  node[FOCAL_DISTANCE] >> correction.second.focal_distance;
  node[FOCAL_SLOPE] >> correction.second.focal_slope;

  // Calculate cached values
  correction.second.cos_rot_correction  = cosf(correction.second.rot_correction);
  correction.second.sin_rot_correction  = sinf(correction.second.rot_correction);
  correction.second.cos_vert_correction = cosf(correction.second.vert_correction);
  correction.second.sin_vert_correction = sinf(correction.second.vert_correction);

  correction.second.laser_ring = 0; // clear initially (set later)
}

/** Read entire calibration file. */
void operator>>(const YAML::Node &node, Calibration &calibration) {
  int num_lasers;
  node[NUM_LASERS] >> num_lasers;
  float distance_resolution_m;
  node[DISTANCE_RESOLUTION] >> distance_resolution_m;
  const YAML::Node &lasers = node[LASERS];
  calibration.laser_corrections.clear();
  calibration.num_lasers            = num_lasers;
  calibration.distance_resolution_m = distance_resolution_m;
  calibration.laser_corrections.resize(num_lasers);
  for (int i = 0; i < num_lasers; i++) {
    std::pair<int, LaserCorrection> correction;
    lasers[i] >> correction;
    const int index = correction.first;
    if (index >= calibration.laser_corrections.size()) {
      calibration.laser_corrections.resize(index + 1);
    }
    calibration.laser_corrections[index] = (correction.second);
    calibration.laser_corrections_map.insert(correction);
  }

  // For each laser ring, find the next-smallest vertical angle.
  //
  // This implementation is simple, but not efficient.  That is OK,
  // since it only runs while starting up.
  double next_angle = -std::numeric_limits<double>::infinity();
  for (int ring = 0; ring < num_lasers; ++ring) {

    // find minimum remaining vertical offset correction
    double min_seen = std::numeric_limits<double>::infinity();
    int next_index  = num_lasers;
    for (int j = 0; j < num_lasers; ++j) {

      double angle = calibration.laser_corrections[j].vert_correction;
      if (next_angle < angle && angle < min_seen) {
        min_seen   = angle;
        next_index = j;
      }
    }

    if (next_index < num_lasers) { // anything found in this ring?
      // store this ring number with its corresponding laser number
      calibration.laser_corrections[next_index].laser_ring = ring;
      next_angle                                           = min_seen;
    }
  }
}

YAML::Emitter &operator<<(YAML::Emitter &out, const std::pair<int, LaserCorrection> correction) {
  out << YAML::BeginMap;
  out << YAML::Key << LASER_ID << YAML::Value << correction.first;
  out << YAML::Key << ROT_CORRECTION << YAML::Value << correction.second.rot_correction;
  out << YAML::Key << VERT_CORRECTION << YAML::Value << correction.second.vert_correction;
  out << YAML::Key << DIST_CORRECTION << YAML::Value << correction.second.dist_correction;
  out << YAML::Key << TWO_PT_CORRECTION_AVAILABLE << YAML::Value
      << correction.second.two_pt_correction_available;
  out << YAML::Key << DIST_CORRECTION_X << YAML::Value << correction.second.dist_correction_x;
  out << YAML::Key << DIST_CORRECTION_Y << YAML::Value << correction.second.dist_correction_y;
  out << YAML::Key << VERT_OFFSET_CORRECTION << YAML::Value
      << correction.second.vert_offset_correction;
  out << YAML::Key << HORIZ_OFFSET_CORRECTION << YAML::Value
      << correction.second.horiz_offset_correction;
  out << YAML::Key << MAX_INTENSITY << YAML::Value << correction.second.max_intensity;
  out << YAML::Key << MIN_INTENSITY << YAML::Value << correction.second.min_intensity;
  out << YAML::Key << FOCAL_DISTANCE << YAML::Value << correction.second.focal_distance;
  out << YAML::Key << FOCAL_SLOPE << YAML::Value << correction.second.focal_slope;
  out << YAML::EndMap;
  return out;
}

YAML::Emitter &operator<<(YAML::Emitter &out, const Calibration &calibration) {
  out << YAML::BeginMap;
  out << YAML::Key << NUM_LASERS << YAML::Value << calibration.laser_corrections.size();
  out << YAML::Key << DISTANCE_RESOLUTION << YAML::Value << calibration.distance_resolution_m;
  out << YAML::Key << LASERS << YAML::Value << YAML::BeginSeq;
  for (const auto &laser_correction : calibration.laser_corrections_map) {
    out << laser_correction;
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;
  return out;
}

void Calibration::read(const std::string &calibration_file) {
  std::ifstream fin(calibration_file.c_str());
  if (!fin.is_open()) {
    initialized = false;
    return;
  }
  initialized = true;
  try {
    YAML::Node doc;
    fin.close();
    doc = YAML::LoadFile(calibration_file);
    doc >> *this;
  } catch (YAML::Exception &e) {
    std::cerr << "YAML Exception: " << e.what() << std::endl;
    initialized = false;
  }
  fin.close();
}

void Calibration::write(const std::string &calibration_file) {
  std::ofstream fout(calibration_file.c_str());
  YAML::Emitter out;
  out << *this;
  fout << out.c_str();
  fout.close();
}

} // namespace velodyne_decoder
