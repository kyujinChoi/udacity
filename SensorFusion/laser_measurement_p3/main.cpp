#include <iostream>
#include <sstream>
#include <vector>
#include <eigen3/Eigen/Core>
#include "measurement_package.h"
#include "tracking.h"
#include "viewer.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::ifstream;
using std::istringstream;
using std::string;
using std::vector;


int main() {

  /**
   * Set Measurements
   */
  vector<MeasurementPackage> measurement_pack_list;

  // hardcoded input file with laser and radar measurements
  string in_file_name_ = "../obj_pose-laser-radar-synthetic-input.txt";
  ifstream in_file(in_file_name_.c_str(), ifstream::in);

  if (!in_file.is_open()) {
    cout << "Cannot open input file: " << in_file_name_ << endl;
  }

  string line;
  // set i to get only first 3 measurments
  // int i = 0;
  std::vector<Eigen::Vector2d> measure_pos;
  std::vector<Eigen::Vector2d> kalman_pos;
  while (getline(in_file, line)) {

    MeasurementPackage meas_package;

    istringstream iss(line);
    string sensor_type;
    iss >> sensor_type; // reads first element from the current line
    int64_t timestamp;
    if (sensor_type.compare("L") == 0) {  // laser measurement
      // read measurements
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      float x;
      float y;
      iss >> x;
      iss >> y;
      Eigen::Vector2d pos;
      pos << x,y;
      measure_pos.push_back(pos);
      meas_package.raw_measurements_ << x,y;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);

    } else if (sensor_type.compare("R") == 0) {
      // Skip Radar measurements
      continue;
    }
    // ++i;
  }

  // Create a Tracking instance
  Tracking tracking;

  // call the ProcessingMeasurement() function for each measurement
  size_t N = measurement_pack_list.size();
  // start filtering from the second frame 
  // (the speed is unknown in the first frame)
  
  for (size_t k = 0; k < N; ++k) {
    kalman_pos.push_back(tracking.ProcessMeasurement(measurement_pack_list[k]));
  }

  if (in_file.is_open()) {
    in_file.close();
  }
  viewKalmanResult(measure_pos,kalman_pos);
  return 0;
}