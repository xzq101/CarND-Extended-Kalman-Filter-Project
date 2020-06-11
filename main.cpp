#include "pch.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Dense>
#include "measurement_package.h"
#include "FusionEKF.h"


using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::ifstream;
using std::istringstream;
using std::string;
using std::vector;

int main()
{
	vector<MeasurementPackage> measurement_pack_list;
	string in_file_name_ = "obj_pose-laser-radar-synthetic-input.txt";
	ifstream in_file(in_file_name_.c_str(), ifstream::in);

	Tools tools;
	vector<VectorXd> estimations;
	vector<VectorXd> ground_truth;

	FusionEKF fusionEKF;

	if (!in_file.is_open()) {
		cout << "Cannot open input file: " << in_file_name_ << endl;
	}
	string line;
	int i = 0;
	string sensor_type;
	int64_t timestamp;
	double x;
	double y;
	MeasurementPackage meas_package;
	while (getline(in_file, line)  ) {
//		cout << line << endl;

		istringstream iss(line);

		iss >> sensor_type; // reads first element from the current line

		if (sensor_type.compare("L") == 0) {  // laser measurement
			meas_package.sensor_type_ = MeasurementPackage::LASER;
			meas_package.raw_measurements_ = VectorXd(2);
			iss >> x;
			iss >> y;
			meas_package.raw_measurements_ << x, y;
			iss >> timestamp;
			meas_package.timestamp_ = timestamp;
			measurement_pack_list.push_back(meas_package);
		}
		else if (sensor_type.compare("R") == 0) {
			// Skip Radar measurements
			meas_package.sensor_type_ = MeasurementPackage::RADAR;
			meas_package.raw_measurements_ = VectorXd(3);
			double ro;
			double theta;
			double ro_dot;
			iss >> ro;
			iss >> theta;
			iss >> ro_dot;
			meas_package.raw_measurements_ << ro, theta ,ro_dot;
			iss >> timestamp;
			meas_package.timestamp_ = timestamp;
			measurement_pack_list.push_back(meas_package);
			//continue;
		}
		double x_gt;
		double y_gt;
		double vx_gt;
		double vy_gt;
		iss >> x_gt;
		iss >> y_gt;
		iss >> vx_gt;
		iss >> vy_gt;

		VectorXd gt_values(4);
		gt_values(0) = x_gt;
		gt_values(1) = y_gt;
		gt_values(2) = vx_gt;
		gt_values(3) = vy_gt;
		ground_truth.push_back(gt_values);
//		cout << sensor_type << meas_package.raw_measurements_ << endl;
		++i;
	}
	size_t N = measurement_pack_list.size();
	cout << "N = measurement_pack_list: " << N<<endl;
	for (size_t k = 0; k < N; ++k) {
		if (k == 272) {
			cout << "N = measurement_pack_list=272: " << N << endl;
		}
		fusionEKF.ProcessMeasurement(measurement_pack_list[k]);

		VectorXd estimate(4);

		double p_x = fusionEKF.ekf_.x_(0);
		double p_y = fusionEKF.ekf_.x_(1);
		double v1 = fusionEKF.ekf_.x_(2);
		double v2 = fusionEKF.ekf_.x_(3);

		estimate(0) = p_x;
		estimate(1) = p_y;
		estimate(2) = v1;
		estimate(3) = v2;
		estimations.push_back(estimate);

		VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);
		std::cout << "RMSE:"<< RMSE.transpose()<< std::endl;
	}
 
	std::cout << "Hello World!\n";
}