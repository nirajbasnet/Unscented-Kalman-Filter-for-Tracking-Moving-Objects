#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "ukf.h"
#include "ground_truth_package.h"
#include "measurement_package.h"
#include "matplotlibcpp.h"
#include <cmath>

namespace plt = matplotlibcpp;

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

void check_arguments(int argc, char *argv[]) {
    string usage_instructions = "Usage instructions: ";
    usage_instructions += argv[0];
    usage_instructions += " path/to/input.txt output.txt";

    bool has_valid_args = false;

    // make sure the user has provided input and output files
    if (argc == 1) {
        cerr << usage_instructions << endl;
    } else if (argc == 2) {
        cerr << "Please include an output file.\n" << usage_instructions << endl;
    } else if (argc == 3) {
        has_valid_args = true;
    } else if (argc > 3) {
        cerr << "Too many arguments.\n" << usage_instructions << endl;
    }

    if (!has_valid_args) {
        exit(EXIT_FAILURE);
    }
}

void check_files(ifstream &in_file, string &in_name,
                 ofstream &out_file, string &out_name) {
    if (!in_file.is_open()) {
        cerr << "Cannot open input file: " << in_name << endl;
        exit(EXIT_FAILURE);
    }

    if (!out_file.is_open()) {
        cerr << "Cannot open output file: " << out_name << endl;
        exit(EXIT_FAILURE);
    }
}

int main(int argc, char *argv[]) {

    check_arguments(argc, argv);

    string in_file_name_ = argv[1];
    ifstream in_file_(in_file_name_.c_str(), ifstream::in);

    string out_file_name_ = argv[2];
    ofstream out_file_(out_file_name_.c_str(), ofstream::out);

    check_files(in_file_, in_file_name_, out_file_, out_file_name_);

    /**********************************************
     *  Set Measurements                          *
     **********************************************/

    vector<MeasurementPackage> measurement_pack_list;
    vector<GroundTruthPackage> gt_pack_list;

    string line;

    // prep the measurement packages (each line represents a measurement at a
    // timestamp)
    while (getline(in_file_, line)) {
        string sensor_type;
        MeasurementPackage meas_package;
        GroundTruthPackage gt_package;
        istringstream iss(line);
        long long timestamp;

        // reads first element from the current line
        iss >> sensor_type;

        if (sensor_type.compare("L") == 0) {
            // laser measurement

            // read measurements at this timestamp
            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = VectorXd(2);
            float px;
            float py;
            iss >> px;
            iss >> py;
            meas_package.raw_measurements_ << px, py;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
            measurement_pack_list.push_back(meas_package);
        } else if (sensor_type.compare("R") == 0) {
            // radar measurement

            // read measurements at this timestamp
            meas_package.sensor_type_ = MeasurementPackage::RADAR;
            meas_package.raw_measurements_ = VectorXd(3);
            float ro;
            float phi;
            float ro_dot;
            iss >> ro;
            iss >> phi;
            iss >> ro_dot;
            meas_package.raw_measurements_ << ro, phi, ro_dot;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
            measurement_pack_list.push_back(meas_package);
        }

        // read ground truth data to compare later
        float x_gt;
        float y_gt;
        float vx_gt;
        float vy_gt;
        iss >> x_gt;
        iss >> y_gt;
        iss >> vx_gt;
        iss >> vy_gt;
        gt_package.gt_values_ = VectorXd(4);
        gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
        gt_pack_list.push_back(gt_package);
    }

    // Create a UKF instance
    UKF ukf;

    // used to compute the RMSE later
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;
    vector<double> est_px_ekf;
    vector<double> est_py_ekf;
    vector<double> gt_px_ekf;
    vector<double> gt_py_ekf;
    vector<double> radar_px_ekf;
    vector<double> radar_py_ekf;
    vector<double> lidar_px_ekf;
    vector<double> lidar_py_ekf;

    // start filtering from the second frame (the speed is unknown in the first
    // frame)

    size_t number_of_measurements = measurement_pack_list.size();

    // column names for output file
    out_file_ << "time_stamp" << "\t";
    out_file_ << "px_state" << "\t";
    out_file_ << "py_state" << "\t";
    out_file_ << "v_state" << "\t";
    out_file_ << "yaw_angle_state" << "\t";
    out_file_ << "yaw_rate_state" << "\t";
    out_file_ << "sensor_type" << "\t";
    out_file_ << "NIS" << "\t";
    out_file_ << "px_measured" << "\t";
    out_file_ << "py_measured" << "\t";
    out_file_ << "px_ground_truth" << "\t";
    out_file_ << "py_ground_truth" << "\t";
    out_file_ << "vx_ground_truth" << "\t";
    out_file_ << "vy_ground_truth" << "\n";


    for (size_t k = 0; k < number_of_measurements; ++k) {
        // Call the UKF-based fusion

        ukf.ProcessMeasurement(measurement_pack_list[k]);

        // timestamp
        out_file_ << measurement_pack_list[k].timestamp_ << "\t"; // pos1 - est

        // output the state vector
        out_file_ << ukf.x_(0) << "\t"; // pos1 - est
        out_file_ << ukf.x_(1) << "\t"; // pos2 - est
        out_file_ << ukf.x_(2) << "\t"; // vel_abs -est
        out_file_ << ukf.x_(3) << "\t"; // yaw_angle -est
        out_file_ << ukf.x_(4) << "\t"; // yaw_rate -est

        // output lidar and radar specific data
        if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER) {
            // sensor type
            out_file_ << "lidar" << "\t";

            // NIS value
            out_file_ << ukf.NIS_laser_ << "\t";

            // output the lidar sensor measurement px and py
            out_file_ << measurement_pack_list[k].raw_measurements_(0) << "\t";
            out_file_ << measurement_pack_list[k].raw_measurements_(1) << "\t";
            lidar_px_ekf.push_back(measurement_pack_list[k].raw_measurements_(0));
            lidar_py_ekf.push_back(measurement_pack_list[k].raw_measurements_(1));

        } else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR) {
            // sensor type
            out_file_ << "radar" << "\t";

            // NIS value
            out_file_ << ukf.NIS_radar_ << "\t";

            // output radar measurement in cartesian coordinates
            float rho = measurement_pack_list[k].raw_measurements_(0);
            float phi = measurement_pack_list[k].raw_measurements_(1);
            out_file_ << rho * cos(phi) << "\t"; // px measurement
            out_file_ << rho * sin(phi) << "\t"; // py measurement
            radar_px_ekf.push_back(rho * cos(phi));
            radar_py_ekf.push_back(rho * sin(phi));
        }

        // output the ground truth
        out_file_ << gt_pack_list[k].gt_values_(0) << "\t";
        out_file_ << gt_pack_list[k].gt_values_(1) << "\t";
        out_file_ << gt_pack_list[k].gt_values_(2) << "\t";
        out_file_ << gt_pack_list[k].gt_values_(3) << "\n";

        // convert ukf x vector to cartesian to compare to ground truth
        VectorXd ukf_x_cartesian_ = VectorXd(4);

        float x_estimate_ = ukf.x_(0);
        float y_estimate_ = ukf.x_(1);
        float vx_estimate_ = ukf.x_(2) * cos(ukf.x_(3));
        float vy_estimate_ = ukf.x_(2) * sin(ukf.x_(3));

        ukf_x_cartesian_ << x_estimate_, y_estimate_, vx_estimate_, vy_estimate_;

        estimations.push_back(ukf_x_cartesian_);
        ground_truth.push_back(gt_pack_list[k].gt_values_);

        est_px_ekf.push_back(x_estimate_);
        est_py_ekf.push_back(y_estimate_);
        gt_px_ekf.push_back(gt_pack_list[k].gt_values_(0));
        gt_py_ekf.push_back(gt_pack_list[k].gt_values_(1));

    }

    // compute the accuracy (RMSE)
    Tools tools;
    cout << "RMSE" << endl << tools.CalculateRMSE(estimations, ground_truth) << endl;

    // Set the size of output image to 1200x780 pixels
    plt::figure_size(1200, 780);
    // Plot line from given x and y data. Color is selected automatically.
    plt::named_plot("EKF estimate",est_px_ekf, est_py_ekf,"g");
    plt::named_plot("Ground Truth",gt_px_ekf, gt_py_ekf,"r");
    plt::named_plot("Radar data",radar_px_ekf, radar_py_ekf,"+");
    plt::named_plot("Lidar data",lidar_px_ekf, lidar_py_ekf,"*");

    plt::xlabel("x  (in meters)");
    plt::ylabel("y  (in meters)");
    plt::title("UKF tracking of moving object");
    // Enable legend.
    plt::legend();
    
    // Save the image (file format is determined by the extension)
    plt::save("../output/state_estimate.png");

    // close files
    if (out_file_.is_open()) {
        out_file_.close();
    }

    if (in_file_.is_open()) {
        in_file_.close();
    }

    cout << "Done!" << endl;
    return 0;
}
