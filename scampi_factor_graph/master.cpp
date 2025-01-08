#include "src/main.cpp"
#include <chrono>

// Declare a time point variable to store the starting time
std::chrono::time_point<std::chrono::high_resolution_clock> tic_time;

// Function to start the timer
void tic() {
    tic_time = std::chrono::high_resolution_clock::now();
}

// Function to stop the timer and return the elapsed time in seconds
double toc() {
    auto toc_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = toc_time - tic_time;
    return elapsed.count();
}

int main(int argc, char *argv[])
{   
    // robot characteristic
    CableRobotParams robot_params(0.7100703113867337, 333.54);

    std::default_random_engine generator(std::random_device{}());
    std::normal_distribution<double> distribution(0.0, 3.0/sqrt(3.0));
    std::normal_distribution<double> distribution_orientaion(0.0, 0.02/sqrt(3.0));

    Eigen::Vector3d Pulley_a(-125.0, -110.0, 48.0);
    Eigen::Vector3d Pulley_b( 125.0, -110.0, 48.0);
    Eigen::Vector3d Pulley_c( 125.0,  110.0, 48.0);
    Eigen::Vector3d Pulley_d(-125.0,  110.0, 48.0);

    robot_params.setPulleyPoses(Pulley_a, Pulley_b, Pulley_c, Pulley_d);

    Eigen::Vector3d Ee_a(-0.21 , -0.21 , -0.011);  
    Eigen::Vector3d Ee_b(0.21  , -0.21 , -0.011);
    Eigen::Vector3d Ee_c(0.21  ,  0.21 , -0.011);
    Eigen::Vector3d Ee_d(-0.21 ,  0.21 , -0.011);
    robot_params.setEEAnchors(Ee_a, Ee_b, Ee_c, Ee_d);

    Eigen::Vector3d r_to_cog(0, 0, -0.12);
    robot_params.setCog(r_to_cog);

    std::vector<Eigen::Matrix<double, 3, 1>> p_platform_collection;
    std::vector<Eigen::Matrix<double, 3, 3>> rot_init_platform_collection;
    std::vector<Eigen::Matrix<double, 4, 1>> cable_length_collection;
    std::vector<gtsam::Vector2> calibration_result;

    std::cout << "******** Extracting Dataset ********" << std::endl;
    // Open the CSV file of real dataset and record them in data vector
    std::ifstream file_position("./dataset/pos_i_cpp_test.csv"); 
    std::vector<std::vector<double>> real_data_position;
    if (file_position) {
        std::string line;
        while (getline(file_position, line)) {
            std::stringstream ss(line);
            std::vector<double> row;
            std::string val;
            while (getline(ss, val, ',')) {
                row.push_back(stod(val));
            }
            real_data_position.push_back(row);
        }
    std::cout << "Number of position data: " << real_data_position.size() << std::endl;
    } else {
        std::cout << "Unable to open file." << std::endl;
    }
    double lenght_dataset_for_calibration = real_data_position.size();
    // Rewrite the data in it's object
    for (size_t i = 0; i < real_data_position.size(); i++)
    {
        p_platform_collection.push_back(Eigen::Vector3d(real_data_position[i][0], real_data_position[i][1], real_data_position[i][2]));
    }

    // ******************************
    // // This is used for euler orientation extraction 
    std::ifstream file_orientation("./dataset/R_i_cpp_test.csv");
    std::vector<std::vector<double>> real_orientation;
    if (file_orientation) {
        std::string line;
        while (getline(file_orientation, line)) {
            std::stringstream ss(line);
            std::vector<double> row;
            std::string val;
            while (getline(ss, val, ',')) {
                row.push_back(stod(val));
            }
            real_orientation.push_back(row);
        }
    std::cout << "Number of orientation data: " << real_orientation.size() << std::endl;
    } else {
        std::cout << "Unable to open file." << std::endl;
    }

    std::ifstream file_lcat("./dataset/lc_meas_cpp_test.csv");
    std::vector<std::vector<double>> real_data_lcat;
    if (file_lcat) {
        std::string line;
        while (getline(file_lcat, line)) {
            std::stringstream ss(line);
            std::vector<double> row;
            std::string val;
            while (getline(ss, val, ',')) {
                row.push_back(stod(val));
            }
            real_data_lcat.push_back(row);
        }
    std::cout << "Number of encoder data: " << real_data_lcat.size() << std::endl;
    } else {
        std::cout << "Unable to open file." << std::endl;
    }
    // Rewrite the data in it's object
    for (size_t i = 0; i < real_data_lcat.size(); i++)
    {   
        cable_length_collection.push_back(Eigen::Vector4d(real_data_lcat[i][0], real_data_lcat[i][1],
                                                        real_data_lcat[i][2], real_data_lcat[i][3]));
    }
    
    for (size_t i = 0; i < cable_length_collection.size(); i++) // cable_length_collection.size()
    {
        Eigen::Vector3d p_platform(p_platform_collection[i][0] + distribution(generator),p_platform_collection[i][1] + distribution(generator),p_platform_collection[i][2] + distribution(generator));

        gtsam::Rot3 rot_init_;
        double pitch = real_orientation[i][0] * M_PI/180.0 + distribution_orientaion(generator);
        double roll = real_orientation[i][1] * M_PI/180.0 + distribution_orientaion(generator);
        double yaw = real_orientation[i][2] * M_PI/180.0 + distribution_orientaion(generator);
        rot_init_ = rot_init_.Ypr(yaw, pitch, roll);
        Eigen::Matrix3d rot_init = gtsamRot3ToEigenMatrix(rot_init_);

        Eigen::VectorXd lc_cat = gtsam::Vector4{cable_length_collection[i][0], cable_length_collection[i][1], cable_length_collection[i][2], cable_length_collection[i][3]}; // IKresults[1]

        // Start the timer
        tic();
        // start inverse optimization
        std::vector<MatrixXd> IKresults = IK_Factor_Graph_Optimization(robot_params, rot_init, p_platform);
        // the result of inverse optimization
        // std::cout << std::endl << "-------------------inverse result--------------------------" << std::endl;
        // std::cout << std::endl << "rot_platform: " << std::endl << IKresults[0] << std::endl;
        // std::cout << std::endl << "l_cat: " << std::endl << IKresults[1] << std::endl;
        // std::cout << std::endl << "cable_forces: " << std::endl << IKresults[2] << std::endl;
        // std::cout << std::endl << "c1: " << std::endl << IKresults[3] << std::endl;
        // std::cout << std::endl << "c2: " << std::endl << IKresults[4] << std::endl;
        // std::cout << std::endl << "b_in_w: " << std::endl << IKresults[5] << std::endl;
        // std::cout << std::endl << "l_cat_error: " << std::endl << (IKresults[1] - gtsam::Vector4{cable_length_collection[i][0], cable_length_collection[i][1], cable_length_collection[i][2], cable_length_collection[i][3]}).norm() << std::endl;

        // start forward optimization
        Eigen::Vector2d fc_1 = IKresults[2].col(0); 
        Eigen::Vector3d pos_init = p_platform;
        Eigen::Matrix3d rtation_init = rot_init;
        std::vector<MatrixXd> FKresults = FK_Factor_Graph_Optimization(robot_params, lc_cat, fc_1, pos_init, rtation_init);
        // the result of forward optimization
        // Stop the timer and print the elapsed time
        double elapsed = toc();
        std::cout << "Elapsed time: " << elapsed << " seconds" << std::endl;
        
        // std::cout << std::endl << "-------------------forward result--------------------------" << std::endl;
        // std::cout << std::endl << "rot_platform: " << std::endl << FKresults[0] << std::endl;
        // std::cout << std::endl << "p_platform: " << std::endl << FKresults[1] << std::endl;
        // std::cout << std::endl << "cable_forces: " << std::endl << FKresults[2] << std::endl;
        // std::cout << std::endl << "c1: " << std::endl << FKresults[3] << std::endl;
        // std::cout << std::endl << "c2: " << std::endl << FKresults[4] << std::endl;
        // std::cout << std::endl << "b_in_w: " << std::endl << FKresults[5] << std::endl;
        double position_error_before_kinematic_update = (p_platform - gtsam::Vector3{p_platform_collection[i][0], p_platform_collection[i][1], p_platform_collection[i][2]}).norm();
        double position_error_after_kinematic_update = (FKresults[1] - gtsam::Vector3{p_platform_collection[i][0], p_platform_collection[i][1], p_platform_collection[i][2]}).norm();
        std::cout << std::endl << "position_error before kinematic estimation: " << position_error_before_kinematic_update;
        std::cout << std::endl << "position_error after  kinematic estimation: " << position_error_after_kinematic_update;
        std::cout << std::endl << "----------------------";

        double pitch_error_before_kinematic_update = std::abs(EigenMatrixToGtsamRot3(rtation_init).pitch() - real_orientation[i][0]*M_PI/180.0);
        double pitch_error_after_kinematic_update  = std::abs(EigenMatrixToGtsamRot3(FKresults[0]).pitch() - real_orientation[i][0]*M_PI/180.0);
        std::cout << std::endl << "pitch_error before kinematic estimation: " << pitch_error_before_kinematic_update;
        std::cout << std::endl << "pitch_error after  kinematic estimation: " << pitch_error_after_kinematic_update;
        std::cout << std::endl << "----------------------";

        double roll_error_before_kinematic_update = std::abs(EigenMatrixToGtsamRot3(rtation_init).roll() - real_orientation[i][1]*M_PI/180.0);
        double roll_error_after_kinematic_update  = std::abs(EigenMatrixToGtsamRot3(FKresults[0]).roll() - real_orientation[i][1]*M_PI/180.0);
        std::cout << std::endl << "roll_error before kinematic estimation: " << roll_error_before_kinematic_update;
        std::cout << std::endl << "roll_error after  kinematic estimation: " << roll_error_after_kinematic_update;
        std::cout << std::endl << "----------------------";

        double yaw_error_before_kinematic_update = std::abs(EigenMatrixToGtsamRot3(rtation_init).yaw() - real_orientation[i][2]*M_PI/180.0);
        double yaw_error_after_kinematic_update  = std::abs(EigenMatrixToGtsamRot3(FKresults[0]).yaw() - real_orientation[i][2]*M_PI/180.0);
        std::cout << std::endl << "yaw_error before kinematic estimation: " << yaw_error_before_kinematic_update;
        std::cout << std::endl << "yaw_error after  kinematic estimation: " << yaw_error_after_kinematic_update;
        std::cout << std::endl << "---------------------------------------------------------------------------------------------------------" << std::endl;

        calibration_result.push_back({position_error_before_kinematic_update, position_error_after_kinematic_update});

        std::ofstream file("./result/calibration_result.csv"); // Create a file stream object
        for (const auto& calib : calibration_result) // Loop through the vector elements
        {
            file << calib[0] << "," << calib[1] << std::endl; // Write each element, separated by commas, and end the line
        }
        file.close(); // Close the file stream
    }
    return 0;
}
