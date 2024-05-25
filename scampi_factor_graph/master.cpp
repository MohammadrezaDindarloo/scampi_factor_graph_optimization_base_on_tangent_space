#include "src/main.cpp"

/*
    GT Data 
    l_gt: 183.324069126233,169.162981532604,155.642263929178,170.927219368362
    f1_gt: 690.675811131,105.732855715641 
    ee_poistion_gt: 9.94283158474076,9.93145571155357,15.9060269359906
    ee_orientation_gt: -0.19122393512445,-1.06322173025346,0.971850603803798
*/

int main(int argc, char *argv[])
{   
    // robot characteristic
    CableRobotParams robot_params(0.7100703113867337, 333.54);

    std::default_random_engine generator(std::random_device{}());
    std::normal_distribution<double> distribution(0.0, 3.0/sqrt(3.0));

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

    Eigen::Vector3d p_platform(9.94283158474076 + distribution(generator),9.93145571155357 + distribution(generator),15.9060269359906 + distribution(generator));

    gtsam::Rot3 rot_init_;
    double pitch = -0.19122393512445 * M_PI/180.0;
    double roll = -1.06322173025346 * M_PI/180.0;
    double yaw = 0.971850603803798 * M_PI/180.0;
    rot_init_ = rot_init_.Ypr(yaw, pitch, roll);
    Eigen::Matrix3d rot_init = gtsamRot3ToEigenMatrix(rot_init_);
    
    // start inverse optimization
    std::vector<MatrixXd> IKresults = IK_Factor_Graph_Optimization(robot_params, rot_init, p_platform);
    // the result of inverse optimization
    std::cout << std::endl << "-------------------inverse result--------------------------" << std::endl;
    // std::cout << std::endl << "rot_platform: " << std::endl << IKresults[0] << std::endl;
    // std::cout << std::endl << "l_cat: " << std::endl << IKresults[1] << std::endl;
    // std::cout << std::endl << "cable_forces: " << std::endl << IKresults[2] << std::endl;
    // std::cout << std::endl << "c1: " << std::endl << IKresults[3] << std::endl;
    // std::cout << std::endl << "c2: " << std::endl << IKresults[4] << std::endl;
    // std::cout << std::endl << "b_in_w: " << std::endl << IKresults[5] << std::endl;
    std::cout << std::endl << "l_cat_error: " << std::endl << (IKresults[1] - gtsam::Vector4{183.324069126233,169.162981532604,155.642263929178,170.927219368362}).norm() << std::endl;
    // std::cout << std::endl << "cable_forces_error: " << std::endl << (IKresults[2].col(0) - gtsam::Vector2{690.675811131,105.732855715641}).norm() << std::endl;

    // start forward optimization
    Eigen::VectorXd lc_cat = gtsam::Vector4{183.324069126233,169.162981532604,155.642263929178,170.927219368362}; // IKresults[1]
    Eigen::Vector2d fc_1 = IKresults[2].col(0); 
    Eigen::Vector3d pos_init = p_platform;
    Eigen::Matrix3d rtation_init = rot_init;
    std::vector<MatrixXd> FKresults = FK_Factor_Graph_Optimization(robot_params, lc_cat, fc_1, pos_init, rtation_init);
    // the result of forward optimization
    std::cout << std::endl << "-------------------forward result--------------------------" << std::endl;
    // std::cout << std::endl << "rot_platform: " << std::endl << FKresults[0] << std::endl;
    // std::cout << std::endl << "p_platform: " << std::endl << FKresults[1] << std::endl;
    // std::cout << std::endl << "cable_forces: " << std::endl << FKresults[2] << std::endl;
    // std::cout << std::endl << "c1: " << std::endl << FKresults[3] << std::endl;
    // std::cout << std::endl << "c2: " << std::endl << FKresults[4] << std::endl;
    // std::cout << std::endl << "b_in_w: " << std::endl << FKresults[5] << std::endl;
    std::cout << std::endl << "position_error before kinematic estimation: " << std::endl << (p_platform - gtsam::Vector3{9.94283158474076,9.93145571155357,15.9060269359906}).norm() << std::endl;
    std::cout << std::endl << "position_error after  kinematic estimation: " << std::endl << (FKresults[1] - gtsam::Vector3{9.94283158474076,9.93145571155357,15.9060269359906}).norm() << std::endl;
    // std::cout << std::endl << "cable_forces_error: " << std::endl << (FKresults[2].col(0) - gtsam::Vector2{690.675811131,105.732855715641}).norm() << std::endl;

    return 0;
}
