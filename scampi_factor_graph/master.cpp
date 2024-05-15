#include "src/main.cpp"

int main(int argc, char *argv[])
{   
    // robot characteristic
    CableRobotParams robot_params(0.1034955, 43.164);

    // Eigen::Vector3d Pulley_a(-1.9874742031097412, -8.319656372070312, 8.471846580505371);
    // Eigen::Vector3d Pulley_b(2.5193355532036756, -8.388501748709967, 8.469020753679201);
    // Eigen::Vector3d Pulley_c(2.717151941069913, 4.774436992746004, 8.364108863330584);
    // Eigen::Vector3d Pulley_d(-1.7965602546229, 4.832889384134232, 8.370128714520508);

    Eigen::Vector3d Pulley_a(-1.9874742 , -8.31965637,  8.47184658);
    Eigen::Vector3d Pulley_b(2.52022147, -8.38887501,  8.46931362);
    Eigen::Vector3d Pulley_c(2.71799795, 4.77520639, 8.36416322);
    Eigen::Vector3d Pulley_d(-1.79662371,  4.83333111,  8.37001991);

    robot_params.setPulleyPoses(Pulley_a, Pulley_b, Pulley_c, Pulley_d);

    Eigen::Vector3d Ee_a(-0.21 , -0.21 , -0.011);  
    Eigen::Vector3d Ee_b(0.21  , -0.21 , -0.011);
    Eigen::Vector3d Ee_c(0.21  ,  0.21 , -0.011);
    Eigen::Vector3d Ee_d(-0.21 ,  0.21 , -0.011);
    robot_params.setEEAnchors(Ee_a, Ee_b, Ee_c, Ee_d);

    Eigen::Vector3d r_to_cog(0, 0, -0.12);
    robot_params.setCog(r_to_cog);

    // Eigen::Vector3d p_platform(3.09173747e-01, -1.83715841e+00,  2.18367984e+00);
    Eigen::Vector3d p_platform(3.09173747e-01, -1.83715841e+00,  2.18367984e+00);
    Eigen::Matrix3d rot_init;
    rot_init << 0.99268615,  0.11337417, -0.04147891,
               -0.11309773,  0.99354347,  0.00895918,
                0.04222684, -0.00420248,  0.99909921; 
    
    // start inverse optimization
    std::vector<MatrixXd> IKresults = IK_Factor_Graph_Optimization(robot_params, rot_init, p_platform);
    // the result of inverse optimization
    std::cout << std::endl << "-------------------inverse result--------------------------" << std::endl;
    std::cout << std::endl << "rot_platform: " << std::endl << IKresults[0] << std::endl;
    std::cout << std::endl << "l_cat: " << std::endl << IKresults[1] << std::endl;
    std::cout << std::endl << "cable_forces: " << std::endl << IKresults[2] << std::endl;
    std::cout << std::endl << "c1: " << std::endl << IKresults[3] << std::endl;
    std::cout << std::endl << "c2: " << std::endl << IKresults[4] << std::endl;
    std::cout << std::endl << "b_in_w: " << std::endl << IKresults[5] << std::endl;

    // start forward optimization
    Eigen::VectorXd lc_cat = IKresults[1];
    Eigen::Vector2d fc_1 = IKresults[2].col(0);
    Eigen::Vector3d pos_init = p_platform;
    Eigen::Matrix3d rtation_init = rot_init;
    std::vector<MatrixXd> FKresults = FK_Factor_Graph_Optimization(robot_params, lc_cat, fc_1, pos_init, rtation_init);
    // the result of forward optimization
    std::cout << std::endl << "-------------------forward result--------------------------" << std::endl;
    std::cout << std::endl << "rot_platform: " << std::endl << FKresults[0] << std::endl;
    std::cout << std::endl << "p_platform: " << std::endl << FKresults[1] << std::endl;
    std::cout << std::endl << "cable_forces: " << std::endl << FKresults[2] << std::endl;
    std::cout << std::endl << "c1: " << std::endl << FKresults[3] << std::endl;
    std::cout << std::endl << "c2: " << std::endl << FKresults[4] << std::endl;
    std::cout << std::endl << "b_in_w: " << std::endl << FKresults[5] << std::endl;

    std::cout << std::endl << "-------------------forward vs inverse--------------------------" << std::endl;
    std::cout << std::endl << "p_platform_error in  mm: " << std::endl << (FKresults[1] - p_platform).norm() * 1000 << std::endl;
    std::cout << std::endl << "cable_forces_error in N: " << std::endl << (FKresults[2] - IKresults[2]).norm() << std::endl;
    std::cout << std::endl << "rot_platform: " << std::endl << FKresults[0].inverse() * (IKresults[0]) << std::endl;

    return 0;
}
