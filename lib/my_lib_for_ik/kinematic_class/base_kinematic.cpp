#include "base_kinematic.hpp"

void iiwa_kinematics::saveMatrix(std::string name, Eigen::Matrix<double,N_JOINTS,3> mat)
{
    std::ofstream ofs;
    // ofs.open(name, std::ofstream::out | std::ios_base::app);
    ofs.open(name, std::ofstream::out);

    if (!ofs.is_open()) { 
        std::cout << "ERROR" << std::endl;
        return;
    } 

    for (int i = 0; i < N_JOINTS; ++i)
    {
        ofs << mat(i,0) << "\t" << mat(i,1) << "\t" << mat(i,2) << "\n";
    }
  
    ofs.close(); 
}