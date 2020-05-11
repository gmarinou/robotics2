#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <rbdl/rbdl.h>
#include <rbdl/addons/luamodel/luamodel.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double g = 9.81;

MatrixNd CalcOrientationEulerXYZ( double x, double y, double z ) {
    return rotx(x) * roty(y) * rotz(z);
}


int main(int argc, char *argv[]) {

	//Initial values
	Model humanoid;

	if (! RigidBodyDynamics::Addons::LuaModelReadFromFile( "../models/humanoid_model.lua" , &humanoid, false)) {
		std::cerr << "Error loading model - aborting" << std::endl;
		abort();
	}
	
    VectorNd q (VectorNd::Zero(humanoid.dof_count));
    VectorNd dq (VectorNd::Zero(humanoid.dof_count));
	double mass;
    Vector3d com;
    
    RigidBodyDynamics::Utils::CalcCenterOfMass(humanoid, q, dq, NULL ,mass, com);
    
    Vector3d r_foot = CalcBodyToBaseCoordinates(humanoid, q, humanoid.GetBodyId("foot_right"), Vector3dZero );
    Vector3d l_foot = CalcBodyToBaseCoordinates(humanoid, q, humanoid.GetBodyId("foot_left"), Vector3dZero );
    Vector3d pelv = CalcBodyToBaseCoordinates(humanoid, q, humanoid.GetBodyId("pelvis"), Vector3dZero );
	
    std::cout << "The humanoid robot has " << humanoid.dof_count << " degrees of freedom and ways " << mass << " kilograms" << std::endl;
    std::cout << "Center of mass position: " << com.transpose() << std::endl;
    std::cout << "Feet distance: " << (l_foot - r_foot)[1] << std::endl;
    std::cout << "Pelvis height: " << (pelv - r_foot)[2] << std::endl;
    
    // Compute inverse kinematic
	InverseKinematicsConstraintSet CS;
    
    // Change this parameter for infeasible configurations
    CS.lambda = 0.00001;
    VectorNd q_init (VectorNd::Zero(humanoid.dof_count));
    
    // Bent knees in the same direction
    q_init[12] = 0.2;
    q_init[18] = 0.2;
    VectorNd q_res (VectorNd::Zero(humanoid.dof_count));
    
    Vector3d R_foot_pos = Vector3d(0.0, -0.25, 0.0);
    Vector3d L_foot_pos = Vector3d(0.0, 0.25, 0.0);
    Vector3d Pelvis_pos = Vector3d(0.0, 0.0, 0.6);
    
    Matrix3d R_foot_ort = Matrix3dIdentity;
    Matrix3d L_foot_ort = Matrix3dIdentity;
    Matrix3d Pelvis_ort = Matrix3dIdentity;
    
    CS.AddFullConstraint(humanoid.GetBodyId("foot_right"), Vector3d(0.0, 0.0, -0.1), R_foot_pos, R_foot_ort);
    CS.AddFullConstraint(humanoid.GetBodyId("foot_left"), Vector3d(0.0,  0.0, -0.1), L_foot_pos, L_foot_ort);
    CS.AddOrientationConstraint(humanoid.GetBodyId("base_link"), Pelvis_ort);
    unsigned int pelvis = CS.AddFullConstraint(humanoid.GetBodyId("pelvis"), Vector3dZero, Pelvis_pos, Pelvis_ort);
    
    std::ofstream of("animation.csv");
	std::ofstream ff("arrows.ff");

    for (unsigned int i = 0; i < 100; i ++){
        
        double Pelvis_height = 0.75 + 0.4 * sin( M_PI * i / 10. );
        double Pelvis_angle = sin( M_PI * i / 10. );
        
        CS.target_positions[pelvis] = Vector3d(0.0, 0.0, Pelvis_height);
        CS.target_orientations[pelvis] = CalcOrientationEulerXYZ( 0.0, 0.0, Pelvis_angle);
        
        bool ret = InverseKinematics(humanoid, q_init, CS, q_res);
        if (!ret) {
            std::cout << "InverseKinematics did not find a solution" << std::endl;
        }
        RigidBodyDynamics::Utils::CalcCenterOfMass(humanoid, q_res, dq, NULL ,mass, com);
        q_init = q_res;
        
        of << i / 10. << ", ";
        ff << i / 10. << ", ";
        for (unsigned int j = 0; j < humanoid.dof_count; j++){
            of << q_res[j] << ", ";
        }
        of << i << ",\n";
        ff << com[0] << ", " << com[1] << ", " << com[2] << ", 1000, 0, 0, 0, 0, 0\n";
        
    }
    
	of.close();
	return 0;
}
