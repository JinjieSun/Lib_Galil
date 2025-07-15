
#ifndef CTCR_KINEMATICS_CLIB_H
#define CTCR_KINEMATICS_CLIB_H

#include "ctcr_kinematics_clib_global.h"
#include "stdbool.h"

//! \brief C-Style Library to call the C++ ForwardKinematic Library from Matlab.
//!
//! The functionality is limited to calulating the forward kinematic of a tubuluar continuum robot
//! with exactly 3 tubes. It can be choosen, if the kinematic should calulate the body frame Jacobian or not.
//! Please note, that the calulation without the Jacobian is significantly faster.
//! Curently not supported:
//!     Calculation of compliance matrix
//!     External loads
//!
//! If calling this library from Matlab please remember, that Matlab is mapping multidimensional arrays
//! to memory in column-major order. Instread C++ is using row-major order.
//!
//! The memory for the results must be preallocated by Matlab. Therefore the preallocated memory
//! must provide enough space for all results. If there is not enough space for every robot backbone point,
//! no data is copied to the preallocated memory.
//!
//! If an error occures during any function call, an error message can be retrieved by calling getErrMsg().
//!
//! \note Matlab only supports only a very limited number of C-compilers. It must match the compiler of this library
//! and therefore the compiler of the used Qt-Version (the kinematic calculation relies partly on Qt).
//! Matlab is only available as 64Bit-version. Because there is no official MinGW-64Bit-version of Qt,
//! you must compile Qt yourself!
//! Supported compiler:
//!     Matlab 2016b: gcc (MinGW 4.9.2 64Bit)


// Block for C Compiler
#ifdef __cplusplus
extern "C" {
#endif

// WARNING: works for 3 tubes ONLY
#define NUM_JOINT_VARS 6 //!< number of joint variables (3 tubes with 1 translation and 1 rotation)
#define NUM_CART_VARS 6 //!< number of Cartesian variables (x,y,z + 3 rotations)
#define NUM_SUPPORTED_TUBES 3 //!< Currently only 3 tubes are supported

// Library functions

//! \brief Initializes the forward kinematics with the tube parameters.
//! \param [in] canulaFileName Path and name of xml-file which contains the tube descripton
//! \return true if initialization was successful; false otherwise
CTCR_KINEMATICS_CLIBSHARED_EXPORT bool initKinematic(const char * canulaFileName);

//! \brief Calulates the forward kinematic s if an 3-tube concentric tube robot WITHOUT Jacobian.
//!
//! External loads and Compliance matrix is not supported.
//! Please note that nearly no checks are performed, if the arrays have the correct size.
//! \param [in] alpha_rad Pointer to array which holds the tube rotations in rad.
//! \param [in] beta_m Pointer to array which holds the tube translations in meters. Must be <= 0.
//! \param [in] numAllocatedPnts Signals for how many backbone points memory has been allocated.
//!                              If more backbone points are calculated by the kinematics,
//!                              the function returns 0 and no results are copied.
//! \param [out] backbonePntsPos Pointer to 2D-array which should hold the position (x,y,z) of
//!                              all backbone points. Dimension must be [numAllocatedPnts x 3]
//! \param [out] backbonePntsRot Pointer to 2D-array which should hold the rotation matrix of
//!                              all backbone points. Each rotation matrix is stored as a row vector
//!                              which can be retrieved in Matlab by using the reshape command.
//!                              Dimension must be [numAllocatedPnts x 9]
//! \param [out] theta           Pointer to 2D-array which should hold the rotation angle in rad
//!                              of the tubes in relation to the innermost tube at every backbone point.
//!                              Dimension must be [numAllocatedPnts x NUM_SUPPORTED_TUBES-1]
//! \param [out] tubeEndIdx      Pointer to array which should hold the index of the backbonepoint,
//!                              at which each tube ends. Dimension must be [NUM_SUPPORTED_TUBES]
//! \return Number of calulated and returned backbone points. If 0 is returned, an error occured.
CTCR_KINEMATICS_CLIBSHARED_EXPORT int calcForwKinematicSimple(
            double alpha_rad[NUM_SUPPORTED_TUBES], double beta_m[NUM_SUPPORTED_TUBES],
            int numAllocatedPnts,
            double backbonePntsPos[][3], double backbonePntsRot[][9],
            double theta[][NUM_SUPPORTED_TUBES-1],int tubeEndIdx[NUM_SUPPORTED_TUBES]);

//! \brief Calulates the forward kinematic s if an 3-tube concentric tube robot WITH
//!        body frame Jacobian.
//!
//! External loads and Compliance matrix is not supported.
//! Please note that nearly no checks are performed, if the arrays have the correct size.
//! \param [in] alpha_rad Pointer to array which holds the tube rotations in rad.
//! \param [in] beta_m Pointer to array which holds the tube translations in meters. Must be <= 0.
//! \param [in] numAllocatedPnts Signals for how many backbone points memory has been allocated.
//!                              If more backbone points are calculated by the kinematics,
//!                              the function returns 0 and no results are copied.
//! \param [out] backbonePntsPos Pointer to 2D-array which should hold the position (x,y,z) of
//!                              all backbone points. Dimension must be [numAllocatedPnts x 3]
//! \param [out] backbonePntsRot Pointer to 2D-array which should hold the rotation matrix of
//!                              all backbone points. Each rotation matrix is stored as a row vector
//!                              which can be retrieved in Matlab by using the reshape command.
//!                              Dimension must be [numAllocatedPnts x 9]
//! \param [out] theta           Pointer to 2D-array which should hold the rotation angle in rad
//!                              of the tubes in relation to the innermost tube at every backbone point.
//!                              Dimension must be [numAllocatedPnts x NUM_SUPPORTED_TUBES-1]
//! \param [out] tubeEndIdx      Pointer to array which should hold the index of the backbonepoint,
//!                              at which each tube ends. Dimension must be [NUM_SUPPORTED_TUBES]
//! \param [out] jacobianBody    Pointer to 2D-array which should hold the body Jacobian of the robot tip.
//!                              The Jacobian matrix is stored as a row vector which can be retrieved
//!                              in Matlab by using the reshape command.
//!                              Dimension must be [NUM_CART_VARS x NUM_JOINT_VARS]
//! \param [out] jacobianSpatial Pointer to 2D-array which should hold the spatial Jacobian of the robot tip.
//!                              The Jacobian matrix is stored as a row vector which can be retrieved
//!                              in Matlab by using the reshape command.
//!                              Dimension must be [NUM_CART_VARS x NUM_JOINT_VARS]
//! \return Number of calulated and returned backbone points. If 0 is returned, an error occured.
CTCR_KINEMATICS_CLIBSHARED_EXPORT int calcForwKinematicJacobian(
            double alpha_rad[NUM_SUPPORTED_TUBES], double beta_m[NUM_SUPPORTED_TUBES],
            int numAllocatedPnts,
            double backbonePntsPos[][3], double backbonePntsRot[][9],
            double theta[][NUM_SUPPORTED_TUBES-1],int tubeEndIdx[NUM_SUPPORTED_TUBES],
            double jacobianBody[NUM_CART_VARS][NUM_JOINT_VARS], double jacobianSpatial[NUM_CART_VARS][NUM_JOINT_VARS]);

//! \brief Calulates the forward kinematic s if an 3-tube concentric tube robot including external forces and moments
//!        at the tip WITH body frame Jacobian and Compliance Matrices.
//!
//! Please note that nearly no checks are performed, if the arrays have the correct size.
//! \param [in] alpha_rad        Pointer to array which holds the tube rotations in rad.
//! \param [in] beta_m           Pointer to array which holds the tube translations in meters. Must be <= 0.
//! \param [in] wrench           Pointer to array which holds the external wrench at the tip in N and Nm.
//! \param [in] numAllocatedPnts Signals for how many backbone points memory has been allocated.
//!                              If more backbone points are calculated by the kinematics,
//!                              the function returns 0 and no results are copied.
//! \param [out] backbonePntsPos Pointer to 2D-array which should hold the position (x,y,z) of
//!                              all backbone points. Dimension must be [numAllocatedPnts x 3]
//! \param [out] backbonePntsRot Pointer to 2D-array which should hold the rotation matrix of
//!                              all backbone points. Each rotation matrix is stored as a row vector
//!                              which can be retrieved in Matlab by using the reshape command.
//!                              Dimension must be [numAllocatedPnts x 9]
//! \param [out] theta           Pointer to 2D-array which should hold the rotation angle in rad
//!                              of the tubes in relation to the innermost tube at every backbone point.
//!                              Dimension must be [numAllocatedPnts x NUM_SUPPORTED_TUBES-1]
//! \param [out] tubeEndIdx      Pointer to array which should hold the index of the backbonepoint,
//!                              at which each tube ends. Dimension must be [NUM_SUPPORTED_TUBES]
//! \param [out] jacobianBody    Pointer to 2D-array which should hold the body Jacobian of the robot tip.
//!                              The Jacobian matrix is stored as a row vector which can be retrieved
//!                              in Matlab by using the reshape command.
//!                              Dimension must be [NUM_CART_VARS x NUM_JOINT_VARS]
//! \param [out] jacobianSpatial Pointer to 2D-array which should hold the spatial Jacobian of the robot tip.
//!                              The Jacobian matrix is stored as a row vector which can be retrieved
//!                              in Matlab by using the reshape command.
//!                              Dimension must be [NUM_CART_VARS x NUM_JOINT_VARS]
//! \param [out] compliance      Pointer to 2D-array which should hold the Compliance of the robot tip.
//!                              The Compliance matrix is stored as a row vector which can be retrieved
//!                              in Matlab by using the reshape command.
//!                              Dimension must be [NUM_CART_VARS x NUM_JOINT_VARS]
//!
//! \return Number of calulated and returned backbone points. If 0 is returned, an error occured.
CTCR_KINEMATICS_CLIBSHARED_EXPORT int calcForwKinematicWrenchJacobianCompliance(
            double alpha_rad[NUM_SUPPORTED_TUBES], double beta_m[NUM_SUPPORTED_TUBES], double wrench[NUM_CART_VARS],
            int numAllocatedPnts,
            double backbonePntsPos[][3], double backbonePntsRot[][9],
            double theta[][NUM_SUPPORTED_TUBES-1],int tubeEndIdx[NUM_SUPPORTED_TUBES],
            double jacobianBody[NUM_CART_VARS][NUM_JOINT_VARS], double jacobianSpatial[NUM_CART_VARS][NUM_JOINT_VARS],
            double compliance[NUM_CART_VARS][NUM_JOINT_VARS]);


//! \brief Resets the initial conditions of ODE solver for the kinematic calculations (u_z = m_x = m_y = 0).
//!
//! Normaly, the kinematic calculations use the last calculation result as start condition for the next calculation.
//! This speeds up the calculation of successive points in the workspace. If you want to calculate the kinematics from
//! nearly independent configuration parameters, you should call this function before each kinematic calculation.
CTCR_KINEMATICS_CLIBSHARED_EXPORT void resetInitialConditions();

//! \brief Returns an error message after an error occured.
CTCR_KINEMATICS_CLIBSHARED_EXPORT const char * getErrMsg();

#ifdef __cplusplus
}
#endif


#endif // CTCR_KINEMATICS_CLIB_H
