#include "IK.h"
#include "FK.h"
#include "minivectorTemplate.h"
#include <Eigen/Dense>
#include <adolc/adolc.h>
#include <cassert>
#if defined(_WIN32) || defined(WIN32)
  #ifndef _USE_MATH_DEFINES
    #define _USE_MATH_DEFINES
  #endif
#endif
#include <math.h>
using namespace std;

// CSCI 520 Computer Animation and Simulation
// Jernej Barbic and Yijing Li

namespace
{

// Converts degrees to radians.
template<typename real>
inline real deg2rad(real deg) { return deg * M_PI / 180.0; }

template<typename real>
Mat3<real> Euler2Rotation(const real angle[3], RotateOrder order)
{
  Mat3<real> RX = Mat3<real>::getElementRotationMatrix(0, deg2rad(angle[0]));
  Mat3<real> RY = Mat3<real>::getElementRotationMatrix(1, deg2rad(angle[1]));
  Mat3<real> RZ = Mat3<real>::getElementRotationMatrix(2, deg2rad(angle[2]));

  switch(order)
  {
    case RotateOrder::XYZ:
      return RZ * RY * RX;
    case RotateOrder::YZX:
      return RX * RZ * RY;
    case RotateOrder::ZXY:
      return RY * RX * RZ;
    case RotateOrder::XZY:
      return RY * RZ * RX;
    case RotateOrder::YXZ:
      return RZ * RX * RY;
    case RotateOrder::ZYX:
      return RX * RY * RZ;
  }
  assert(0);
}

// Performs forward kinematics, using the provided "fk" class.
// This is the function whose Jacobian matrix will be computed using adolc.
// numIKJoints and IKJointIDs specify which joints serve as handles for IK:
//   IKJointIDs is an array of integers of length "numIKJoints"
// Input: numIKJoints, IKJointIDs, fk, eulerAngles (of all joints)
// Output: handlePositions (world-coordinate positions of all the IK joints; length is 3 * numIKJoints)
template<typename real>
void forwardKinematicsFunction(
    int numIKJoints, const int * IKJointIDs, const FK & fk,
    const std::vector<real> & eulerAngles, std::vector<real> & handlePositions)
{
  //   Students should implement this.
  //   The implementation of this function is very similar to function computeLocalAndGlobalTransforms in the FK class.
  //   The recommended approach is to first implement FK::computeLocalAndGlobalTransforms.
  //   Then, implement the same algorithm into this function. To do so,
  // you can use fk.getJointUpdateOrder(), fk.getJointRestTranslation(), and fk.getJointRotateOrder() functions.
  // Also useful is the multiplyAffineTransform4ds function in minivectorTemplate.h .
  //   It would be in principle possible to unify this "forwardKinematicsFunction" and FK::computeLocalAndGlobalTransforms(),
  //   so that code is only written once. We considered this; but it is actually not easily doable.
  //   If you find a good approach, feel free to document it in the README file, for extra credit.

  // 1) Convert Euler angles to RigidTransform4d
  std::vector<Mat3<real>> localTransformsR;
  std::vector<Vec3<real>> localTransformsT;

  for(int i=0; i<fk.getNumJoints(); i++) {
    Vec3<real> eulerAngle = { eulerAngles[3*i+0], eulerAngles[3*i+1], eulerAngles[3*i+2] };
    Mat3<real> R = Euler2Rotation(eulerAngle.data(), fk.getJointRotateOrder(i));
    
    Vec3<real> restEulerAngle = { fk.getJointOrient(i)[0], fk.getJointOrient(i)[1], fk.getJointOrient(i)[2] };
    Mat3<real> jointOrientationR = Euler2Rotation(restEulerAngle.data(), fk.getJointRotateOrder(i));

    // Pass 4x4 transformation matrix
    localTransformsR.push_back(jointOrientationR * R);
    Vec3<real> jointRestTranslation = { fk.getJointRestTranslation(i)[0], fk.getJointRestTranslation(i)[1], fk.getJointRestTranslation(i)[2] };
    localTransformsT.push_back(jointRestTranslation);
  }

  // 2) Recursively compute globalTransforms (root -> leaves)
  std::vector<Mat3<real>> globalTransformsR;
  std::vector<Vec3<real>> globalTransformsT;
  for(int i=0; i<fk.getNumJoints(); i++)
  {
    int curIdx = fk.getJointUpdateOrder(i);
    int parentIdx = fk.getJointParent(curIdx);

    if (parentIdx == -1) // Root
    {
      globalTransformsR.push_back(localTransformsR[curIdx]);
      globalTransformsT.push_back(localTransformsT[curIdx]);
    }
    else
    {
      Mat3<real> Rout;
      Vec3<real> Tout;
      multiplyAffineTransform4ds(
        globalTransformsR[parentIdx], globalTransformsT[parentIdx],
        localTransformsR[curIdx], localTransformsT[curIdx],
        Rout, Tout);
      globalTransformsR.push_back(Rout);
      globalTransformsT.push_back(Tout);
    }
  }

  // 3) Apply global transformation to end joints
  for(int i=0; i<numIKJoints; i++)
  {
    int jointID = IKJointIDs[i];
    handlePositions[3*i+0] = globalTransformsT[jointID][0];
    handlePositions[3*i+1] = globalTransformsT[jointID][1];
    handlePositions[3*i+2] = globalTransformsT[jointID][2];
  }
}

} // end anonymous namespaces

IK::IK(int numIKJoints, const int * IKJointIDs, FK * inputFK, int adolc_tagID)
{
  this->numIKJoints = numIKJoints;
  this->IKJointIDs = IKJointIDs;
  this->fk = inputFK;
  this->adolc_tagID = adolc_tagID;

  FKInputDim = fk->getNumJoints() * 3;
  FKOutputDim = numIKJoints * 3;

  train_adolc();
}

void IK::train_adolc()
{
  // Students should implement this.
  // Here, you should setup adol_c:
  //   Define adol_c inputs and outputs. 
  //   Use the "forwardKinematicsFunction" as the function that will be computed by adol_c.
  //   This will later make it possible for you to compute the gradient of this function in IK::doIK
  //   (in other words, compute the "Jacobian matrix" J).
  // See ADOLCExample.cpp .

  // 1) Call trace_on to ask ADOL-C to begin recording how function f is implemented
  trace_on(adolc_tagID); // Start tracking computation with ADOL-C

  // 2) Define input of the function f
  vector<adouble> eulerAngles(FKInputDim); // define the input of the function f
  for(int i = 0; i < FKInputDim; i++)
    eulerAngles[i] <<= 0.0;

  // 3) Define output of the function f
  vector<adouble> handlePositions(FKOutputDim);

  // 4) Define the computation of f
  forwardKinematicsFunction(numIKJoints, IKJointIDs, *fk, eulerAngles, handlePositions);

  // 5) Pass the output
  vector<double> output(FKOutputDim);
  for(int i = 0; i < FKOutputDim; i++)
    handlePositions[i] >>= output[i]; // Use >>= to tell ADOL-C that y[i] are the output variables

  // 6) Call trace_off to stop recording the function f
  trace_off(); // ADOL-C tracking finished
}

void IK::doIK(const Vec3d * targetHandlePositions, Vec3d * jointEulerAngles)
{
  int numJoints = fk->getNumJoints();

  // Students should implement this.
  // Use adolc to evalute the forwardKinematicsFunction and its gradient (Jacobian). It was trained in train_adolc().
  // Specifically, use ::function, and ::jacobian .
  // See ADOLCExample.cpp .
  //
  // Use it implement the Tikhonov IK method (or the pseudoinverse method for extra credit).
  // Note that at entry, "jointEulerAngles" contains the input Euler angles. 
  // Upon exit, jointEulerAngles should contain the new Euler angles.

  // Ask ADOL-C to evaluate f for different x: y = f(x)
  double newHandlePositions[FKOutputDim];
  for (int i = 0; i < FKOutputDim; i++)
    newHandlePositions[i] = 0.0;
  ::function(adolc_tagID, FKOutputDim, FKInputDim, jointEulerAngles->data(), newHandlePositions);

  // Ask ADOL-C to evalute the jacobian matrix of f on different x: df/dx
  double jacobianMatrix[FKOutputDim*FKInputDim]; // Row-major matrix
  double * jacobianMatrixEachRow[FKOutputDim]; // Pointer array where each pointer points to one row of the jacobian matrix
  for (int i = 0; i < FKOutputDim; i++)
    jacobianMatrixEachRow[i] = &jacobianMatrix[i*FKInputDim];
  ::jacobian(adolc_tagID, FKOutputDim, FKInputDim, jointEulerAngles->data(), jacobianMatrixEachRow);

  ////////////////////////////////////////////////////////////////////////////////////
  // Solve linear system

  // Create J (mxn)
  Eigen::MatrixXd J(FKOutputDim, FKInputDim); // Eigen column-major matrix
  for(int rowID = 0; rowID < FKOutputDim; rowID++)
    for(int colID = 0; colID < FKInputDim; colID++)
      J(rowID,colID) = jacobianMatrix[FKInputDim*rowID+colID];

  // Create J^T (nxm)
  Eigen::MatrixXd JT = J.transpose();

  // Create I (nxn)
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(FKInputDim, FKInputDim);

  // Create db (mx1) 
  Eigen::VectorXd db(FKOutputDim); // Eigen column vector
  for(int i = 0; i < numIKJoints; i++)
  {
    db(3*i+0) = targetHandlePositions[i][0] - newHandlePositions[3*i+0]; // x
    db(3*i+1) = targetHandlePositions[i][1] - newHandlePositions[3*i+1]; // y
    db(3*i+2) = targetHandlePositions[i][2] - newHandlePositions[3*i+2]; // z
  }

  // Solve for dt in (JT*J+a*I)dt = JT*db
  double alpha = 0.01;
  Eigen::VectorXd dt = (JT * J + alpha * I).ldlt().solve(JT * db);

  // Push the result back to jointEulerAngles
  for(int i = 0; i < numJoints; i++)
  {
    jointEulerAngles[i][0] += dt(3*i+0); // x
    jointEulerAngles[i][1] += dt(3*i+1); // y
    jointEulerAngles[i][2] += dt(3*i+2); // z
  }
}

