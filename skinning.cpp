#include "skinning.h"
#include "vec3d.h"
#include <algorithm>
#include <cassert>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
using namespace std;

// CSCI 520 Computer Animation and Simulation
// Jernej Barbic and Yijing Li

Skinning::Skinning(int numMeshVertices, const double * restMeshVertexPositions,
    const std::string & meshSkinningWeightsFilename)
{
  this->numMeshVertices = numMeshVertices;
  this->restMeshVertexPositions = restMeshVertexPositions;

  cout << "Loading skinning weights..." << endl;
  ifstream fin(meshSkinningWeightsFilename.c_str());
  assert(fin);
  int numWeightMatrixRows = 0, numWeightMatrixCols = 0;
  fin >> numWeightMatrixRows >> numWeightMatrixCols;
  assert(fin.fail() == false);
  assert(numWeightMatrixRows == numMeshVertices);
  int numJoints = numWeightMatrixCols;

  vector<vector<int>> weightMatrixColumnIndices(numWeightMatrixRows);
  vector<vector<double>> weightMatrixEntries(numWeightMatrixRows);
  fin >> ws;
  while(fin.eof() == false)
  {
    int rowID = 0, colID = 0;
    double w = 0.0;
    fin >> rowID >> colID >> w;
    weightMatrixColumnIndices[rowID].push_back(colID);
    weightMatrixEntries[rowID].push_back(w);
    assert(fin.fail() == false);
    fin >> ws;
  }
  fin.close();

  // Build skinning joints and weights.
  numJointsInfluencingEachVertex = 0;
  for (int i = 0; i < numMeshVertices; i++)
    numJointsInfluencingEachVertex = std::max(numJointsInfluencingEachVertex, (int)weightMatrixEntries[i].size());
  assert(numJointsInfluencingEachVertex >= 2);

  // Copy skinning weights from SparseMatrix into meshSkinningJoints and meshSkinningWeights.
  meshSkinningJoints.assign(numJointsInfluencingEachVertex * numMeshVertices, 0);
  meshSkinningWeights.assign(numJointsInfluencingEachVertex * numMeshVertices, 0.0);
  for (int vtxID = 0; vtxID < numMeshVertices; vtxID++)
  {
    vector<pair<double, int>> sortBuffer(numJointsInfluencingEachVertex);
    for (size_t j = 0; j < weightMatrixEntries[vtxID].size(); j++)
    {
      int frameID = weightMatrixColumnIndices[vtxID][j];
      double weight = weightMatrixEntries[vtxID][j];
      sortBuffer[j] = make_pair(weight, frameID);
    }
    sortBuffer.resize(weightMatrixEntries[vtxID].size());
    assert(sortBuffer.size() > 0);
    sort(sortBuffer.rbegin(), sortBuffer.rend()); // sort in descending order using reverse_iterators
    for(size_t i = 0; i < sortBuffer.size(); i++)
    {
      meshSkinningJoints[vtxID * numJointsInfluencingEachVertex + i] = sortBuffer[i].second;
      meshSkinningWeights[vtxID * numJointsInfluencingEachVertex + i] = sortBuffer[i].first;
    }

    // Note: When the number of joints used on this vertex is smaller than numJointsInfluencingEachVertex,
    // the remaining empty entries are initialized to zero due to vector::assign(XX, 0.0) .
  }
}

void Skinning::applyLBS(const RigidTransform4d * jointSkinTransforms, double * newMeshVertexPositions) const
{
  // Method 1: Linear Blend Skinning

  // pi = SUM( wj * Mj * pi_rest )
  for (int i=0; i<numMeshVertices; i++)
  {
    // Initialize to 0
    newMeshVertexPositions[3*i+0] = 0.0;
    newMeshVertexPositions[3*i+1] = 0.0;
    newMeshVertexPositions[3*i+2] = 0.0;

    for (int j=0; j<numJointsInfluencingEachVertex; j++)
    {
      int idx = i * numJointsInfluencingEachVertex + j; // look up index for meshSkinningJoints and meshSkinningWeights
      int jointIdx = meshSkinningJoints[idx];

      // Compute weighted transformed vertex position for a joint
      Vec4d p_rest = Vec4d(restMeshVertexPositions[3*i+0], restMeshVertexPositions[3*i+1], restMeshVertexPositions[3*i+2], 1.0);
      Vec4d p = meshSkinningWeights[idx] * jointSkinTransforms[jointIdx] * p_rest;

      // Add joint's contribution
      newMeshVertexPositions[3*i+0] += p[0];
      newMeshVertexPositions[3*i+1] += p[1];
      newMeshVertexPositions[3*i+2] += p[2];
    }
  }
}

void Skinning::applyDQS(const RigidTransform4d * jointSkinTransforms, double * newMeshVertexPositions) const
{
  // Method 2: Dual Quaternion Skinning

  for (int i=0; i<numMeshVertices; i++)
  {
    // Initialize dual quaternion (q1, q2)
    std::vector<Eigen::Quaterniond> q1(numMeshVertices), q2(numMeshVertices);
    for (int j=0; j<numMeshVertices; j++)
    {
      q1[i] = { 0.0, 0.0, 0.0, 0.0 };
      q2[i] = { 0.0, 0.0, 0.0, 0.0 };
    }

    // q = SUM( Wj * q_j )
    for (int j=0; j<numJointsInfluencingEachVertex; j++)
    {
      int idx = i * numJointsInfluencingEachVertex + j; // look up index for meshSkinningJoints and meshSkinningWeights
      int jointIdx = meshSkinningJoints[idx];

      Eigen::Matrix3d R; // Eigen column-major matrix
      for(int rowID = 0; rowID < 3; rowID++)
        for(int colID = 0; colID < 3; colID++)
          R(rowID,colID) = jointSkinTransforms[jointIdx][rowID][colID];

      Vec3d t_tmp = jointSkinTransforms[jointIdx].getTranslation();
      Eigen::Vector4d t = { 0.0, t_tmp[0], t_tmp[1], t_tmp[2] }; // w, x, y, z

      Eigen::Quaterniond q1_j = Eigen::Quaterniond(R); // q1 <- R
      Eigen::Quaterniond q2_j = { // q2 <- (1/2 * t * q1)
        0.5 * t[0] * q1[i].w(),
        0.5 * t[0] * q1[i].x(),
        0.5 * t[1] * q1[i].y(),
        0.5 * t[2] * q1[i].z()
      };

      // Normalize dual quaternion
      q1_j.normalized();
      q2_j.normalized();
      
      // q1[i] += w * q1_j
      q1[i] = Eigen::Quaterniond(
        q1[i].w() + meshSkinningWeights[idx] * q1_j.w(),
        q1[i].x() + meshSkinningWeights[idx] * q1_j.x(),
        q1[i].y() + meshSkinningWeights[idx] * q1_j.y(),
        q1[i].z() + meshSkinningWeights[idx] * q1_j.z()
      );
      
      // q2[i] += w * q2_j
      q2[i] = Eigen::Quaterniond(
        q2[i].w() + meshSkinningWeights[idx] * q2_j.w(),
        q2[i].x() + meshSkinningWeights[idx] * q2_j.x(),
        q2[i].y() + meshSkinningWeights[idx] * q2_j.y(),
        q2[i].z() + meshSkinningWeights[idx] * q2_j.z()
      );
    }

    // Normalize dual quaternion
    q1[i].normalized();
    q2[i].normalized();

    // x |-> Rx + t
    Eigen::Matrix3d R = q1[i].toRotationMatrix();
    Eigen::Vector3d t = { 2.0*q2[i].x()/q1[i].x(), 2.0*q2[i].y()/q1[i].y(), 2.0*q2[i].z()/q1[i].z() };

    Eigen::Vector3d oldPos = { restMeshVertexPositions[3*i+0], restMeshVertexPositions[3*i+1], restMeshVertexPositions[3*i+2] };
    Eigen::Vector3d newPos = R * oldPos + t;

    // Update vertex position
    newMeshVertexPositions[3*i+0] = newPos[0];
    newMeshVertexPositions[3*i+1] = newPos[1];
    newMeshVertexPositions[3*i+2] = newPos[2];
  }
}

void Skinning::applySkinning(const RigidTransform4d * jointSkinTransforms, double * newMeshVertexPositions) const
{
  bool useLBS = false;

  if (useLBS)
    applyLBS(jointSkinTransforms, newMeshVertexPositions);
  else
    applyDQS(jointSkinTransforms, newMeshVertexPositions);
}

