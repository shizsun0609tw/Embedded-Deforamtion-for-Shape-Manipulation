#pragma once

#ifndef INCLUDE_GLM_H
#define INCLUDE_GLM_H
#include "glm.h"
#include "mtxlib.h"
#endif

#include <iostream>
#include <vector>
#include <set>
#include <map>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>

using namespace std;
using namespace Eigen;

class DeformationGraph
{
public:
	DeformationGraph();

public:
	void Init(_GLMmodel* originMesh, _GLMmodel* samplingMesh);

public:
	void Run();
	void SetControlPoints(vector<vector<int>> controlPoints);
	void SetControlPointsTranslate(int selectedId, vector3 vec);
	vector<int> GetSamplingIndices();

private:
	void InitRotAndTrans();
	void UpdateSampleVertices();
	void ApplyResults();
	void CalConnectedMap();
	void CalSamplingVertices();

private:
	void GaussainNewton();
	void Calf(MatrixXf &f);
	void CalJ(SparseMatrix<float> &J);
	void CalWeights();
	float F(MatrixXf &x);
	float CalErot();
	float CalEcon();
	float CalEreg();

private:
	_GLMmodel* originMesh;
	_GLMmodel* samplingMesh;

private:
	float w_rot;
	float w_reg;
	float w_con;

private:
	int sample_nodes;
	int sample_edges;
	int sample_controls;
	int k_nearest;
	vector<int> sample_idices;
	vector<Matrix3f> rot;
	vector<Vector3f> trans;
	vector<vector<int>> control_points_id;
	vector<vector<vector3>> control_points_data;
	vector<vector<pair<int, float>>> weights;
	map<int, set<int>> connectedMap;
};