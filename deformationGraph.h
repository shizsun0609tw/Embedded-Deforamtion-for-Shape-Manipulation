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

using namespace std;

class DeformationGraph
{
public:
	DeformationGraph();

public:
	void Init(_GLMmodel* originMesh, _GLMmodel* samplingMesh);

public:
	void Run();
	void SetControlPoints(vector<vector<int>> controlPoints);
	vector<int> GetSamplingIndices();

private:
	void UpdateOriginMesh();
	void CalConnectedMap();
	void CalSamplingVertices();

private:
	_GLMmodel* mesh;
	_GLMmodel* samplingMesh;
	vector<int> sample_idices;
	vector<int> sample_edges;
	vector<vector<int>> control_points;
	map<int, set<int>> connectedMap;
};