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
	void Init(_GLMmodel* mesh);
	void UniformSampling();
	vector<pair<vector3, vector3>> GetEdges();

private:
	void CalConnectedMap();

private:
	_GLMmodel* mesh;
	vector<vector3> sample_vertices;
	map<int, set<int>> connectedMap;
	vector<int> sample_edges;
};