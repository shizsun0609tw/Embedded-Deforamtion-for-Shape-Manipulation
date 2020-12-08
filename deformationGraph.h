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
	vector<int> GetSamplingVertices();

private:
	void CalConnectedMap();
	void CalSamplingVertices();

private:
	_GLMmodel* mesh;
	_GLMmodel* samplingMesh;
	vector<int> sample_vertices;
	vector<int> sample_edges;
	map<int, set<int>> connectedMap;
};