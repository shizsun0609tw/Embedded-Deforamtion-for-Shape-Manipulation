#include "deformationGraph.h"

DeformationGraph::DeformationGraph()
{

}

void DeformationGraph::Init(_GLMmodel* mesh)
{
	this->mesh = mesh;

    CalConnectedMap();
}

void DeformationGraph::UniformSampling()
{
    const int K_MAX = 4;
    const int DEFAULT = 0;
    const int DROP = -1;
    const int CONNECTED = 1;
    vector<int> flags(connectedMap.size() + 1, DEFAULT);

    for (auto iter = connectedMap.begin(); iter != connectedMap.end(); ++iter)
    {
        if (flags[iter->first] == DROP) continue;

        for (int k = 0; k < K_MAX; ++k)
        {

        }
    }
}

vector<pair<vector3, vector3>> DeformationGraph::GetEdges()
{
	vector<pair<vector3, vector3>> res;

	return res;
}

void DeformationGraph::CalConnectedMap()
{
    for (int i = 0; i < mesh->numtriangles; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            connectedMap.insert(pair<int, set<int>>(mesh->triangles[i].vindices[j], set<int>()));

            connectedMap[mesh->triangles[i].vindices[j]].insert(mesh->triangles[i].vindices[(j + 1) % 3]);
            connectedMap[mesh->triangles[i].vindices[j]].insert(mesh->triangles[i].vindices[(j + 2) % 3]);
        }
    }
}