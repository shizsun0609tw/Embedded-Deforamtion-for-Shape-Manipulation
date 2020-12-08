#include "deformationGraph.h"

DeformationGraph::DeformationGraph()
{

}

void DeformationGraph::Init(_GLMmodel* mesh, _GLMmodel* samplingMesh)
{
	this->mesh = mesh;
    this->samplingMesh = samplingMesh;

    CalConnectedMap();
    CalSamplingVertices();
}

void DeformationGraph::CalSamplingVertices()
{
    for (int i = 1; i <= samplingMesh->numvertices; ++i)
    {
        float min = INT_MAX;
        int idx = 0;
        for (int j = 1; j <= mesh->numvertices; ++j)
        {
            float temp = fabs(samplingMesh->vertices[i * 3 + 0] - mesh->vertices[j * 3 + 0])
                        + fabs(samplingMesh->vertices[i * 3 + 1] - mesh->vertices[j * 3 + 1])
                        + fabs(samplingMesh->vertices[i * 3 + 2] - mesh->vertices[j * 3 + 2]);
            if (temp < min)
            {
                min = temp;
                idx = j;
            }
        }
        sample_vertices.push_back(idx);
    }

    cout << "Fit Sample Vertices:" << sample_vertices.size() << endl;
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

vector<int> DeformationGraph::GetSamplingVertices()
{
    return sample_vertices;
}