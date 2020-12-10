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
        sample_idices.push_back(idx);
        samplingMesh->vertices[i * 3 + 0] = mesh->vertices[idx * 3 + 0];
        samplingMesh->vertices[i * 3 + 1] = mesh->vertices[idx * 3 + 1];
        samplingMesh->vertices[i * 3 + 2] = mesh->vertices[idx * 3 + 2];
    }

    cout << "Fit Sample Vertices:" << sample_idices.size() << endl;
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

void DeformationGraph::Run()
{
    UpdateOriginMesh();
}

void DeformationGraph::UpdateOriginMesh()
{
    for (int i = 0; i < sample_idices.size(); ++i)
    {
        mesh->vertices[3 * sample_idices[i] + 0] = samplingMesh->vertices[i * 3 + 3];
        mesh->vertices[3 * sample_idices[i] + 1] = samplingMesh->vertices[i * 3 + 4];
        mesh->vertices[3 * sample_idices[i] + 2] = samplingMesh->vertices[i * 3 + 5];
    }
}

void DeformationGraph::SetControlPoints(vector<vector<int>> controlPoints)
{
    control_points = controlPoints;
}

vector<int> DeformationGraph::GetSamplingIndices()
{
    return sample_idices;
}