#pragma once
#include <PolyMesh/PolyMesh.h>
#include <PolyMesh/PolyMesh_Base.h>
#include <queue>

using namespace acamcad;
using namespace polymesh;

struct node
{
	int id;
	double dis;
	node(int id, double d)
	{
		this->id = id;
		this->dis = d;
	}
	bool operator<(const node& rhs) const { return dis > rhs.dis; }
};

struct PathInfo
{
	int s_p, e_p;
	double length;
	std::vector<int> path;
	PathInfo(int a, int b, double c)
	{
		s_p = a;
		e_p = b;
		length = c;
	}
	bool operator<(const PathInfo& rhs) const
	{
		return length > rhs.length;
	}
};

//计算数组lmk之间的点的路径
void Dijkstra_group(PolyMesh& Mesh, std::vector<int>& lmk, std::vector<std::vector<int>>&path);

//计算数组lmk之间的点的路径
void Dijkstra(PolyMesh& Mesh, int s_p, int e_p, std::vector<int>& path);