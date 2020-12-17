#include <iostream>
#include <fstream>
#include <sstream>
#include "PolyMesh\IOManager.h"
#include "Algorithm.h"
#include <string>

using namespace acamcad;
using namespace polymesh;

void main(int argc, char** argv)
{
	//alien.obj 1 v.txt path.txt
	if (argc != 4)
	{
		std::cout << "========== Hw1 Usage  ==========\n";
		std::cout << std::endl;
		std::cout << "ShortestPath:	ACAM_mesh_HW1.exe	alien.obj	vertices.txt	path.txt\n";
		std::cout << "MST:		ACAM_mesh_HW1.exe	alien.obj	vertices.txt	path.txt\n";
		std::cout << std::endl;
		std::cout << "=================================================\n";
		return;
	}
	//????????
	std::string mesh_path = argv[1];
	PolyMesh* mesh = new PolyMesh();
	loadMesh(mesh_path, mesh);
	std::cout << mesh->numVertices();
	std::vector<int> landmarks;
	std::ifstream _in(argv[2]);
	std::string line;
	while (std::getline(_in,line))
	{
		std::stringstream ss;
		ss << line;
		int vid;
		ss >> vid;
		landmarks.push_back(vid);
	}
	std::vector<int> vertice, edges;
	if (landmarks.size() == 2)
	{
		std::vector<int> path;
		Dijkstra(*mesh, landmarks[0], landmarks[1], path);
		vertice.push_back(path[0]);
		for (int i = 0; i < path.size() - 1; i++)
		{
			vertice.push_back(path[i + 1]);
			edges.push_back(mesh->edgeBetween(mesh->vert(path[i]), mesh->vert(path[i+1]))->index());
		}
	}
	else if (landmarks.size()>2)
	{
		std::vector<std::vector<int>> path;
		Dijkstra_group(*mesh, landmarks, path);
		for (auto a : path)
		{
			edges.push_back(mesh->edgeBetween(mesh->vert(a[0]), mesh->vert(a[1]))->index());
			for (int i = 1; i < a.size() - 1; i++)
			{
				edges.push_back(mesh->edgeBetween(mesh->vert(a[i]), mesh->vert(a[i+1]))->index());
				vertice.push_back(a[i]);
			}
		}
		for (auto a : landmarks)
			vertice.push_back(a);
	}
	std::ofstream _out(argv[3]);
	_out << "VERTICES\n";
	for (auto a : vertice)
		_out << a << std::endl;
	_out << "EDGES\n";
	for (auto a : edges)
		_out << a << std::endl;
	_out.close();
}
