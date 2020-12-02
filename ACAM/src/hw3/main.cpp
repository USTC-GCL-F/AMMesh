#include <iostream>
#include <fstream>
#include <sstream>
#include "PolyMesh\IOManager.h"
//#include "Algorithm.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

using namespace acamcad;
using namespace polymesh;

void main(int argc, char** argv)
{
	//alien.obj 1 v.txt path.txt
	if (argc != 2)
	{
		std::cout << "========== Hw3 Usage  ==========\n";
		std::cout << std::endl;
		std::cout << "hw3.exe	alien.obj\n";
		std::cout << std::endl;
		std::cout << "=================================================\n";
		return;
	}
	//¶ÁÈëÍø¸ñ
	std::string mesh_path = argv[1];
	PolyMesh* mesh = new PolyMesh();
	loadMesh(mesh_path, mesh);
	std::cout << mesh->numVertices();
	mesh->updateMeshNormal();
	Eigen::MatrixXd NewNormal(mesh->numPolygons(), 3);
	Eigen::VectorXd FaceArea(mesh->numPolygons());
	for (FaceIter fh = mesh->polyfaces_begin(); fh < mesh->polyfaces_end(); fh++)
	{
		std::vector<MVert*> P;
		for (FaceVertexIter vv_it = mesh->fv_iter(*fh); vv_it.isValid(); ++vv_it)
		{
			P.push_back(*vv_it);
		}
		auto e12 = P[1]->position() - P[0]->position();
		auto e13 = P[2]->position() - P[0]->position();
		double area = cross(e12, e13).norm() * 0.5;
		FaceArea[(*fh)->index()] = area;
	}

	for (auto fh : mesh->polygons())
	{

	}



}
