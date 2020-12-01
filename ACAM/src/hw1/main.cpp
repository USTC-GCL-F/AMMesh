#include <iostream>
#include <fstream>
#include <sstream>
#include "PolyMesh\IOManager.h"

using namespace acamcad;
using namespace polymesh;

// ‰»Î£∫
void main(int argc, char** argv)
{
	//alien.obj 1 v.txt path.txt
	if (argc != 4)
	{
		std::cout << "========== Usage  ==========\n";
		std::cout << std::endl;
		std::cout << "ShortestPath:	hw1.exe	alien.obj	vertices.txt	path.txt\n";
		std::cout << "MST:		hw1.exe	alien.obj	vertices.txt	path.txt\n";
		std::cout << std::endl;
		std::cout << "=================================================\n";
		return;
	}
	//∂¡»ÎÕ¯∏Ò
	std::string mesh_path = argv[1];
	PolyMesh* mesh = new PolyMesh();
	loadMesh(mesh_path, mesh);
	std::cout << mesh->numVertices();
	std::vector<int> landmarks;
	std::ifstream _in(argv[3]);
	std::string line;
	while (std::getline(_in,line))
	{
		std::stringstream ss;
		ss << line;
		int vid;
		ss >> vid;
		landmarks.push_back(vid);
	}




	

}