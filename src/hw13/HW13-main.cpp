#include "PolyMesh\IOManager.h"
#include<fstream>
#include<queue>
#include<sstream>
#include<time.h>
using namespace acamcad;
using namespace polymesh;

struct Tri
{
	int id;
	int tag;
	double weight;
	
	Tri( int id, int tag, double weight) :tag(tag), id(id), weight(weight) {};

	bool operator<(const Tri& a) const
	{
		return weight > a.weight;
	}
};

double cal_area(PolyMesh* mesh_,MHalfedge* he)
{
	MVector3 p0, p1, p2;
	p0 = he->fromVertex()->position();
	p1 = he->toVertex()->position();
	p2 = he->next()->toVertex()->position();
	return abs(norm(cross(p0 - p1, p2 - p1)));
}

void vsa_algorithm(PolyMesh* mesh_, int k_num, std::vector<int>& partition, std::vector<MVector3>& plane) {
	int f_num = mesh_->numPolygons();
	plane.resize(k_num);
	partition.clear();
	partition.resize(f_num, -1);

	srand(time(0));
	mesh_->updateMeshNormal();
	std::vector<int> seed_set(k_num);
	std::vector<bool> is_conq(f_num, false);
	for (int i = 0; i < k_num; )
	{
		int ran = rand() % f_num;
		if (partition[ran] == -1)
		{
			partition[ran] = i;
			plane[i] = mesh_->polyface(i)->normal();
			seed_set[i] = ran;
			i++;
		}
	}

	double last_energy = 0;
	for (int itnum = 0; ; itnum++)
	{
		std::vector<double> small_energy(k_num, 1e10);
		double total_energy = 0;
		for (int i = 0; i < f_num; i++)
		{
			if (partition[i] != -1)
			{
				double energy = norm(plane[partition[i]] - (mesh_->polyface(i)->normal()));
				total_energy += energy * cal_area(mesh_, mesh_->polyface(i)->halfEdge());
				if (energy < small_energy[partition[i]])
				{
					small_energy[partition[i]] = energy;
					seed_set[partition[i]] = i;
				}
			}
		}

		if (itnum > 0) std::cout << "Iteration " << itnum << " Total energy : " << total_energy << "\n";

		if (itnum > 1 && abs(total_energy - last_energy) / total_energy < 1e-4 || itnum>20) break;

		last_energy = total_energy;

		for (int i = 0; i < f_num; i++) is_conq[i] = false;
		std::priority_queue<Tri> que;
		for (int i = 0; i < k_num; i++)
		{
			partition[seed_set[i]] = i;
			is_conq[seed_set[i]] = true;
			for (FaceFaceIter ffit = mesh_->ff_iter(mesh_->polyface(seed_set[i])); ffit.isValid(); ffit++)
			{
				MPolyFace* face = (*ffit);
				double energy = norm(face->normal() - plane[i]);
				que.push(Tri(face->index(), i, energy));
			}
		}

		while (que.size() != 0)
		{
			int fid = que.top().id;
			int pid = que.top().tag;
			que.pop();
			if (!is_conq[fid])
			{
				partition[fid] = pid;
				is_conq[fid] = true;

				for (FaceFaceIter ffit = mesh_->ff_iter(mesh_->polyface(fid)); ffit.isValid(); ffit++)
				{
					if (!is_conq[(*ffit)->index()])
					{
						double energy = norm((*ffit)->normal() - plane[pid]);
						que.push(Tri((*ffit)->index(), pid, energy));
					}
				}
			}
		}

		std::vector<MVector3> new_normal(k_num, MVector3(0, 0, 0));
		for (int j = 0; j < f_num; j++)
		{
			double area = cal_area(mesh_, mesh_->polyface(j)->halfEdge());
			new_normal[partition[j]] += mesh_->polyface(j)->normal() * area;
		}
		for (int i = 0; i < k_num; i++)
		{
			new_normal[i].normalize();
			plane[i] = new_normal[i];
		}
	}
}
void writePartition(const std::string& filename, std::vector<int>& partition)
{
	std::fstream ofile(filename.c_str(), std::ios_base::out);
	ofile << partition.size()<<std::endl;
	for (int i = 0; i < partition.size(); i++)
	{
		ofile << partition[i] << std::endl;
	}
}

void main(int argc, char** argv)
{
	if (argc != 4)
	{
		std::cout << "========== Hw13 Usage  ==========\n";
		std::cout << std::endl;
		std::cout << "ACAM_mesh_HW13.exe [model] [partion_num] [out_partion]\n";
		std::cout << "ACAM_mesh_HW13.exe	mesh.obj 20 Partition.txt\n";
		std::cout << std::endl;
		std::cout << "=================================================\n";
		return;
	}
	//¶ÁÍø¸ñ
	std::string mesh_path = argv[1];
	std::stringstream ss;
	std::string cmd2 = argv[2];
	ss << cmd2;
	double K;
	ss >> K;
	std::string output_partition = argv[3];

	//std::string input_mesh = "input_mesh.obj";
	//double K = 20;
	//std::string output_partition = "Partition.txt";

	PolyMesh* mesh = new PolyMesh();
	std::vector<int> partition;
	std::vector<MVector3> plane(K);

	loadMesh(mesh_path, mesh);
	vsa_algorithm(mesh, K, partition, plane);
	writePartition(output_partition, partition);
}