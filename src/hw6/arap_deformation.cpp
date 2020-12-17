// arap_deformation
#include <fstream>
#include"PolyMesh\IOManager.h"
#include"PolyMesh\PolyMesh.h"
#include<Eigen/SVD>
#include<Eigen/Dense>
#include<Eigen/Sparse>
#include <vector>
#include<set>
#include<iostream>
#include<math.h>
#include<algorithm>
#include<omp.h>
#include <fstream>
using namespace acamcad;
using namespace polymesh;
using namespace std;

PolyMesh mesh;

void calc_cot_weight(vector<double>& cots)
{
	cots.clear();
	cots.resize(mesh.numHalfEdges(), 0.);
	for (auto ithe = mesh.halfedge_begin(); ithe != mesh.halfedge_end(); ithe++)
	{
		if (mesh.isBoundary(*ithe))
			continue;
		auto v0=(*ithe)->fromVertex()->position();
		auto v1 = (*ithe)->toVertex()->position();
		auto v2 = (*ithe)->next()->toVertex()->position();
		auto e0 = v0 - v2;
		auto e1 = v1 - v2;
		double cotangle = dot(e0,e1) / cross(e0,e1).norm();
//		cots[ithe->idx()] = cotangle;
		cots[(*ithe)->index()] = 1.;
	}
}

void main(int argc, char** argv)
{
	if (argc != 4)
	{
		std::cout << "========== Hw6 Usage  ==========\n";
		std::cout << std::endl;
		std::cout << "Input:	ACAM_mesh_HW6.exe	[model]	[fix vertex id] [move vertex id and position]\n";  //fix vertex id: number + vertex id ;move vertex id and position: number + id position
		std::cout << "Input:	ACAM_mesh_HW6.exe	mesh.obj	fix_handle.txt	move_handle.txt\n";
		std::cout << std::endl;
		std::cout << "=================================================\n";
		return;
	}

	//¶ÁÈëÍø¸ñ
	std::string mesh_path = argv[1];
	loadMesh(mesh_path, &mesh);

	//loadMesh("Bunny_head.obj",&mesh);
	mesh.updateMeshNormal();
	int nf = mesh.numPolygons();
	int nv = mesh.numVertices();

	//position backup
	vector<MVector3> pos_mesh_ref;
	pos_mesh_ref.resize(nv);
	for (auto itv = mesh.vertices_begin(); itv !=mesh.vertices_end() ; itv++)
	{
		pos_mesh_ref[(*itv)->index()] = (*itv)->position();
	}

	vector<double> cots;
	calc_cot_weight(cots);
	

	//set fix handle 
	//set<int> handles_f = {12,505,381,712,296};
	set<int> handles_f;
	ifstream fix_handle(argv[2]);
	int fix_num;
	fix_handle >> fix_num;
	for (int i = 0; i < fix_num; i++)
	{
		int vertex_tmp;
		fix_handle >> vertex_tmp;
		handles_f.emplace(vertex_tmp);
	}

	//set move handle
	//vector<int> handles_m = {652};
	//vector<MVector3> handles_m_pos = { MVector3(0.05,0.2,0.05) };
	vector<int> handles_m;
	vector<MVector3> handles_m_pos;
	ifstream move_handle(argv[3]);
	int move_num;
	move_handle >> move_num;
	for (int i = 0; i < move_num; i++)
	{
		int vertex_tmp;
		move_handle >> vertex_tmp;
		double p1, p2, p3;
		move_handle >> p1;
		move_handle >> p2;
		move_handle >> p3;
		handles_m.push_back(vertex_tmp);
		handles_m_pos.push_back(MVector3(p1, p2, p3));
	}


	set<int> handles = handles_f;
	handles.insert(handles_m.begin(), handles_m.end());

	//calc cot-weight laplacian matrix
	vector<Eigen::Triplet<double>> trivec;
	for (int i = 0; i < nv; i++)
	{
		if (handles.count(i) > 0)
		{
			trivec.emplace_back(i, i, 1.);
			continue;
		}
		auto v_h = mesh.vert(i);
		double weight_sum = 0.;
		for (auto itvoh = mesh.voh_iter(v_h); itvoh.isValid(); ++itvoh)
		{
			auto v_to_h = (*itvoh)->toVertex();
			double weight_ = cots[(*itvoh)->index()] + cots[(*itvoh)->pair()->index()];
			
			weight_sum += weight_;
			trivec.emplace_back(i, v_to_h->index(), -weight_);
		}
		trivec.emplace_back(i, i, weight_sum);
	}
	Eigen::SparseMatrix<double> smat;
	smat.resize(nv, nv);
	smat.setFromTriplets(trivec.begin(),trivec.end());

	Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
	solver.compute(smat);
	
	Eigen::MatrixX3d uv;
	uv.resize(nv, 3);

	vector<Eigen::Matrix3d> Lts;
	Lts.resize(nv);
	Eigen::MatrixX3d b;
	b.resize(nv, 3);

	//local-global iteration
	for (int iter = 0; iter < 10; iter++)
	{
		//local calc Lt
#pragma omp parallel for
		for (int i = 0; i < nv; i++)
		{
			auto v_h = mesh.vert(i);
			Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
			for (auto itvoh = mesh.voh_iter(v_h); itvoh.isValid(); ++itvoh)
			{
				auto v_to_h = (*itvoh)->toVertex();
				auto e_ = pos_mesh_ref[i] - pos_mesh_ref[v_to_h->index()];
				auto ep_ = v_h->position() - v_to_h->position();
				double weight_ = cots[(*itvoh)->index()] + cots[(*itvoh)->pair()->index()];
				Eigen::Vector3d ep(ep_[0], ep_[1], ep_[2]);
				Eigen::Vector3d e(e_[0], e_[1], e_[2]);
				J += weight_ * (e*ep.transpose());
			}
			
			Eigen::JacobiSVD<Eigen::Matrix3d> svd(J, Eigen::ComputeFullU| Eigen::ComputeFullV);			
			Eigen::Matrix3d U = svd.matrixU();
			Eigen::Matrix3d V = svd.matrixV();

			Eigen::Matrix3d R = V * U.transpose();

			if (R.determinant() < 0)
			{
				U(0, 2) *= -1;
				U(1, 2) *= -1;
				U(2, 2) *= -1;
				R = V * U.transpose();
			}
			Lts[i] = R;
		}

		//global calc b
#pragma omp parallel for
		for (int i = 0; i < nv; i++)
		{
			auto v_h = mesh.vert(i);
			Eigen::Vector3d b_tmp(0., 0., 0.);
			for (auto itvoh = mesh.voh_iter(v_h); itvoh.isValid(); ++itvoh)
			{
				auto v_to_h = (*itvoh)->toVertex();
				auto ep_ = pos_mesh_ref[i] - pos_mesh_ref[v_to_h->index()];
				Eigen::Vector3d ep(ep_[0], ep_[1], ep_[2]);

				Eigen::Matrix3d JR = Lts[i] + Lts[v_to_h->index()];
				double weight_ = (cots[(*itvoh)->index()] + cots[(*itvoh)->pair()->index()]) / 2.0;
				b_tmp += weight_ * (JR*ep);

			}
			b(i, 0) = b_tmp[0];
			b(i, 1) = b_tmp[1];
			b(i, 2) = b_tmp[2];
		}

		//set handles
		for (int i:handles_f)
		{
			auto b_tmp = pos_mesh_ref[i];
			b(i, 0) = b_tmp[0];
			b(i, 1) = b_tmp[1];
			b(i, 2) = b_tmp[2];
		}
		
		for (int i = 0; i < handles_m.size(); i++)
		{
			auto b_tmp = handles_m_pos[i];
			b(handles_m[i], 0) = b_tmp[0];
			b(handles_m[i], 1) = b_tmp[1];
			b(handles_m[i], 2) = b_tmp[2];
		}

		//global solve
		uv.col(0) = solver.solve(b.col(0));
		uv.col(1) = solver.solve(b.col(1));
		uv.col(2) = solver.solve(b.col(2));

#pragma omp parallel for
		for (int i = 0; i < nv; i++)
		{
			auto v_h = mesh.vert(i);
			v_h->setPosition(uv(i, 0), uv(i, 1), uv(i, 2));
		}
	}

	writeMesh("deformation.obj", &mesh);

}