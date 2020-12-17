#include "PolyMesh\IOManager.h"
#include <Eigen/Sparse>
#define pi 3.1415926

using namespace acamcad;
using namespace polymesh;
using namespace Eigen;

void BarycentricCoordinates(PolyMesh* mesh)
{
	int v_n = mesh->numVertices();
	std::vector<int> inner_id(v_n);
	int iter;
	int boundary_num, inner_num;

	std::vector<double> u(mesh->numVertices());
	std::vector<double> v(mesh->numVertices());

	iter = 0;
	for (VertexIter v_it = mesh->vertices_begin(); v_it != mesh->vertices_end(); ++v_it)
	{
		if (!mesh->isBoundary(*v_it))
		{
			inner_id[(*v_it)->index()] = iter;
			iter++;
		}
	}
	inner_num = iter;
	boundary_num = v_n - iter;

	MHalfedge* first_heh;
	for (HalfEdgeIter he_it = mesh->halfedge_begin(); he_it != mesh->halfedge_end(); ++he_it)
	{
		if (mesh->isBoundary(*he_it))
		{
			first_heh = *he_it;
			break;
		}
	}
	MHalfedge* iter_heh = first_heh->next();
	iter = 0;
	while (iter_heh != first_heh)
	{
		MVert* from_v = iter_heh->fromVertex();
		u[from_v->index()] = cos(double(2 * pi *iter / boundary_num));
		v[from_v->index()] = sin(double(2 * pi *iter / boundary_num));
		iter_heh = iter_heh->next();
		iter++;
	}
	u[first_heh->fromVertex()->index()] = cos(double(2 * pi *iter / boundary_num));
	v[first_heh->fromVertex()->index()] = sin(double(2 * pi *iter / boundary_num));

	SparseMatrix<double> weight(v_n, v_n);
	std::vector<Triplet<double>> triplet;

	for (FaceIter f_it = mesh->polyfaces_begin(); f_it != mesh->polyfaces_end(); ++f_it)
	{
		MHalfedge* heh = (*f_it)->halfEdge();
		MVert* v0 = heh->fromVertex();
		MVert* v1 = heh->toVertex();
		MHalfedge* next_heh = heh->next();
		MVert* v2 = next_heh->toVertex();

		double l2 = (v0->position() - v1->position()).norm();
		double l1 = (v0->position() - v2->position()).norm();
		double l0 = (v1->position() - v2->position()).norm();

		double angle = acos(dot(v2->position() - v0->position(), v1->position() - v0->position())/(l1*l2));
		triplet.push_back(Triplet<double>(v0->index(), v1->index(), tan(angle*0.5) / l2));
		triplet.push_back(Triplet<double>(v0->index(), v2->index(), tan(angle*0.5) / l1));

		angle = acos(dot(v0->position() - v1->position(), v2->position() - v1->position())/(l2*l0));
		triplet.push_back(Triplet<double>(v1->index(), v0->index(), tan(angle*0.5) / l2));
		triplet.push_back(Triplet<double>(v1->index(), v2->index(), tan(angle*0.5) / l0));

		angle = acos(dot(v0->position() - v2->position(), v1->position() - v2->position())/(l0*l1));
		triplet.push_back(Triplet<double>(v2->index(), v0->index(), tan(angle*0.5) / l1));
		triplet.push_back(Triplet<double>(v2->index(), v1->index(), tan(angle*0.5) / l0));
	}
	weight.setFromTriplets(triplet.begin(), triplet.end());

	SparseMatrix<double> inner(inner_num, inner_num);
	VectorXd ub(inner_num), vb(inner_num);

	triplet.clear();
	for (VertexIter v_it = mesh->vertices_begin(); v_it != mesh->vertices_end(); ++v_it)
	{
		if (!mesh->isBoundary(*v_it))
		{
			double tempu = 0;
			double tempv = 0;
			for (VertexVertexIter vv_it = mesh->vv_iter(*v_it); vv_it.isValid(); ++vv_it)
			{
				if (mesh->isBoundary(*vv_it))
				{
					tempu += u[(*vv_it)->index()] * weight.coeff((*v_it)->index(), (*vv_it)->index());
					tempv += v[(*vv_it)->index()] * weight.coeff((*v_it)->index(), (*vv_it)->index());
				}
				else
				{
					triplet.push_back(Triplet<double>(inner_id[(*v_it)->index()], inner_id[(*vv_it)->index()], -weight.coeff((*v_it)->index(), (*vv_it)->index())));
				}
				triplet.push_back(Triplet<double>(inner_id[(*v_it)->index()], inner_id[(*v_it)->index()], weight.coeff((*v_it)->index(), (*vv_it)->index())));
			}
			ub[inner_id[(*v_it)->index()]] = tempu;
			vb[inner_id[(*v_it)->index()]] = tempv;
		}
	}
	inner.setFromTriplets(triplet.begin(), triplet.end());
	SparseLU<SparseMatrix<double>> solver;
	solver.analyzePattern(inner);
	solver.factorize(inner);
	VectorXd result_u = solver.solve(ub);
	VectorXd result_v = solver.solve(vb);

	for (VertexIter v_it = mesh->vertices_begin(); v_it != mesh->vertices_end(); ++v_it)
	{
		if (!mesh->isBoundary(*v_it))
		{
			(*v_it)->setPosition(result_u[inner_id[(*v_it)->index()]], result_v[inner_id[(*v_it)->index()]], 0);
		}
		else
		{
			(*v_it)->setPosition(u[(*v_it)->index()], v[(*v_it)->index()], 0);
		}
	}
}


void main(int argc, const char **argv)
{
	if (argc != 2)
	{
		std::cout << "========== Hw7 Usage  ==========\n";
		std::cout << std::endl;
		std::cout << "blablabla.exe	test.obj\n";
		std::cout << std::endl;
		std::cout << "=================================================\n";
		return;
	}
	std::string mesh_path = argv[1];
	PolyMesh* mesh = new PolyMesh();
	loadMesh(mesh_path, mesh);
	BarycentricCoordinates(mesh);
	writeMesh("result_" + mesh_path, mesh);
	return;
}