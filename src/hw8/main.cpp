#include "PolyMesh\IOManager.h"
#include <Eigen/Sparse>
#include <Eigen/SVD>

using namespace acamcad;
using namespace polymesh;
using namespace Eigen;

void MeshInterpolation(PolyMesh* source, PolyMesh* target, double delta_t)
{
	std::string command = "mkdir result ";
	system(command.c_str());

	int nv = source->numVertices();
	int nf = source->numPolygons();

	std::vector<Matrix2d> S(nf);
	std::vector<Vector3d> xx(nf);
	std::vector<Vector3d> yy(nf);
	std::vector<double> area(nf);
	std::vector<double> angle(nf);

	Matrix2d I = MatrixXd::Identity(2, 2);

	std::vector<std::vector<int>> v_id(nf);

	// Initial
	for (FaceIter f_it = source->polyfaces_begin(); f_it != source->polyfaces_end(); ++f_it)
	{
		int id = (*f_it)->index();
		MHalfedge* heh = (*f_it)->halfEdge();
		MVert* v0 = heh->fromVertex();
		MVert* v1 = heh->toVertex();
		MHalfedge* next_heh = heh->next();
		MVert* v2 = next_heh->toVertex();

		v_id[id].push_back(v0->index());
		v_id[id].push_back(v1->index());
		v_id[id].push_back(v2->index());

		area[id] = cross(v1->position() - v0->position(), v2->position()- v0->position()).norm() / 2.0;
		
		xx[id] = Vector3d(v2->position()[0] - v1->position()[0], v0->position()[0] - v2->position()[0], v1->position()[0] - v0->position()[0]);
		yy[id] = Vector3d(v1->position()[1] - v2->position()[1], v2->position()[1] - v0->position()[1], v0->position()[1] - v1->position()[1]);

		Vector3d u(target->vert(v0->index())->position()[0], target->vert(v1->index())->position()[0], target->vert(v2->index())->position()[0]);
		Vector3d v(target->vert(v0->index())->position()[1], target->vert(v1->index())->position()[1], target->vert(v2->index())->position()[1]);

		Matrix2d J;
		J << yy[id].dot(u) / (2 * area[id]), xx[id].dot(u) / (2 * area[id]), yy[id].dot(v) / (2 * area[id]), xx[id].dot(v) / (2 * area[id]);
		JacobiSVD<Matrix2d> svd(J, ComputeFullV | ComputeFullU);

		Matrix2d V = svd.matrixV();
		Matrix2d U = svd.matrixU();
		Matrix2d R = U * V.transpose();

		Matrix2d sigma;
		sigma(0, 1) = sigma(1, 0) = 0;
		sigma(0, 0) = svd.singularValues()[0];
		sigma(1, 1) = svd.singularValues()[1];
		S[id] = V * sigma * V.transpose();
		angle[id] = atan2(R(1, 0), R(1, 1));
	}

	// fix nv-1
	double u0 = source->vert(nv - 1)->position()[0];
	double v0 = source->vert(nv - 1)->position()[1];


	SparseMatrix<double> A(2 * nv - 2, 2 * nv - 2);
	std::vector<Triplet<double>>triplet;

	for (FaceIter f_it = source->polyfaces_begin(); f_it != source->polyfaces_end(); ++f_it)
	{
		int id = (*f_it)->index();
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				if ((v_id[id][i] == nv - 1) || (v_id[id][j] == nv - 1))
				{
					continue;
				}
				else
				{
					triplet.push_back(Triplet<double>(2 * v_id[id][i], 2 * v_id[id][j], yy[id][i] * yy[id][j] / (area[id] * area[id] * 2)));
					triplet.push_back(Triplet<double>(2 * v_id[id][i], 2 * v_id[id][j], xx[id][i] * xx[id][j] / (area[id] * area[id] * 2)));
					triplet.push_back(Triplet<double>(2 * v_id[id][i] + 1, 2 * v_id[id][j] + 1, yy[id][i] * yy[id][j] / (area[id] * area[id] * 2)));
					triplet.push_back(Triplet<double>(2 * v_id[id][i] + 1, 2 * v_id[id][j] + 1, xx[id][i] * xx[id][j] / (area[id] * area[id] * 2)));
				}
			}
		}
	}
	A.setFromTriplets(triplet.begin(), triplet.end());
	SparseLU<SparseMatrix<double>> solver;
	solver.analyzePattern(A);
	solver.factorize(A);

	double t = 0;
	while (t <= 1)
	{
		std::cout << "iter:" << t << std::endl;

		VectorXd b(2 * nv - 2);
		b.setZero();

		for (FaceIter f_it = source->polyfaces_begin(); f_it != source->polyfaces_end(); ++f_it)
		{
			int id = (*f_it)->index();
			Matrix2d A, R;
			double theta_t = angle[id] * t;

			R << cos(theta_t), -sin(theta_t), sin(theta_t), cos(theta_t);
			A = R * ((1.0 - t)*I + t * S[id]);
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					if ((v_id[id][i] == nv - 1) || (v_id[id][j] == nv - 1))
					{
						if ((v_id[id][i] != nv - 1) && (v_id[id][j] == nv - 1))
						{
							b[2 * v_id[id][i]] += -yy[id][i] * yy[id][j] * u0 / (area[id] * area[id] * 2);
							b[2 * v_id[id][i]] += -xx[id][i] * xx[id][j] * u0 / (area[id] * area[id] * 2);
							b[2 * v_id[id][i] + 1] += -yy[id][i] * yy[id][j] * v0 / (area[id] * area[id] * 2);
							b[2 * v_id[id][i] + 1] += -xx[id][i] * xx[id][j] * v0 / (area[id] * area[id] * 2);
						}
					}
				}
				if (v_id[id][i] != nv - 1)
				{
					b[2 * v_id[id][i]] += yy[id][i] * A(0, 0) / area[id];
					b[2 * v_id[id][i]] += xx[id][i] * A(0, 1) / area[id];
					b[2 * v_id[id][i] + 1] += yy[id][i] * A(1, 0) / area[id];
					b[2 * v_id[id][i] + 1] += xx[id][i] * A(1, 1) / area[id];
				}
			}
		}
		VectorXd result = solver.solve(b);
		for (VertexIter v_it = source->vertices_begin(); v_it != source->vertices_end(); ++v_it)
		{
			int id = (*v_it)->index();
			if (id != nv - 1)
			{
				(*v_it)->setPosition(result[2 * id], result[2 * id + 1], 0);
			}
		}
		
		std::string string_t = std::to_string(t);
		writeMesh("result\\" + string_t + ".obj", source);
		t += delta_t;
	}
}


void main(int argc, const char **argv)
{
	if (argc != 4)
	{
		std::cout << "========== Hw8 Usage  ==========\n";
		std::cout << std::endl;
		std::cout << "blablabla.exe	source.obj target.obj delta_t\n";
		std::cout << std::endl;
		std::cout << "=================================================\n";
		return;
	}
	std::string source_path = argv[1];
	std::string target_path = argv[2];
	std::string t_string = argv[3];

	PolyMesh* source_mesh = new PolyMesh();
	PolyMesh* target_mesh = new PolyMesh();
	loadMesh(source_path, source_mesh);
	loadMesh(target_path, target_mesh);
	double delta_t = atof(t_string.c_str());
	MeshInterpolation(source_mesh, target_mesh, delta_t);
	return;
}