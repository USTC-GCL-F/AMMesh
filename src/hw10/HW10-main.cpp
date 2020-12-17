#include <PolyMesh\IOManager.h>
#include<complex>
#include<Eigen/Eigen>
#include<fstream>
#include<math.h>
#include<string>
using namespace acamcad;
using namespace polymesh;

#define  PI 3.14159265358979323846

void calculateMeshFaceBase(PolyMesh* mesh, std::vector<MVector3>& f_base)
{
	f_base.resize(mesh->numPolygons() * 2);
	for (FaceIter f_it = mesh->polyfaces_begin(), f_end = mesh->polyfaces_end(); f_it != f_end; ++f_it) 
	{
		assert((*f_it)->PolyNum() >= 3);

		int id = (*f_it)->index();
		FaceVertexIter  fv_it = mesh->fv_iter(*f_it);

		MVert* v1, * v2, * v3;
		MVector3 p1, p2, p3;
		v1 = *fv_it; p1 = v1->position(); ++fv_it;
		v2 = *fv_it; p2 = v2->position(); ++fv_it;
		v3 = *fv_it; p3 = v3->position();

		MVector3 no = cross((p3 - p1), (p2 - p1));
		no.normalize();
		f_base[id * 2] = (p2 - p1).normalized();
		f_base[id * 2 + 1] = cross(no, f_base[id * 2]).normalized();
	}
}

void crossfieldCreator(PolyMesh* mesh, std::vector<int>& cons_id, std::vector<MVector3>& cons_vec, std::vector<MVector3>& crossfield)
{
	using namespace std;
	typedef complex<double> COMPLEX;

	int fnum = mesh->numPolygons();
	vector<int> status(fnum, 0);
	vector<COMPLEX> f_dir(fnum);

	crossfield.clear();
	crossfield.resize(fnum);
	vector<MVector3> f_base(fnum * 2);
	calculateMeshFaceBase(mesh, f_base);

	for (int i = 0; i < cons_id.size(); i++)
	{
		int fid = cons_id[i];
		status[fid] = 1;
		MVector3 cf = cons_vec[i].normalized();
		f_dir[fid] = COMPLEX(dot(cf, f_base[fid * 2]), dot(cf, f_base[fid * 2 + 1]));
	}

	vector<int> id2sln(fnum, -1);
	vector<int> sln2id(0);
	int count = 0;
	for (int i = 0; i < fnum; i++)
	{
		if (status[i] == 0)
		{
			sln2id.push_back(i);
			id2sln[i] = count;
			count++;
		}
	}

	Eigen::SimplicialLDLT<Eigen::SparseMatrix<COMPLEX>> slu;
	Eigen::SparseMatrix<COMPLEX> A;
	Eigen::VectorXcd b_pre(mesh->numEdges());
	Eigen::VectorXcd b;
	b_pre.setZero();
	vector<Eigen::Triplet<COMPLEX>> tris;

	count = 0;
	for (FaceIter f_it = mesh->polyfaces_begin(), f_end = mesh->polyfaces_end(); f_it != f_end; ++f_it) 
	{
		int id_f = (*f_it)->index();
		if (status[id_f] == 0) {
			COMPLEX sum = 0;
			for (FaceHalfEdgeIter fh_it = mesh->fhe_iter(*f_it); fh_it.isValid(); fh_it++) 
			{
				if (!mesh->isBoundary((*fh_it)->edge())) 
				{
					MVector3 p1, p2;
					MPolyFace* f_g;
					p1 = (*fh_it)->toVertex()->position();
					p2 = (*fh_it)->fromVertex()->position();
					f_g = (*fh_it)->pair()->polygon();
					int id_g = f_g->index();
					if (id_f < id_g)
					{
						MVector3 e = (p2 - p1).normalized();

						COMPLEX e_f = COMPLEX(dot(e, f_base[id_f * 2]), dot(e, f_base[id_f * 2 + 1]));
						COMPLEX e_g = COMPLEX(dot(e, f_base[id_g * 2]), dot(e, f_base[id_g * 2 + 1]));

						COMPLEX e_f_c_4 = pow(conj(e_f), 4);
						COMPLEX e_g_c_4 = pow(conj(e_g), 4);

						tris.push_back(Eigen::Triplet<COMPLEX>(count, id2sln[id_f], e_f_c_4));
						if (status[id_g] == 0) 
						{
							tris.push_back(Eigen::Triplet<COMPLEX>(count, id2sln[id_g], -e_g_c_4));
						}
						else
						{
							b_pre[count] += e_g_c_4 * f_dir[id_g];
						}
						count++;
					}
				}
			}
		}
	}
	A.resize(count, sln2id.size());
	b.resize(count);
	b = b_pre.head(count);
	A.setFromTriplets(tris.begin(), tris.end());
	Eigen::SparseMatrix<COMPLEX> AT = A.adjoint();
	slu.compute(AT * A);
	Eigen::VectorXcd x = slu.solve(AT * b);

	crossfield.resize(4 * fnum);
	for (int i = 0; i < fnum; i++)
	{
		if (status[i] == 0)
		{
			f_dir[i] = x(id2sln[i]);
		}
		double length = 1;
		double arg = std::arg(f_dir[i]) / 4;
		for (int j = 0; j < 4; j++)
		{
			crossfield[i * 4 + j] = f_base[i * 2] * length * cos(arg + j * PI / 2) + f_base[i * 2 + 1] * length * sin(arg + j * PI / 2);
		}
	}
}

void loadConstrains(const std::string& filename, std::vector<int>& cons_id, std::vector<MVector3>& cons_vec)
{
	std::fstream infile(filename.c_str(), std::ios_base::in);
	int num;
	infile >> num;
	cons_id.reserve(num);
	cons_vec.reserve(num);
	int tempi;
	for (int i = 0; i < num; i++)
	{
		infile >> tempi;
		cons_id.push_back(tempi);
		double p0, p1, p2;
		infile >> p0;	infile >> p1;	infile >> p2;
		cons_vec.push_back(MVector3(p0, p1, p2));
	}
	infile.close();
}

void writeCrossField(const std::string& filename, std::vector<MVector3>& crossfield)
{
	std::fstream ofile(filename.c_str(), std::ios_base::out);
	int num=crossfield.size();
	ofile << num << std::endl;
	for (int i = 0; i < num; i++)
	{
		ofile << crossfield[i][0] << " " << crossfield[i][1] << " " << crossfield[i][2] << std::endl;
	}
	ofile.close();
}

void main(int argc, char** argv)
{
	if (argc != 4)
	{
		std::cout << "========== Hw10 Usage  ==========\n";
		std::cout << std::endl;
		std::cout << "Input:	ACAM_mesh_HW10.exe	mesh.obj	constrains.txt	crossfield.txt\n";
		std::cout << std::endl;
		std::cout << "=================================================\n";
		return;
	}

	//读入网格
	std::string mesh_path = argv[1];
	PolyMesh* mesh = new PolyMesh();
	loadMesh(mesh_path, mesh);

	std::string input_constrains = argv[2];
	std::string output_crossfield = argv[3];

	//std::string input_mesh = "input_mesh.obj";
	//std::string input_constrains = "constrains.txt";
	//std::string output_crossfield = "crossfield.txt";

	//PolyMesh* mesh = new PolyMesh();
	std::vector<int> cons_id(0);
	std::vector<MVector3> cons_vec(0), crossfield(0);
	//loadMesh(input_mesh, mesh);
	loadConstrains(input_constrains, cons_id, cons_vec);
	crossfieldCreator(mesh,cons_id, cons_vec, crossfield);
	writeCrossField(output_crossfield, crossfield);
}