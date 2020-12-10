#include "PolyMesh\IOManager.h"
#include "Eigen/Dense"
#include <queue>
#include <map>
#include <time.h>

#define lambda 1e3

using namespace acamcad;
using namespace polymesh;

struct Edge_priority
{
	double cost;
	MEdge* eh;
	int state;
	MPoint3 NewPoint;
};

struct cmp
{
	bool operator()(const Edge_priority &a, const Edge_priority &b) 
	{
		return a.cost > b.cost;
	}
};

std::priority_queue<Edge_priority, std::vector<Edge_priority>, cmp> Cost;
std::map<MVert*, Eigen::Matrix4d> Qv;
std::map<MEdge*, int> State;


void cal_Q(MVert* vh, PolyMesh* mesh)
{
	MVector3 p_vh = vh->position();
	Eigen::Matrix4d Q_temp;
	Q_temp.setZero();
	MVector3 face_nor;
	double a, b, c, d;
	Eigen::Matrix<double, 4, 1> p;
	if (!mesh->isBoundary(vh))
	{
		for (auto vf_it = mesh->vf_iter(vh); vf_it.isValid(); ++vf_it)
		{
			face_nor = (*vf_it)->normal();
			a = face_nor[0], b = face_nor[1], c = face_nor[2], d = -dot(face_nor, p_vh);
			p = { a,b,c,d };
			Q_temp += p * p.transpose();
		}
		Qv[vh] = Q_temp;
	}
	else
	{
		for (auto vf_it = mesh->vf_iter(vh); vf_it.isValid(); ++vf_it)
		{
			face_nor = (*vf_it)->normal();
			a = face_nor[0], b = face_nor[1], c = face_nor[2], d = -dot(face_nor, p_vh);
			p = { a,b,c,d };
			Q_temp += p * p.transpose();
			for (auto fhh_it = mesh->fhe_iter(*vf_it); fhh_it.isValid(); ++fhh_it)
			{
				if ((*fhh_it)->isBoundary())
				{
					MVector3 vir_face_nor = (cross((*fhh_it)->pair()->tangent(), face_nor)).normalized();
					a = vir_face_nor[0], b = vir_face_nor[1], c = vir_face_nor[2], d = -dot(vir_face_nor, p_vh);
					p = { a,b,c,d };
					Q_temp += p * p.transpose();
					break;
				}
			}
		}
		/*for (auto vhh_it = mesh->voh_iter(vh); vhh_it.isValid(); ++vhh_it)
		{
			if (!(*vhh_it)->isBoundary())
			{
				MPolyFace* vf = (*vhh_it)->polygon();
				MVector3 face_nor = vf->normal();
				double a = face_nor[0], b = face_nor[1], c = face_nor[2], d = -dot(face_nor, p_vh);
				Eigen::Matrix<double, 4, 1> p = { a,b,c,d };
				Q_temp += p * p.transpose();
			}
			else
			{
				MHalfedge* hh_boundary = *vhh_it;
				MHalfedge* prev_hh_boundary = hh_boundary->prev();
				MPolyFace* face_hh = hh_boundary->pair()->polygon(), *face_prev_hh = prev_hh_boundary->pair()->polygon();
				MVector3 face_hh_nor = face_hh->normal(), face_prev_hh_nor = face_prev_hh->normal();
				MVector3 vir_face_hh_nor = cross(hh_boundary->tangent(), face_hh_nor).normalized(), vir_face_prev_hh_nor = cross(prev_hh_boundary->tangent(), face_prev_hh_nor).normalized();
				double a = vir_face_hh_nor[0], b = vir_face_hh_nor[1], c = vir_face_hh_nor[2], d = -dot(vir_face_hh_nor, p_vh);
				Eigen::Matrix<double, 4, 1> p = { a,b,c,d };
				Q_temp += lambda * (p * p.transpose());
				a = vir_face_prev_hh_nor[0], b = vir_face_prev_hh_nor[1], c = vir_face_prev_hh_nor[2], d = -dot(vir_face_prev_hh_nor, p_vh);
				p = { a,b,c,d };
				Q_temp += lambda * (p * p.transpose());
			}
		}*/
	}
}

void cal_Cost(MEdge* eh)
{
	Edge_priority temp;
	MHalfedge* hh = eh->halfEdge();
	MVert* v_from = hh->fromVertex(), *v_to = hh->toVertex();
	Eigen::Matrix4d Q_plus = Qv[v_from] + Qv[v_to], Q_solve = Q_plus;
	Q_solve(3, 0) = 0.0, Q_solve(3, 1) = 0.0, Q_solve(3, 2) = 0.0, Q_solve(3, 3) = 1.0;
	MPoint3 new_point;
	Eigen::Vector4d new_vec;
	if (Q_solve.determinant() == 0)
	{
		MVector3 temp = 0.5*(v_from->position() + v_to->position());
		new_point = { temp[0], temp[1], temp[2] };
		new_vec = { new_point[0], new_point[1], new_point[2], 1.0 };
	}
	else
	{
		Eigen::Vector4d temp = { 0.0,0.0,0.0,1.0 };
		new_vec = Q_solve.inverse()*temp;
		new_point = { new_vec[0], new_vec[1], new_vec[2] };
	}
	temp.cost = new_vec.transpose()*Q_plus*new_vec;
	temp.eh = eh;
	temp.state = State[eh];
	temp.NewPoint = new_point;
	Cost.push(temp);
}

void update_Q_and_Cost(MVert* vh, PolyMesh* mesh)
{
	cal_Q(vh, mesh);
	for (auto vv_it = mesh->vv_iter(vh); vv_it.isValid(); ++vv_it)
	{
		cal_Q(*vv_it, mesh);
	}
	for (auto ve_it = mesh->ve_iter(vh); ve_it.isValid(); ++ve_it)
	{
		State[*ve_it]++;
		cal_Cost(*ve_it);
	}
}

bool QEM_collapse(const Edge_priority &temp_edge, PolyMesh* mesh)
{
	bool is_collapse = false;
	MEdge* eh = temp_edge.eh;
	MHalfedge* hh = eh->halfEdge(), *hh_oppo = hh->pair();
	MVert *v_from = hh->fromVertex(), *v_to = hh->toVertex();
	MVert* vh;

	if (mesh->is_collapse_ok(hh))
	{
		v_to->setPosition(temp_edge.NewPoint);
		mesh->collapse(hh);
		is_collapse = true;
		vh = v_to;
	}
	else if (mesh->is_collapse_ok(hh_oppo))
	{
		v_from->setPosition(temp_edge.NewPoint);
		mesh->collapse(hh_oppo);
		is_collapse = true;
		vh = v_from;
	}
	if (is_collapse)
	{
		update_Q_and_Cost(vh, mesh);
	}
	return is_collapse;
}

void QEM(PolyMesh* mesh)
{
	if (mesh->numVertices() == 3)
		return;
	/// Initial Qv
	for (MVert* vh : mesh->vertices())
	{
		MVector3 p_vh = vh->position();
		Eigen::Matrix4d Q_temp;
		Q_temp.setZero();
		for (auto vf_it = mesh->vf_iter(vh); vf_it.isValid(); ++vf_it)
		{
			MVector3 face_nor = (*vf_it)->normal();
			double a = face_nor[0], b = face_nor[1], c = face_nor[2], d = -dot(face_nor, p_vh);
			Eigen::Matrix<double, 4, 1> p = { a,b,c,d };
			Q_temp += p * p.transpose();
		}
		Qv.insert(std::make_pair(vh, Q_temp));
	}

	///Initial Cost
	for (MEdge* eh : mesh->edges())
	{
		State.insert(std::make_pair(eh, 0));
		cal_Cost(eh);
	}
		
	/// simplification
	int N_V = mesh->numVertices();
	int target_num = std::min((int)(0.5*N_V), 1000);
	while(N_V > target_num)
	{
		std::cout << N_V << std::endl;
		Edge_priority temp_edge = Cost.top();
		Cost.pop();
		MEdge* eh = temp_edge.eh;
		if (temp_edge.state == State[eh])
		{
			if (eh->index() != -1)
			{
				if (QEM_collapse(temp_edge, mesh))
				{
					N_V--;
				}
			}
		}
	}
}

int main()
{
	// read input mesh
	PolyMesh* mesh = new PolyMesh();
	loadMesh("cat_open.obj", mesh);

	/*MHalfedge* bound_hh;
	for (MHalfedge* hh : mesh->halfEdges())
	{
		if (hh->isBoundary())
			bound_hh = hh;
	}
	MHalfedge* temp_hh = bound_hh->prev();
	int ii = 2;
	while (temp_hh != bound_hh)
	{
		std::cout << ii << std::endl;
		temp_hh = temp_hh->prev();
		ii++;
	}*/

	clock_t start, end;
	std::cout << "Begin QEM" << std::endl;
	start = clock();
	QEM(mesh);
	end = clock();
	std::cout << "time: " << (double)(end - start) / CLOCKS_PER_SEC << "s" << std::endl;
	writeMesh("cat_open_QEM.obj", mesh);
}