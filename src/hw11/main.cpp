#include "PolyMesh\IOManager.h"
#include "AABB_Tree\AABB_Tree.h"
#include <string>

using namespace acamcad;
using namespace polymesh;

AABB_Tree* abtree;
PolyMesh* mesh;

void split_long_edges(double high)
{
	int NE = mesh->numEdges();
	for (int i = 0; i < NE; i++)
	{		
		MEdge* e = mesh->edge(i);
		double len = e->length();
		if (len > high)	mesh->splitEdgeTriangle(e);
	}
}

void collapse_short_edges(double high, double low)
{
	int NE = mesh->numEdges();
	for (int i = NE-1; i >=0; i--)
	{
		if (i > mesh->numEdges() - 1)	continue;
		MEdge* e = mesh->edge(i);
		MHalfedge* he = e->halfEdge();
		MVert* p0 = he->fromVertex();
		MVert* p1 = he->toVertex();
		if (!mesh->is_collapse_ok(he)) continue;
		if (mesh->isBoundary(p0) || mesh->isBoundary(p1)) continue;
		double len = e->length();
		if (len < low)
		{
			bool is_collapse = true;
			for (VertexVertexIter vv_it = mesh->vv_iter(p0); vv_it.isValid(); ++vv_it)
			{
				MVert* vv = *vv_it;
				double len = (p1->position() - vv->position()).norm();
				if (len > high)
				{
					is_collapse = false;
					break;
				}
			}
			if (is_collapse)	mesh->collapseTriangle(he);
		}
	}
}

void equalize_valences()
{
	std::vector<int> target_valence;
	int deviation_pre, deviation_post;
	for (VertexIter v_it = mesh->vertices_begin(); v_it != mesh->vertices_end(); ++v_it)
	{
		if (mesh->isBoundary(*v_it))	target_valence.push_back(4);
		else target_valence.push_back(6);
	}
	for (EdgeIter e_it = mesh->edges_begin(); e_it != mesh->edges_end(); ++e_it)
	{
		if (mesh->isBoundary(*e_it) || !mesh->is_flip_ok_Triangle(*e_it)) continue;
		MHalfedge* he1 = (*e_it)->halfEdge();
		MVert* v1 = he1->fromVertex();
		MVert* v2 = he1->toVertex();

		MHalfedge* he2 = (*e_it)->halfEdge()->next();
		MVert* v3 = he2->toVertex();
		MHalfedge* he3 = (*e_it)->halfEdge()->pair()->next();
        MVert* v4 = he3->toVertex();
		
		deviation_pre = abs(int(mesh->valence(v1) - target_valence[v1->index()]))
			+ abs(int(mesh->valence(v2) - target_valence[v2->index()]))
			+ abs(int(mesh->valence(v3) - target_valence[v3->index()]))
			+ abs(int(mesh->valence(v4) - target_valence[v4->index()]));
		deviation_post = abs(int(mesh->valence(v1) - 1 - target_valence[v1->index()]))
			+ abs(int(mesh->valence(v2) - 1 - target_valence[v2->index()]))
			+ abs(int(mesh->valence(v3) + 1 - target_valence[v3->index()]))
			+ abs(int(mesh->valence(v4) + 1 - target_valence[v4->index()]));
		if (deviation_pre> deviation_post) mesh->flipEdgeTriangle(*e_it);
	}
}

void tangential_relaxation()
{
	mesh->updateMeshNormal();
	for (VertexIter v_it = mesh->vertices_begin(); v_it != mesh->vertices_end(); ++v_it)
	{
		if (mesh->isBoundary(*v_it)) continue;
		double count = 0.0;
		MPoint3 p = (*v_it)->position();
		MPoint3 q(0.0, 0.0, 0.0);
		for (VertexVertexIter vv_it = mesh->vv_iter(*v_it); vv_it.isValid(); ++vv_it)
		{
			q += (*vv_it)->position();
			++count;
		}
		q /= count;
		MVector3 n = (*v_it)->normal();
		n.normalize();
		(*v_it)->setPosition(q + (n.dot(p-q)) * n);
	}
}

void get_AABB_tree()
{
	std::vector<Vector3f> point_set;
	point_set.clear();
	for (FaceIter f_it = mesh->polyfaces_begin(); f_it != mesh->polyfaces_end(); ++f_it)
	{
		for (FaceVertexIter fv_it = mesh->fv_iter(*f_it); fv_it.isValid(); ++fv_it)
		{
			MVert* fv = *fv_it;
			Vector3f p;
			p[0] = float(fv->x());
			p[1] = float(fv->y());
			p[2] = float(fv->z());
			point_set.push_back(p);
		}
	}
	abtree = new AABB_Tree(point_set);
}

void project_to_surface()
{
	for (VertexIter v_it = mesh->vertices_begin(); v_it != mesh->vertices_end(); ++v_it)
	{
		Vector3f p;
		p[0] = float((*v_it)->x());
		p[1] = float((*v_it)->y());
		p[2] = float((*v_it)->z());
		Vector3f ab_nearst_point;
		abtree->findNearstPoint(p, ab_nearst_point);
		MPoint3 new_point;
		new_point[0] = double(ab_nearst_point[0]);
		new_point[1] = double(ab_nearst_point[1]);
		new_point[2] = double(ab_nearst_point[2]);
		(*v_it)->setPosition(new_point);
	}
}

double calculateTargetEdgeLength()
{

	double target_edge_length = 0.0;
	std::cout << mesh->numEdges() << std::endl;
	for (EdgeIter e_it = mesh->edges_begin(); e_it != mesh->edges_end(); ++e_it)
	{
		target_edge_length += (*e_it)->length();
	}
	target_edge_length /= mesh->numEdges();
	return target_edge_length;
}

void main(int argc, char** argv)
{
	if (argc != 3)
	{
		std::cout << "========== Hw11 Usage  ==========\n";
		std::cout << std::endl;
		std::cout << "Input:	ACAM_mesh_HW11.exe	input_mesh.obj	output_mesh.obj\n";
		std::cout << std::endl;
		std::cout << "=================================================\n";
		return;
	}

	mesh = new PolyMesh();
	//¶ÁÈëÍø¸ñ
	std::string mesh_path = argv[1];
	loadMesh(mesh_path, mesh);

	std::string out_path = argv[2];

	//mesh load an write , now only support obj/off
	//loadMesh("small_bunny.obj", mesh);
	double target_edge_length, high, low;
	target_edge_length = calculateTargetEdgeLength()/2.0;
	high = 4.0 / 3.0 * target_edge_length;
	low = 4.0 / 5.0 * target_edge_length;
	get_AABB_tree();
	for (int i = 0; i < 10; i++)
	{
		split_long_edges(high);
		collapse_short_edges(high, low);
		equalize_valences();
		tangential_relaxation();
		project_to_surface();
	}
	writeMesh(out_path, mesh);
	
}


