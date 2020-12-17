#include "PolyMesh\IOManager.h"

using namespace acamcad;
using namespace polymesh;

void testBasicTriangle()
{
	PolyMesh* mesh = new PolyMesh();
	std::cout << "add v0, v1, v2" << std::endl;

	auto v0 = mesh->addVertex(0, 0, 0);
	auto v1 = mesh->addVertex(1, 0, 0);
	auto v2 = mesh->addVertex(0, 1, 0);

	std::cout << "add p0" << std::endl;
	std::vector<MVert*> vertlist; vertlist.push_back(v0); vertlist.push_back(v1); vertlist.push_back(v2);
	auto p0 = mesh->addPolyFace(vertlist);

	writeMesh("triangle.obj", mesh);
	delete(mesh);
}

void exampleSplitTriangle()
{
	std::cout
		<< "----------------------" << std::endl
		<< "    test SplitEdge    " << std::endl
		<< "----------------------" << std::endl;

	// v2---v3
	// | \	|
	// |  \ |
	// |   \|
	// v0---v1

	PolyMesh* mesh = new PolyMesh();

	MVert* v0 = mesh->addVertex(0, 0, 0);
	MVert* v1 = mesh->addVertex(1, 0, 0);
	MVert* v2 = mesh->addVertex(0, 1, 0);
	MVert* v3 = mesh->addVertex(1, 1, 0);

	std::vector<MVert*> vertlist; vertlist.push_back(v0); vertlist.push_back(v1); vertlist.push_back(v2);
	MPolyFace* p0 = mesh->addPolyFace(vertlist);
	vertlist.clear();
	vertlist.push_back(v1); vertlist.push_back(v3); vertlist.push_back(v2);
	MPolyFace* p1 = mesh->addPolyFace(vertlist);
	writeMesh("split_before.obj", mesh);

	MEdge* e12 = mesh->edgeBetween(v1, v2);	//get the edge between to vertex
	MVert * v4 = mesh->splitEdgeTriangle(e12);
	v4->setPosition(0.5, 0.6, 0.1);

	MEdge* e10 = mesh->edgeBetween(v0, v1);
	MVert* v5 = mesh->splitEdgeTriangle(e10);
	writeMesh("split_after.obj", mesh);
	delete(mesh);

}

void exampleFlipTriangle()
{
	std::cout
		<< "-----------------------" << std::endl
		<< "      test FlipEdge    " << std::endl
		<< "-----------------------" << std::endl;

	// v2---v3
	// | \	|
	// |  \ |
	// |   \|
	// v0---v1

	PolyMesh* mesh = new PolyMesh();

	MVert* v0 = mesh->addVertex(0, 0, 0);
	MVert* v1 = mesh->addVertex(1, 0, 0);
	MVert* v2 = mesh->addVertex(0, 1, 0);
	MVert* v3 = mesh->addVertex(1, 1, 0);

	std::vector<MVert*> vertlist; vertlist.push_back(v0); vertlist.push_back(v1); vertlist.push_back(v2);
	MPolyFace* p0 = mesh->addPolyFace(vertlist);
	vertlist.clear();
	vertlist.push_back(v1); vertlist.push_back(v3); vertlist.push_back(v2);
	MPolyFace* p1 = mesh->addPolyFace(vertlist);
    writeMesh("flip_before.obj", mesh);

	MEdge* e12 = mesh->edgeBetween(v1, v2);
	mesh->flipEdgeTriangle(e12);
	writeMesh("flip_after.obj", mesh);
	delete(mesh);
}

void exampleCollapseTriangle()
{
	// test collapse edge
	std::cout
		<< "-------------------------" << std::endl
		<< "    test CollapseEdge    " << std::endl
		<< "-------------------------" << std::endl;

	PolyMesh* mesh = new PolyMesh();
	
	MVert* v0 = mesh->addVertex(0, 0, 0);
	MVert* v1 = mesh->addVertex(1, 0, 0);
	MVert* v2 = mesh->addVertex(0, 1, 0);
	MVert* v3 = mesh->addVertex(1, 1, 0);

	std::vector<MVert*> vertlist; vertlist.push_back(v0); vertlist.push_back(v1); vertlist.push_back(v2);
	MPolyFace* p0 = mesh->addPolyFace(vertlist);
	vertlist.clear();
	vertlist.push_back(v1); vertlist.push_back(v3); vertlist.push_back(v2);
	MPolyFace* p1 = mesh->addPolyFace(vertlist);

	MEdge* e12 = mesh->edgeBetween(v1, v2);
	MVert* v4 = mesh->splitEdgeTriangle(e12);
	v4->setPosition(0.5, 0.6, 0.1);

	MEdge* e34 = mesh->edgeBetween(v4, v3);
	MVert* v5 = mesh->splitEdgeTriangle(e34);
	v5->setPosition(0.6, 0.6, 0.1);
	writeMesh("collapse_before.obj", mesh);

	//MHalfedge* he54 = mesh->halfedgeBetween(v5, v4);
	//mesh->collapseTriangle(he54);
	//Print(mesh);

	MHalfedge* he45 = mesh->halfedgeBetween(v4, v5);
	mesh->collapseTriangle(he45);
	writeMesh("collapse_after.obj", mesh);
	delete(mesh);
}

void exampleMeshIter(PolyMesh* mesh)
{
	std::cout << (mesh->isEmpty() ? "[not valid]" : "[valid]") << std::endl;
	std::cout << " V:" << mesh->numVertices() << std::endl;
	//iter all mesh vertices
	for (VertexIter v_it = mesh->vertices_begin(); v_it != mesh->vertices_end(); ++v_it)
	{
		MVert* v = *v_it;	//get the vertex pointer, you can check class MVert to know more
		MPoint3 p = (v)->position();	//get the vertex position
		mesh->isIsolated(v);	//check if the vertex is isolated
		mesh->isBoundary(v);	//check if the vertex is boundary

		for (VertexVertexIter vv_it = mesh->vv_iter(v); vv_it.isValid(); ++vv_it)
		{
			MVert* vv = *vv_it;
		}
		for (VertexOHalfEdgeIter voh_it = mesh->voh_iter(v); voh_it.isValid(); ++voh_it)
		{
			MHalfedge* voh = *voh_it;
		}
		for (VertexEdgeIter ve_it = mesh->ve_iter(v); ve_it.isValid(); ++ve_it)
		{
			MEdge* ve = *ve_it;
		}
		for (VertexFaceIter vf_it = mesh->vf_iter(v); vf_it.isValid(); ++vf_it)
		{
			MPolyFace* vf = *vf_it;
		}
	}

	std::cout << "HE:" << mesh->numHalfEdges() << std::endl;
	//iter all mesh halfedge
	for (HalfEdgeIter he_it = mesh->halfedge_begin(); he_it != mesh->halfedge_end(); ++he_it)
	{
		MHalfedge* he = *he_it;	//get the halfedge pointer, you can check class MHalfedge to know more
		MEdge* edge = he->edge();
	}

	std::cout << " E:" << mesh->numEdges() << std::endl;
	//iter all mesh edge
	for (EdgeIter e_it = mesh->edges_begin(); e_it != mesh->edges_end(); ++e_it)
	{
		MEdge* e = *e_it;	//get the edge pointer, you can check class MHalfedge to know more
		MHalfedge* he = e->halfEdge();
	}

	std::cout << " P:" << mesh->numPolygons() << std::endl;
	for (FaceIter f_it = mesh->polyfaces_begin(); f_it != mesh->polyfaces_end(); ++f_it)
	{
		MPolyFace* f = *f_it;	//get the face pointer, you can check class MHalfedge to know more

		for (FaceVertexIter fv_it = mesh->fv_iter(*f_it); fv_it.isValid(); ++fv_it)
		{
			MVert* fv = *fv_it;
		}
		for (FaceHalfEdgeIter fhe_it = mesh->fhe_iter(*f_it); fhe_it.isValid(); ++fhe_it)
		{
			MHalfedge* fv = *fhe_it;
		}
		for (FaceFaceIter ff_it = mesh->ff_iter(*f_it); ff_it.isValid(); ++ff_it)
		{
			MPolyFace* fv = *ff_it;
		}
	}
}

void exampleMeshTraverse(PolyMesh* mesh)
{
	//you can also use index to get the element
	//note that if you change the topology, the index of the element and the list of the element will also be change
	MVert* v = mesh->vert(0);
	MEdge* e = mesh->edge(0);
	MPolyFace* f = mesh->polyface(0);
	MHalfedge* he = mesh->halfedge(0);

	//you can use the function to return list of element, such as
	mesh->polygonVertices(f);

	//you can also use the adjacent function such as
	mesh->vertAdjacentEdge(v);
	mesh->vertAdjacentPolygon(v);
	mesh->polygonAdjacentPolygon(f);
}


int main()
{
	//create a basic mesh
	testBasicTriangle();

	//mesh load an write , now only support obj/off
	PolyMesh* mesh = new PolyMesh();
	loadMesh("triangle.obj", mesh);
	writeMesh("triangle.obj", mesh);
	//you can also use the IO option such as
	IOOptions opt;
	opt.vert_have_normal = true;	//it will load vertex normal
	loadMesh("triangle.obj", mesh, opt);
	writeMesh("triangle.obj", mesh, opt);

	//using iter to operating mesh;
	exampleMeshIter(mesh);
	//using index to operating mesh;
	exampleMeshTraverse(mesh);
	
	//example for split e triangle mesh edge
	exampleSplitTriangle();
	
	//example for flip e triangle mesh edge
	exampleFlipTriangle();
	
	//example for collapse e triangle mesh edge
	exampleCollapseTriangle();
}

//int main()
//{
//	PolyMesh* mesh = new PolyMesh();
//	loadMesh("collapse_before.obj", mesh);
//	//using iter to operating mesh;
////	exampleMeshIter(mesh);
////	exampleCollapseTriangle(mesh);
//	MEdge* e = mesh->edge(4);
//	MHalfedge* he = e->halfEdge()->pair();
////	std::cout << he->fromVertex()->index() << " " << he->toVertex()->index() << " " << mesh->is_collapse_ok(he) << std::endl;
////	mesh->collapseTriangle(he);
//	writeMesh("collapse_after.obj", mesh);
////	exampleMeshIter(mesh);
//}