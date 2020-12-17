#pragma once

#include "../Math/Massage.h"

#include "PolyMesh_Base.h"
#include "PolyMeshIterators.h"
#include "MemoryPool.h"


namespace acamcad {
namespace polymesh {

typedef std::vector<MVert*>::iterator VertexIter;
typedef std::vector<MEdge*>::iterator EdgeIter;
typedef std::vector<MHalfedge*>::iterator HalfEdgeIter;
typedef std::vector<MPolyFace*>::iterator FaceIter;
typedef std::vector<MVert*>::const_iterator CVertexIter;
typedef std::vector<MEdge*>::const_iterator CEdgeIter;
typedef std::vector<MHalfedge*>::const_iterator CHalfEdgeIter;
typedef std::vector<MPolyFace*>::const_iterator CFaceIter;

const double MehsThreshold = 0.0001;

class PolyMesh
{
private:

	std::vector<MHalfedge*> half_edges_;
	std::vector<MVert*>		vertices_;
	std::vector<MEdge*>		edges_;
	std::vector<MPolyFace*>	polygons_;

	MemoryPool<MHalfedge> poolHE;
	MemoryPool<MVert> poolV;
	MemoryPool<MEdge> poolE;
	MemoryPool<MPolyFace> poolP;

	std::vector<std::string> texture_name;

public:
	PolyMesh() {};
	~PolyMesh();

public:
//=====================================================================
// Get Method
//=====================================================================
	size_t numVertices() const { return vertices_.size(); }
	size_t numEdges() const { return edges_.size(); }
	size_t numPolygons() const { return polygons_.size(); }
	size_t numHalfEdges() const { return half_edges_.size(); }

	const std::vector<MVert*>& vertices() const { return vertices_; }
	const std::vector<MEdge*>& edges() const { return edges_; }
	const std::vector<MPolyFace*>& polyfaces() const { return polygons_; }
	const std::vector<MHalfedge*>& halfEdges() const { return half_edges_; }

	std::vector<MVert*> vertices() { return vertices_; }
	std::vector<MEdge*> edges() { return edges_; }
	std::vector<MPolyFace*> polyfaces() { return polygons_; }
	std::vector<MHalfedge*> halfEdges() { return half_edges_; }

	/// ID should be used cautiously in topology changes
	/// The newly added unit will be at the end of the list
	/// The deleted part will exchange the deleted part with the last part
	MVert* vert(const size_t id) { return (id < numVertices() ? vertices_[id] : nullptr); }
	MEdge* edge(const size_t id) { return (id < numEdges() ? edges_[id] : nullptr); }
	MPolyFace* polyface(const size_t id) { return (id < numPolygons() ? polygons_[id] : nullptr); }
	MHalfedge* halfedge(const size_t id) { return (id < numHalfEdges() ? half_edges_[id] : nullptr); }

	const MVert* vert(const size_t id) const { return (id < numVertices() ? vertices_[id] : nullptr); }
	const MEdge* edge(const size_t id) const { return (id < numEdges() ? edges_[id] : nullptr); }
	const MPolyFace* polyface(const size_t id) const { return (id < numPolygons() ? polygons_[id] : nullptr); }
	const MHalfedge* halfedge(const size_t id) const { return (id < numHalfEdges() ? half_edges_[id] : nullptr); }

	bool isEmpty() const { return vertices_.empty(); }

//=====================================================================
// Traverse Method
//=====================================================================
	/// return the mesh one boundary vertices in order, the mesh may have many boundary , This function only search one boundary;
	std::vector<MVert*> boundaryVertices();	
	/// return all boundary vertices, every one is in order;
	//std::vector<std::vector<MVert*>> boundaryVerticesAll();

	bool isBoundary(MVert* vert) const;
	bool isBoundary(const MEdge* edge) const;
	bool isBoundary(const MHalfedge* halfedge) const;

	std::vector<MVert*>		vertAdjacentVertices(MVert* vert) const;
	std::vector<MEdge*>		vertAdjacentEdge(MVert* vert) const;
	std::vector<MHalfedge*> vertAdjacentHalfEdge(MVert* vert) const;
	std::vector<MPolyFace*> vertAdjacentPolygon(MVert* vert) const;

	std::vector<MPolyFace*> edgeAdjacentPolygon(MEdge* edge) const;
	std::vector<MPolyFace*> polygonAdjacentPolygon(MPolyFace* face) const;
	std::vector<MVert*>		polygonVertices(MPolyFace* face) const;
	std::vector<MHalfedge*> polygonHalfedges(MPolyFace* face) const;
	std::vector<MEdge*>		polygonEdges(MPolyFace* face) const;

	MHalfedge* edgeHalfEdge(MEdge* edge, unsigned int di);

	MEdge* edgeBetween(MVert* v0, MVert* v1);
	MHalfedge* halfedgeBetween(MVert* v0, MVert* v1);
	bool isConnected(MVert* v0, MVert* v1);

	bool isFaceContainsVertices(MPolyFace* face, MVert* vert);
	MHalfedge* faceHalfEdgeFromVert(MPolyFace* face, MVert* vert);

	size_t valence(MVert* vert) const;

	bool isIsolated(MVert* vert) const;
	bool isIsolated(MEdge* edge) const;
	bool isIsolated(MPolyFace* face) const;

	bool isTriangleMesh();

//=====================================================================
// Iterators
//=====================================================================
	VertexOHalfEdgeIter voh_iter(MVert* _v) const {
		return VertexOHalfEdgeIter(_v, this);
	}
	VertexVertexIter vv_iter(MVert* _v) const {
		return VertexVertexIter(_v, this);
	}
	VertexEdgeIter ve_iter(MVert* _v) const {
		return VertexEdgeIter(_v, this);
	}
	VertexFaceIter vf_iter(MVert* _v) const {
		return VertexFaceIter(_v, this);
	}

	FaceHalfEdgeIter fhe_iter(MPolyFace* _f) const {
		return FaceHalfEdgeIter(_f, this);
	}
	FaceVertexIter fv_iter(MPolyFace* _f) const {
		return FaceVertexIter(_f, this);
	}
	FaceEdgeIter fe_iter(MPolyFace* _f) const {
		return FaceEdgeIter(_f, this);
	}
	FaceFaceIter ff_iter(MPolyFace* _f) const {
		return FaceFaceIter(_f, this);
	}

	VertexIter vertices_begin() {
		return vertices_.begin();
	}
	VertexIter vertices_end() {
		return vertices_.end();
	}

	EdgeIter edges_begin() {
		return edges_.begin();
	}
	EdgeIter edges_end() {
		return edges_.end();
	}

	HalfEdgeIter halfedge_begin() {
		return half_edges_.begin();
	}
	HalfEdgeIter halfedge_end() {
		return half_edges_.end();
	}

	FaceIter polyfaces_begin() {
		return polygons_.begin();
	}
	FaceIter polyfaces_end() {
		return polygons_.end();
	}

	CVertexIter const_vertices_begin() const {
		return vertices_.cbegin();
	}
	CVertexIter const_vertices_end() const {
		return vertices_.cend();
	}

	CEdgeIter const_edges_begin() const  {
		return edges_.cbegin();
	}
	CEdgeIter const_edges_end() const  {
		return edges_.cend();
	}

	CHalfEdgeIter const_halfedge_begin() const  {
		return half_edges_.cbegin();
	}
	CHalfEdgeIter const_halfedge_end() const  {
		return half_edges_.cend();
	}

	CFaceIter const_polyfaces_begin() const  {
		return polygons_.cbegin();
	}
	CFaceIter const_polyfaces_end() const  {
		return polygons_.cend();
	}

//=====================================================================
// Base Topology
//=====================================================================
	void reserveMemory(size_t nv);
	void reserveMemory(size_t nv, size_t nf);
	void clear();

	MVert* newVertex();
	MEdge* newEdge();
	MEdge* newEdge(MVert* v1, MVert* v2);
	MHalfedge* newHelfEdge();
	MPolyFace* newPolyFace();

	MVert* addVertex(double x, double y, double z);
	MVert* addVertex(const MPoint3& point);

	/// Juat add an edge and two opposite halfedges. The two halfedge are connected end to end. 
	/// it will set the halfedge vertex but it will not set vert halfedge. 
	/// Be careful when using this function
	MEdge* addEdge(MVert* v_begin, MVert* v_end);

	/// add a face by given vertices, it will not detect duplicate face 
	MPolyFace* addPolyFace(std::vector<size_t>& v_loop_id);
	MPolyFace* addPolyFace(std::vector<MVert*>& v_loop);

	/// Just add a face to connect the given halfedges, requiring the input halfedges to be connected end to end
	/// Don't use this function yet
	MPolyFace* addPolyFace(std::vector<MHalfedge*>& he_loop);

	void deleteVertex(MVert* vert);
	void deleteEdges(MEdge* edge);
	void deletePolyFace(MPolyFace* face);

	size_t delete_isolated_vertices();
	size_t delete_isolated_edges();

	void deleteMultipleVerttex(std::vector<MVert*>& vert_list);
	void deleteMultipleEdge(std::vector<MEdge*> edge_list);
	void deleteMultiplePolyFace(std::vector<MPolyFace*> face_list);

//=====================================================================
// MeshBasedMethod
//=====================================================================
	/// the closet Point is use iter, it is very solw, do not use it now
	MVert* closestPoint(const MPoint3& p);
	double closestPoint(const MPoint3& p, MVert* close_v);

	/// reverse a face so that the normal direction of the facs is reversed.
    /// only reverses the inner halfedge of face, it will cause error in ordinary mesh
	void reverse_face(MPolyFace* face);

	/// Reverse a isolated face, also reverse it's boundary halfedge，return false for non-isolated
	bool reverseIsolatedFace(MPolyFace* face);

	/// Reverse all halfedge of a mesh
	void reverse_mesh();

	/// update all vertices normal use face normal. before use it, make sure you have set face normal.
	void updateVerticesNormal(bool is_update_face = false);
	/// update all faces normal
	void updateFacesNormal();
	void updateMeshNormal();
//=====================================================================
// Low Level API
//=====================================================================
	/// Insert a point in the middle of an edge, only split the edge into two edges
	MVert* splitEdgeMakeVertex(MEdge* edge);

	/// Delete the vertex with a degree of 2 that is not a boundary, leaving an edge connecting the two neighbors of the vertex.
	MEdge* jointEdgeRemoveVertex(MVert* vert);

	/// connect two unconnected vertices v0/v1 in face, creat a edge divide face into two polygonal faces
	MEdge* splitFaceMakeEdge(MPolyFace* face, MVert* v0, MVert* v1);

	/// split a polygon completely into triangles
	void splitNGonTriangle(MPolyFace* face);

	/// split a quadrilateral into two triangles, 
	/// try to avoid grids with poor angles when the isquality setting
	MEdge* splitQuadrilateralTriangle(MPolyFace* face, bool isqulaty = false);

	/// 将输入的顶点连接输入面的其他顶点，将面分成多个三角形，对于非凸情况，会有翻转，不会检测点在不在多边形上。
	void SplitFaceWithSingleFaceVertex(MPolyFace* face, MVert* v_new);

	/// 将一条边split，然后将新生成的顶点连接相邻两个面的其他顶点，将相邻两个面分成多个三角形，对于非凸情况，会有翻转
	MVert* splitEdgeSplitPolygon(MEdge* edge);

	bool is_collapse_ok(MHalfedge* he);
	/**
	* the collapse function for polygon mesh, the from vert will be delete
	*/
	void collapse(MHalfedge* he);


	/// Combine two adjacent faces of an edge into one face
	/// The input edge is the only intersection edge of two adjacent faces. No vertices will be deleted
	/// +-----+        +-----+
	/// |     |        |     |
	/// +-----+   ->   +     +
	/// |     |        |     |
	/// +-----+        +-----+
	MPolyFace* jointFaceRemoveEdge(MEdge* edge);


	/// merge edge: 将只由顶点分割的两条边合并为1条-----调用jointEdgeRemoveVertex
	MEdge* mergeEdge(MEdge* edge0, MEdge* edge1);
	/// 将两个面合并成一个面-----调用jointFaceRemoveEdge
	MPolyFace* mergeFace(MPolyFace* f0, MPolyFace* f1);


	/// 顶点合并，只要将顶点所有的边都合并到一起，这个点自然会删掉，关键是多个点怎么焊接
	/// 主要是调用weldEdge，现在的可能还有问题
	bool weldVertex(MVert* v_ori, MVert* v_tar, double tolerance = 1e-2);	

	///将两条边焊接成一条边，这两条边都是顶点且端点靠近，删除e0和e0的不与e1公用的端点
	bool weldEdge(MEdge* e0, MEdge* e1, double tolerance = 1e-2);	//it will deleat e0;


	/// 顶点分离，将一个不是流形的点分离，添加流形子区域数目的顶点-1。
	bool separateSingleVert(MVert* vert);
	bool separateSingleVert(MVert* vert, std::vector<MVert*>& new_vert_list);

	/// separate a single edge, it will add a new edge
	bool separateSingleEdge(MEdge* edge);
	bool separateSingleEdge(MEdge* edge, std::vector<MEdge*>& new_edge_list);


//=====================================================================
// Triangle Mesh API 
//=====================================================================
	bool is_flip_ok_Triangle(MEdge* edge);

	void flipEdgeTriangle(MEdge* edge);

	bool is_collapse_ok_Triangle(MHalfedge* he);
	/**
	* the from vert will be delete
	*/
	void collapseTriangle(MHalfedge* he);

	MVert* splitEdgeTriangle(MEdge* edge);

//=====================================================================
// Calculating location API
//=====================================================================
	MPoint3 calculatFaceCenter(MPolyFace* face);
	MPoint3 calculatEdgeCenter(MEdge* edge);

//=====================================================================
// Texture processing, rendering API
//=====================================================================
	void add_texture_information(int id, std::string name);
	size_t getFaceTexcoords(std::vector<Texcoord>& hehandles);

private:
	void deletePolyFace_fromMesh(MPolyFace* face);
	void deleteHalfEdge_fromMesh(MHalfedge* halfedge);
	void deleteVertex_fromMesh(MVert* vert);
	void deleteEdge_fromMesh(MEdge* edge);
	
	void collpaseEdge(MHalfedge* he);
	void collpaseLoop(MHalfedge* he);

	bool mesh_manifold_check();

//=====================================================================
//=====================================================================
	/// Combine two adjacent faces of an edge into one face
	/// This function does not detect whether there is a problem with the input，Called by jointFaceRemoveEdge
	MPolyFace* jointFaceRemoveEdge(MPolyFace* f0, MPolyFace* f1, MEdge* edge);
};

}//namespace polymesh
}//namespaec acamcad