#include "PolyMesh/PolyMesh.h"
#include "assert.h"
#include <sstream>

namespace acamcad{
namespace polymesh {

	PolyMesh::~PolyMesh()
	{
		clear();
		poolHE.Clear();
		poolV.Clear();
		poolE.Clear();
		poolP.Clear();
	}

//=====================================================================
// Traverse Method
//=====================================================================
	std::vector<MVert*> PolyMesh::boundaryVertices()
	{
		std::vector< std::vector<MVert*> > boundaries;
		for (MHalfedge* he : half_edges_)
		{
			if (he->isBoundary())
			{
				boundaries.push_back(std::vector<MVert*>());
				std::vector<MVert*>& boundary_part = boundaries[boundaries.size() - 1];
				{
					MHalfedge* cur = he;
					do
					{
						boundary_part.push_back(cur->toVertex());
						cur = cur->next();
					} while (cur != he);
				}
			}
		}

		return boundaries[0];
	}

	bool PolyMesh::isBoundary(MVert* vert) const
	{
		if (vert->halfEdge() == nullptr)
		{
			return true;
		}

		MHalfedge* he_begin = vert->halfEdge();
		MHalfedge* he = vert->halfEdge();
		do {
			if (he->isBoundary())
			{
				return true;
			}
			he = he->rotateNext();

		} while (he != he_begin);

		return false;
	}

	bool PolyMesh::isBoundary(const MEdge* edge) const
	{
		return (edge->halfEdge()->isBoundary() || edge->halfEdge()->pair()->isBoundary());
	}

	bool PolyMesh::isBoundary(const MHalfedge* halfedge) const
	{
		return halfedge->isBoundary(); 
	}

	bool PolyMesh::isIsolated(MVert* vert) const
	{
		return vert->isIsolated();
	}

	bool PolyMesh::isIsolated(MEdge* edge) const
	{
		MHalfedge* he0 = edge->halfEdge();
		MHalfedge* he1 = edge->halfEdge()->pair();

		return (he0->isBoundary() && he1->isBoundary());
	}

	bool PolyMesh::isIsolated(MPolyFace* face) const
	{
		MHalfedge* he_begin = face->halfEdge();
		MHalfedge* he = he_begin;
		MHalfedge* he_pair;
		do
		{
			he_pair = he->pair();
			if (!he_pair->isBoundary())
			{
				return false;
			}
			he = he->next();
		} while (he != he_begin);

		return true;
	}

	size_t PolyMesh::valence(MVert* vert) const {
		if (vert == nullptr)
			return 0;
		else
			return vertAdjacentVertices(vert).size();
	}

	std::vector<MHalfedge*> PolyMesh::vertAdjacentHalfEdge(MVert* vert) const
	{
		std::vector<MHalfedge*> v_adj; v_adj.clear();

		MHalfedge* he_begin = vert->halfEdge();
		MHalfedge* he = vert->halfEdge();

		do {
			v_adj.push_back(he);
			he = he->rotateNext();
		} while (he != he_begin);
		return v_adj;
	}

	std::vector<MVert*> PolyMesh::vertAdjacentVertices(MVert* vert) const
	{
		std::vector<MVert*> v_adj; v_adj.clear();

		MHalfedge* he_begin = vert->halfEdge();
		MHalfedge* he = vert->halfEdge();

		do {
			v_adj.push_back(he->toVertex());
			he = he->rotateNext();
		} while (he != he_begin);
		return v_adj;
	}

	std::vector<MEdge*> PolyMesh::vertAdjacentEdge(MVert* vert) const
	{
		std::vector<MEdge*> v_adj; v_adj.clear();

		MHalfedge* he_begin = vert->halfEdge();
		MHalfedge* he = vert->halfEdge();

		do {
			v_adj.push_back(he->edge());
			he = he->rotateNext();
		} while (he != he_begin);
		return v_adj;
	}

	std::vector<MPolyFace*> PolyMesh::vertAdjacentPolygon(MVert* vert) const
	{
		std::vector<MPolyFace*> v_adj; v_adj.clear();
		MHalfedge* he_begin = vert->halfEdge();
		MHalfedge* he = vert->halfEdge();

		do {
			if (he->polygon() != nullptr)
			{
				v_adj.push_back(he->polygon());
			}
			he = he->rotateNext();
		} while (he != he_begin);
		return v_adj;
	}

	std::vector<MPolyFace*> PolyMesh::edgeAdjacentPolygon(MEdge* edge) const
	{
		std::vector<MPolyFace*> e_poly; e_poly.clear();

		MPolyFace* poly = edge->halfEdge()->polygon();
		if (poly != nullptr) e_poly.push_back(poly);
		
		poly = edge->halfEdge()->pair()->polygon();
		if (poly != nullptr) e_poly.push_back(poly);

		return e_poly;
	}

	std::vector<MPolyFace*> PolyMesh::polygonAdjacentPolygon(MPolyFace* face) const
	{
		std::vector<MPolyFace*> f_face; f_face.clear();

		MHalfedge* he_begin = face->halfEdge();
		MHalfedge* he = face->halfEdge();

		do {
			if (he->pair()->polygon() != nullptr)
				f_face.push_back(he->pair()->polygon());

			he = he->next();
		} while (he != he_begin);
		return f_face;
	}

	std::vector<MVert*> PolyMesh::polygonVertices(MPolyFace* face) const
	{
		std::vector<MVert*> f_vert; f_vert.clear();

		MHalfedge* he_begin = face->halfEdge();
		MHalfedge* he = face->halfEdge();

		do {
			f_vert.push_back(he->toVertex());
			he = he->next();
		} while (he != he_begin);
		return f_vert;
	}

	std::vector<MHalfedge*> PolyMesh::polygonHalfedges(MPolyFace* face) const
	{
		std::vector<MHalfedge*> f_edge; f_edge.clear();

		MHalfedge* he_begin = face->halfEdge();
		MHalfedge* he = face->halfEdge();

		do {
			f_edge.push_back(he);
			he = he->next();
		} while (he != he_begin);
		return f_edge;
	}

	std::vector<MEdge*> PolyMesh::polygonEdges(MPolyFace* face) const
	{
		std::vector<MEdge*> f_edge; f_edge.clear();

		MHalfedge* he_begin = face->halfEdge();
		MHalfedge* he = face->halfEdge();

		do {
			f_edge.push_back(he->edge());
			he = he->next();
		} while (he != he_begin);
		return f_edge;
	}

	MHalfedge* PolyMesh::edgeHalfEdge(MEdge* edge, unsigned int id)
	{
		if (id == 0) 
			return edge->halfEdge();
		else if (id == 1) 
			return edge->halfEdge()->pair();
		else
			return nullptr;
	}

	MHalfedge* PolyMesh::halfedgeBetween(MVert* v0, MVert* v1)
	{
		if (v0 == nullptr || v1 == nullptr) return nullptr;
		if (v0->halfEdge() == nullptr) return nullptr;

		MHalfedge* he_begin = v0->halfEdge();
		MHalfedge* he = v0->halfEdge();

		do {
			if (he->toVertex() == v1)
			{
				return he;
			}
			he = he->rotateNext();
		} while (he != he_begin);

		return nullptr;
	}

	MEdge* PolyMesh::edgeBetween(MVert* v0, MVert* v1)
	{
		//MHalfedge* he = halfedgeBetween(v0, v1);
		//return he->edge();

		if (v0 == nullptr || v1 == nullptr) return nullptr;
		if (v0->halfEdge() == nullptr) return nullptr;

		MHalfedge* he_begin = v0->halfEdge();
		MHalfedge* he = v0->halfEdge();

		do {
			if (he->toVertex() == v1)
			{
				return he->edge();
			}
			he = he->rotateNext();
		} while (he != he_begin);

		return nullptr;
	}

	bool PolyMesh::isConnected(MVert* v0, MVert* v1)
	{
		if (edgeBetween(v0, v1) != nullptr)
			return true;
		else
			return false;
	}

	bool PolyMesh::isFaceContainsVertices(MPolyFace* face, MVert* vert)
	{
		std::vector<MVert*> fv_list = polygonVertices(face);
		size_t fv_num = fv_list.size();
		for (size_t i = 0; i < fv_num; i++)
		{
			if (fv_list[i] == vert) return true;
		}
		return false;
	}

	MHalfedge* PolyMesh::faceHalfEdgeFromVert(MPolyFace* face, MVert* vert)
	{ 
		MHalfedge* re;
		std::vector<MHalfedge*> fhe_list = polygonHalfedges(face);
		size_t fv_num = fhe_list.size();
		for (size_t i = 0; i < fv_num; i++)
		{
			if (fhe_list[i]->fromVertex() == vert)
			{
				re = fhe_list[i];
				return re;
			}
		}
		return nullptr;
	}

	bool PolyMesh::isTriangleMesh()
	{
		for (FaceIter f_it = polyfaces_begin(); f_it != polyfaces_end(); ++f_it)
		{
			if ((*f_it)->PolyNum() != 3)
			{
				return false;
			}
		}
		return true;
	}

//=====================================================================
// Base Topology
//=====================================================================
	void PolyMesh::reserveMemory(size_t nv)
	{
		vertices_.reserve(nv);
		poolV.Reserve(nv);

		half_edges_.reserve(6 * nv);
		poolHE.Reserve(6 * nv);

		edges_.reserve(3 * nv);
		poolE.Reserve(3 * nv);

		polygons_.reserve(2 * nv);
		poolP.Reserve(2 * nv);
	}
	void PolyMesh::reserveMemory(size_t nv, size_t nf)
	{
		vertices_.reserve(nv);
		poolV.Reserve(nv);

		half_edges_.reserve(6 * nv);
		poolHE.Reserve(6 * nv);

		edges_.reserve(3 * nv);
		poolE.Reserve(3 * nv);

		polygons_.reserve(2 * nf);
		poolP.Reserve(2 * nf);
	}

	void PolyMesh::clear()
	{
		for (MVert* v : vertices_)
			poolV.Recycle(v);
		vertices_.clear();

		for (MHalfedge* he : half_edges_)
			poolHE.Recycle(he);
		half_edges_.clear();

		for (MEdge* e : edges_)
			poolE.Recycle(e);
		edges_.clear();

		for (MPolyFace* p : polygons_)
			poolP.Recycle(p);
		polygons_.clear();
	}

	MVert* PolyMesh::newVertex()
	{
		MVert* w_ptr = poolV.Request();
		new(w_ptr) MVert(0, 0, 0);
		w_ptr->set_index(vertices_.size());
		vertices_.push_back(w_ptr);
		return w_ptr;
	}
	MEdge* PolyMesh::newEdge()
	{
		MEdge* w_ptr = poolE.Request();
		new(w_ptr) MEdge();
		w_ptr->set_index(edges_.size());
		edges_.push_back(w_ptr);
		return w_ptr;
	}
	MEdge* PolyMesh::newEdge(MVert* v1, MVert* v2)
	{
		MEdge* w_ptr = poolE.Request();
		new(w_ptr) MEdge(v1, v2);
		w_ptr->set_index(edges_.size());
		edges_.push_back(w_ptr);
		return w_ptr;
	}
	MHalfedge* PolyMesh::newHelfEdge()
	{
		MHalfedge* w_ptr = poolHE.Request();
		new(w_ptr) MHalfedge();
		w_ptr->set_index(half_edges_.size());
		half_edges_.push_back(w_ptr);
		return w_ptr;
	}
	MPolyFace* PolyMesh::newPolyFace()
	{
		MPolyFace* w_ptr = poolP.Request();
		new(w_ptr) MPolyFace();
		w_ptr->set_index(polygons_.size());
		polygons_.push_back(w_ptr);
		return w_ptr;
	}

	MVert* PolyMesh::addVertex(double x, double y, double z)
	{
		MVert* w_ptr = poolV.Request();
		new(w_ptr) MVert(x, y, z);
		w_ptr->set_index(vertices_.size());
		vertices_.push_back(w_ptr);
		return w_ptr;
	}
	MVert* PolyMesh::addVertex(const MPoint3& point)
	{
		MVert* w_ptr = poolV.Request();
		new(w_ptr) MVert(point.x(), point.y(), point.z());
		w_ptr->set_index(vertices_.size());
		vertices_.push_back(w_ptr);
		return w_ptr;
	}

	MEdge* PolyMesh::addEdge(MVert* v_begin, MVert* v_end)
	{
		if (v_begin == nullptr || v_end == nullptr)
		{
			Massage::Error("PolyMesh--AddEdge--v_begin == v_end");
			return nullptr;
		}

		MEdge* edge = edgeBetween(v_begin, v_end);
		if (edge != nullptr)
		{
			return edge;
		}

		edge = newEdge(v_begin, v_end);
		MHalfedge* he0 = newHelfEdge();
		MHalfedge* he1 = newHelfEdge();

		edge->setHalfedge(he0);
		//edge->set_index(edges_.size()); edges_.push_back(edge);

		he0->setNext(he1); he0->setPrev(he1);
		he0->setPair(he1); he0->setVert(v_begin); he0->setEdge(edge);
		he0->setPolygon(nullptr);

		he1->setNext(he0); he1->setPrev(he0);
		he1->setPair(he0); he1->setVert(v_end); he1->setEdge(edge);
		he1->setPolygon(nullptr);

		return edge;
	}

	MPolyFace* PolyMesh::addPolyFace(std::vector<MHalfedge*>& he_loop)
	{
		size_t he_size = he_loop.size();
		if (he_loop.size() == 0)
		{
			return nullptr;
		}
		for (size_t i = 0; i < he_size; i++)
		{
			size_t i1 = (i + 1) % he_size;
			if (he_loop[i]->toVertex() != he_loop[i1]->fromVertex())
			{
				Massage::Error("The face is not end to end");
				return nullptr;
			}
		}

		//在连接时，本来可以考虑拓扑重复？？？，
		for (size_t i = 0; i < he_size; i++)
		{
			size_t i1 = (i + 1) % he_size;
			
			he_loop[i]->setNext(he_loop[i1]);
			he_loop[i1]->setPrev(he_loop[i]);
		}

		MPolyFace* face = newPolyFace();
		face->setHalfedge(he_loop[0]);

		face->update();

		for (size_t i = 0; i < he_size; i++)
		{
			he_loop[i]->setPolygon(face);
		}
		

		return face;
	}

	MPolyFace* PolyMesh::addPolyFace(std::vector<MVert*>& v_loop)
	{
		size_t v_size = v_loop.size();

#ifndef NDEBUG
		for (size_t i = 0; i < v_size; i++)
		{
			if (!isBoundary(v_loop[i]))
			{
				//it means we need add edge to a inner fv_list, it will get a 3D edge.
				std::ostringstream error_info;
				error_info << "[AddFace] : complex vertex " <<
					v_loop[0]->index() << " " << v_loop[1]->index() << " " << v_loop[2]->index() << std::endl;
				Massage::Error(error_info.str());
			}
			size_t i1 = (i + 1) % v_size;
			MEdge* edge = edgeBetween(v_loop[i], v_loop[i1]);
			if (edge != nullptr && !isBoundary(edge))
			{
				//it means we need add face to a inner edge, it will get a 3D face.
				std::ostringstream error_info;
				error_info << "[AddFace] : complex edge " <<
					v_loop[0]->index() << " " << v_loop[1]->index() << " " << v_loop[2]->index() << std::endl;
				Massage::Error(error_info.str());
			}
		}
#endif // !NDEBUG

		MPolyFace* face = newPolyFace();

		std::vector<bool> is_edge_new(v_size, false);
		std::vector<bool> is_needc(v_size, false);


		MHalfedge* he1 = nullptr; MHalfedge* he2 = nullptr;
		std::vector<MHalfedge*> vec_edges;
		for (size_t i = 0; i < v_size; i++)
		{
			size_t i1 = (i + 1) % v_size;

			MEdge* e = edgeBetween(v_loop[i], v_loop[i1]);
			if (e == nullptr)
			{
				e = addEdge(v_loop[i], v_loop[i1]);
				is_edge_new[i] = true;
			}
			
			if (e->halfEdge()->fromVertex() == v_loop[i])
			{
				he1 = e->halfEdge();
				he2 = e->halfEdge()->pair(); 
			}
			else
			{
				he1 = e->halfEdge()->pair();
				he2 = e->halfEdge();
			}

			if (face->halfEdge() == nullptr)
			{
				face->setHalfedge(he1);
			}
			he1->setPolygon(face);	
			vec_edges.push_back(he1);
		}

		MHalfedge* he_ip, * he_in;
		MHalfedge* he_op, * he_on;
		MHalfedge* he_bp, * he_bn;
		for (size_t i = 0; i < v_size; i++)
		{
			size_t ii = (i + 1) % v_size;
			if (!is_edge_new[i] && !is_edge_new[ii])
			{
				he_ip = vec_edges[i];
				he_in = vec_edges[ii];

				if (he_ip->next() != he_in)
				{
					he_op = he_in->pair();
					he_on = he_ip->pair();

					he_bp = he_op;
					do
					{
						he_bp = he_bp->next()->pair();
					} while (!he_bp->isBoundary());
					he_bn = he_bp->next();

					//// ok ?
					//if (boundary_prev == inner_prev)
					//{
					//	omerr() << "PolyMeshT::add_face: patch re-linking failed\n";
					//	return make_smart(InvalidFaceHandle, this);
					//}

					assert(he_bp->isBoundary());
					assert(he_bn->isBoundary());

					MHalfedge* patch_start = he_ip->next();
					MHalfedge* patch_end = he_in->prev();

					he_bp->setNext(patch_start);	patch_start->setPrev(he_bp);
					patch_end->setNext(he_bn);		he_bn->setPrev(patch_end);
					he_ip->setNext(he_in);			he_in->setPrev(he_ip);
				}
			}
		}


		for (size_t i = 0; i < v_size; i++)
		{
			size_t i1 = (i + 1) % v_size;

			MVert* vh = v_loop[i1];
			he_ip = vec_edges[i];
			he_in = vec_edges[i1];

			assert(he_ip != nullptr);
			assert(he_in != nullptr);

			size_t id = 0;
			if (is_edge_new[i])  id |= 1;
			if (is_edge_new[i1]) id |= 2;

			if (id)
			{
				MHalfedge* he_op = he_in->pair();
				MHalfedge* he_on = he_ip->pair();

				// set outer links
				switch (id)
				{
				case 1: // prev is new, next is old
					he_bp = he_in->prev();
					he_bp->setNext(he_on); he_on->setPrev(he_bp);
					vh->setHalfedge(he_on);
					break;

				case 2: // next is new, prev is old
					he_bn = he_ip->next();
					he_op->setNext(he_bn); he_bn->setPrev(he_op);
					vh->setHalfedge(he_bn);
					break;

				case 3: // both are new
					if (vh->halfEdge() == nullptr)
					{
						vh->setHalfedge(he_on);
						he_op->setNext(he_on); he_on->setPrev(he_op);
					}
					else
					{
						he_bn = vh->halfEdge();
						he_bp = he_bn->prev();

						he_bp->setNext(he_on); he_on->setPrev(he_bp);
						he_op->setNext(he_bn); he_bn->setPrev(he_op);
					}
					break;
				}

				he_ip->setNext(he_in); he_in->setPrev(he_ip);
			}
			else
			{
				he_ip->setNext(he_in); he_in->setPrev(he_ip);
				is_needc[i1] = true;
			}
		}

		for (size_t i = 0; i < v_size; i++)
		{
			if (is_needc[i])
			{
				v_loop[i]->adjustOutgoingHalfedge();
			}
		}

		face->update();
		return face;
	}

	MPolyFace* PolyMesh::addPolyFace(std::vector<size_t>& v_loop_id)
	{
		size_t v_size = v_loop_id.size();

		std::vector<MVert*> v_loop; v_loop.reserve(v_size);
		for (size_t i = 0; i < v_size; i++)
		{
			v_loop.push_back(vertices_[v_loop_id[i]]);
		}
		return addPolyFace(v_loop);
	}

	void PolyMesh::deleteVertex(MVert* vert)
	{
		if (vert == nullptr)
			return;

		if (isIsolated(vert))
		{
			deleteVertex_fromMesh(vert);
		}
		else
		{
			for (MEdge* e : vertAdjacentEdge(vert))
			{
				deleteEdges(e);
			}

			deleteVertex_fromMesh(vert);
		}
	}

	void PolyMesh::deleteEdges(MEdge* edge)
	{
		assert(edge != nullptr);
		MHalfedge* he0 = edge->halfEdge();
		MHalfedge* he1 = edge->halfEdge()->pair();

		if (he0->polygon() != nullptr)
			deletePolyFace(he0->polygon());
		if (he1->polygon() != nullptr)
			deletePolyFace(he1->polygon());

		MVert* v0 = he0->fromVertex();
		MVert* v1 = he1->fromVertex();

		MHalfedge* he_to_v0 = he0->prev();
		MHalfedge* he_to_v1 = he1->prev();

		MHalfedge* v0_he_next = he0->rotateNext();
		MHalfedge* v1_he_next = he1->rotateNext();

		if (v0->halfEdge() == he0)
			v0->setHalfedge(v0_he_next == he0 ? nullptr : v0_he_next);

		if (v1->halfEdge() == he1)
			v1->setHalfedge(v1_he_next == he1 ? nullptr : v1_he_next);

		he_to_v0->setNext(v0_he_next);
		v0_he_next->setPrev(he_to_v0);

		he_to_v1->setNext(v1_he_next);
		v1_he_next->setPrev(he_to_v1);

		deleteHalfEdge_fromMesh(he0);
		deleteHalfEdge_fromMesh(he1);
		deleteEdge_fromMesh(edge);
	}

	void PolyMesh::deletePolyFace(MPolyFace* face)
	{
		assert(face != nullptr);

		if (isIsolated(face))
		{
			std::vector<MVert*> fv_list = polygonVertices(face);
			for (MHalfedge* he : polygonHalfedges(face))
			{
				he->setPolygon(nullptr);
			}
			deletePolyFace_fromMesh(face);

			deleteMultipleVerttex(fv_list);
		}
		else
		{
			std::vector<MHalfedge*> f_he_list = polygonHalfedges(face);
			std::vector<MVert*> f_v_list = polygonVertices(face);

			for (MHalfedge* he : f_he_list)
			{
				he->setPolygon(nullptr);
			}
			for (MVert* v : f_v_list)
			{
				v->adjustOutgoingHalfedge();
			}

			deletePolyFace_fromMesh(face);
		}

	}

	size_t PolyMesh::delete_isolated_vertices()
	{
		size_t v_size = vertices_.size();
		if (v_size == 0) return 0;

		size_t iso_num = 0;
		for (size_t i = v_size - 1; ; i--)
		{
			MVert* vert = vertices_[i];
			if (vert->isIsolated())
			{
				deleteVertex_fromMesh(vert);
				iso_num++;
			}
			if (i == 0)
				break;
		}
		return iso_num;
	}

	size_t PolyMesh::delete_isolated_edges()
	{
		size_t e_size = edges_.size();
		if (e_size == 0) return 0;

		size_t iso_num = 0;
		for (size_t i = e_size - 1; ; i--)
		{
			MEdge* edge = edges_[i];

			if (isIsolated(edge))
			{
				deleteEdges(edge);
				iso_num++;
			}
			if (i == 0) break;
		}
		return iso_num;
	}

	void PolyMesh::deleteMultipleVerttex(std::vector<MVert*>& vert_list)
	{	
		if (vert_list.size() == 0)
			return;
		for (size_t i = 0; i < vert_list.size(); i++)
		{
			for (size_t j = 0; j < vert_list.size() - 1 - i; j++)
			{
				if (vert_list[j + 1]->index() > vert_list[j]->index())
				{
					MVert* temp = vert_list[j + 1];
					vert_list[j + 1] = vert_list[j];
					vert_list[j] = temp;
				}
			}
		}

		for (size_t i = 0; i < vert_list.size(); i++)
		{
			deleteVertex(vert_list[i]);
		}
	}
	void PolyMesh::deleteMultipleEdge(std::vector<MEdge*> edge_list)
	{
		if (edge_list.size() == 0)
			return;
		for (size_t i = 0; i < edge_list.size(); i++)
		{
			for (size_t j = 0; j < edge_list.size() - 1 - i; j++)
			{
				if (edge_list[j + 1]->index() > edge_list[j]->index())
				{
					MEdge* temp = edge_list[j + 1];
					edge_list[j + 1] = edge_list[j];
					edge_list[j] = temp;
				}
			}
		}

		for (size_t i = 0; i < edge_list.size(); i++)
		{
			deleteEdges(edge_list[i]);
		}
	}
	void PolyMesh::deleteMultiplePolyFace(std::vector<MPolyFace*> face_list)
	{
		if (face_list.size() == 0)
			return;
		for (size_t i = 0; i < face_list.size(); i++)
		{
			for (size_t j = 0; j < face_list.size() - 1 - i; j++)
			{
				if (face_list[j + 1]->index() > face_list[j]->index())
				{
					MPolyFace* temp = face_list[j + 1];
					face_list[j + 1] = face_list[j];
					face_list[j] = temp;
				}
			}
		}

		for (size_t i = 0; i < face_list.size(); i++)
		{
			deletePolyFace(face_list[i]);
		}
	}


	void PolyMesh::deletePolyFace_fromMesh(MPolyFace* face)
	{
		assert(face->index() < polygons_.size());
		assert(face == polygons_[face->index()]);

		int f_index = face->index();
		poolP.Recycle(face);
		if (f_index != polygons_.size() - 1)
		{
			MPolyFace* face_back = polygons_.back();
			face_back->set_index(f_index);
			polygons_[f_index] = face_back;
		}
		polygons_.pop_back();
	}
	void PolyMesh::deleteHalfEdge_fromMesh(MHalfedge* halfedge)
	{
		assert(halfedge->index() < half_edges_.size());
		assert(halfedge == half_edges_[halfedge->index()]);

		int he_index = halfedge->index();
		poolHE.Recycle(halfedge);
		if (he_index != half_edges_.size() - 1)
		{
			MHalfedge* he_back = half_edges_.back();
			he_back->set_index(he_index);
			half_edges_[he_index] = he_back;
		}
		half_edges_.pop_back();
	}
	void PolyMesh::deleteEdge_fromMesh(MEdge* edge)
	{
		assert(edge->index() < edges_.size());
		assert(edge == edges_[edge->index()]);

		int he_index = edge->index();
		poolE.Recycle(edge);
		if (he_index != edges_.size() - 1)
		{
			MEdge* he_back = edges_.back();
			he_back->set_index(he_index);
			edges_[he_index] = he_back;
		}
		edges_.pop_back();
	}
	void PolyMesh::deleteVertex_fromMesh(MVert* vert)
	{
		assert(vert->index() < vertices_.size());
		assert(vert == vertices_[vert->index()]);

		int v_index = vert->index();
		poolV.Recycle(vert);
		if (v_index != vertices_.size() - 1)
		{
			MVert* v_back = vertices_.back();
			v_back->set_index(v_index);
			vertices_[v_index] = v_back;
		}
		vertices_.pop_back();

	}

	bool PolyMesh::mesh_manifold_check()
	{
#ifdef NDEBUG
		return true;
#else
		//must do something
		for (size_t i = 0; i < vertices_.size(); i++)
		{
			int bnum = 0;
			for (MHalfedge* he : vertAdjacentHalfEdge(vertices_[i]))
			{
				if (isBoundary(he))
				{
					bnum++;
				}
			}
			if (bnum > 1)
				return false;
		}
		return true;
#endif // NDEBUG
	}


//=====================================================================
// Mesh Based Method
//=====================================================================
	double PolyMesh::closestPoint(const MPoint3& p, MVert* close_v)
	{
		size_t v_size = vertices_.size();
		double c_dis = 1e6;
		MVert* cv = nullptr;
		for (size_t i = 0; i < v_size; i++)
		{
			MPoint3 vp = vertices_[i]->position();

			double v_dis = distance(vp, p);

			if (v_dis < c_dis)
			{
				cv = vertices_[i];
				c_dis = v_dis;
			}
		}
		close_v = cv;
		return c_dis;
	}

	MVert* PolyMesh::closestPoint(const MPoint3& p)
	{
		MVert* cv = nullptr;
		double dis = closestPoint(p, cv);
		if (dis < MehsThreshold)
			return cv;
		else
			return nullptr;
	}

//=====================================================================
// Low Level API
//=====================================================================
	MVert* PolyMesh::splitEdgeMakeVertex(MEdge* edge)
	{
		double sp = 0.5;

		MEdge* e0 = edge;
		MHalfedge* he0 = e0->halfEdge();
		MHalfedge* he1 = e0->halfEdge()->pair();

		MVert* v0 = he0->fromVertex();
		MVert* v1 = he1->fromVertex();

		MPoint3 p = v0->position()* sp + v1->position()* (1 - sp);

		MPolyFace* p0 = he0->polygon();
		MPolyFace* p1 = he1->polygon();

		MHalfedge* he0_next = he0->next(); MHalfedge* he0_prev = he0->prev();
		MHalfedge* he1_next = he1->next(); MHalfedge* he1_prev = he1->prev();

		MVert* vt = addVertex(p);
		MEdge* e1 = addEdge(vt, v1);
		MHalfedge* het0 = e1->halfEdge();
		MHalfedge* het1 = het0->pair();

		vt->setHalfedge(het0); v1->setHalfedge(het1);

		he1->setVert(vt);

		het0->setPolygon(p0); het1->setPolygon(p1);

		het0->setNext(he0_next);	he0_next->setPrev(het0);
		he0->setNext(het0);			het0->setPrev(he0);

		het1->setNext(he1);			he1->setPrev(het1);
		het1->setPrev(he1_prev);	he1_prev->setNext(het1);

		if (p1 != nullptr)
		{
			p1->setHalfedge(het1);
		}

		if (p0 != nullptr)
		{
			p0->setHalfedge(he0);
		}

		v0->adjustOutgoingHalfedge();
		v1->adjustOutgoingHalfedge();
		vt->adjustOutgoingHalfedge();
		return vt;
	}

	MEdge* PolyMesh::jointEdgeRemoveVertex(MVert* vert)
	{
		if (valence(vert) != 2)
			return nullptr;

		MHalfedge* he0 = vert->halfEdge();
		MHalfedge* he1 = he0->pair();
		MHalfedge* he2 = he0->prev();
		MHalfedge* he3 = he2->pair();

		assert(he1->toVertex() == vert);
		assert(he2->toVertex() == vert);
		assert(he3->fromVertex() == vert);

		MEdge* e0 = he0->edge();
		MEdge* e1 = he2->edge();

		MVert* v0 = he0->toVertex();
		MVert* v1 = he3->toVertex();

		if (he0->polygon() != he2->polygon() || he1->polygon() != he3->polygon())
			return nullptr;
		//不能删除三角形的一条边
		if (he0->next() == he2->prev() || he1->prev() == he3->next())
			return nullptr;

		MPolyFace* p0 = he0->polygon();
		MPolyFace* p1 = he1->polygon();

		he0->setVert(v1);

		MHalfedge* he2_prev = he2->prev();
		MHalfedge* he3_next = he3->next();

		he0->setPrev(he2_prev); he2_prev->setNext(he0);
		he1->setNext(he3_next); he3_next->setPrev(he1);
		e0->updateVert();

		if (p0 != nullptr && p0->halfEdge() == he2) p0->setHalfedge(he0);
		if (p1 != nullptr && p1->halfEdge() == he3) p1->setHalfedge(he1);

		vert->setHalfedge(nullptr);
		deleteHalfEdge_fromMesh(he2);
		deleteHalfEdge_fromMesh(he3);
		deleteEdge_fromMesh(e1);
		deleteVertex_fromMesh(vert);

		v0->adjustOutgoingHalfedge();
		v1->adjustOutgoingHalfedge();
		return e0;
	}

	MEdge* PolyMesh::mergeEdge(MEdge* edge0, MEdge* edge1)
	{
		MHalfedge* he00 = edge0->halfEdge();
		MHalfedge* he01 = edge0->halfEdge()->pair();

		MHalfedge* he10 = edge1->halfEdge();
		MHalfedge* he11 = edge1->halfEdge()->pair();

		MVert* v00 = edge0->getVert(0); MVert* v01 = edge0->getVert(1);
		MVert* v10 = edge1->getVert(0); MVert* v11 = edge1->getVert(1);
		MVert* ev = nullptr;
		if (v10 == v00 || v10 == v01)
		{
			ev = v10;
		}
		if (v11 == v00 || v11 == v01)
		{
			ev = v11;
		}
		if (ev == nullptr)
		{
			Massage::Warning("Two edge no intersection");
			return nullptr;
		}
		return jointEdgeRemoveVertex(ev);
	}


	MPolyFace* PolyMesh::mergeFace(MPolyFace* f0, MPolyFace* f1)
	{
		MHalfedge* he_begin = f0->halfEdge();
		MHalfedge* he = he_begin;
		do
		{
			if (he->pair()->polygon() == f1)
			{
				return jointFaceRemoveEdge(f0, f1, he->edge());
			}
			he = he->next();
		} while (he != he_begin);

		return nullptr;
	}

	MPolyFace* PolyMesh::jointFaceRemoveEdge(MEdge* edge)
	{
		if (isBoundary(edge))
			return nullptr;
		
		MHalfedge* he0 = edge->halfEdge();
		MHalfedge* he1 = edge->halfEdge()->pair();

		if (he0->polygon() == he1->polygon())
		{
			Massage::Error("Mesh Joint Face -- one in edge have only one face");
			return nullptr;
		}
		MPolyFace* f0 = he0->polygon();
		MPolyFace* f1 = he1->polygon();

		MHalfedge* he_iter = he0->next();
		do
		{
			MHalfedge* he_it_pair = he_iter->pair();
			if (he_it_pair->polygon() == f1)
			{
				Massage::Error("Mesh Joint Face -- two face have more than intersect edge");
				return nullptr;
			}
			he_iter = he_iter->next();
		} while (he_iter != he0);

		MHalfedge* he_begin = he0;
		MHalfedge* he = he0->next();
		do
		{
			if (he->pair()->polygon() == he->polygon())
			{
				return nullptr;
			}
			he = he->next();
		} while (he != he_begin);

		return jointFaceRemoveEdge(f0, f1, edge);
	}

	MPolyFace* PolyMesh::jointFaceRemoveEdge(MPolyFace* f0, MPolyFace* f1, MEdge* edge)
	{
		MHalfedge* he0 = edge->halfEdge();
		MHalfedge* he1 = edge->halfEdge()->pair();

		MHalfedge* he0_pre = he0->prev(); MHalfedge* he0_next = he0->next();
		MHalfedge* he1_pre = he1->prev(); MHalfedge* he1_next = he1->next();

		std::vector<MHalfedge*> f1_edge = polygonHalfedges(f1);
		for (MHalfedge* f_he : f1_edge)
		{
			f_he->setPolygon(f0);
		}

		he0_pre->setNext(he1_next); he1_next->setPrev(he0_pre);
		he1_pre->setNext(he0_next); he0_next->setPrev(he1_pre);


		f0->setHalfedge(he0_next);

		MVert* v0 = he0->fromVertex();
		MVert* v1 = he1->fromVertex();
		if (v0->halfEdge() == he0)
			v0->setHalfedge(he1_next);
		if (v1->halfEdge() == he1)
			v1->setHalfedge(he0_next);

		deleteHalfEdge_fromMesh(he0);
		deleteHalfEdge_fromMesh(he1);
		deleteEdge_fromMesh(edge);
		deletePolyFace_fromMesh(f1);

		v0->adjustOutgoingHalfedge();
		v1->adjustOutgoingHalfedge();

		return f0;
	}


	MEdge* PolyMesh::splitFaceMakeEdge(MPolyFace* face, MVert* v0, MVert* v1)
	{
		if (v0 == v1) return nullptr;
		if (!isFaceContainsVertices(face, v0) || !isFaceContainsVertices(face, v1))
			return nullptr;

		if (isConnected(v0, v1))
			return nullptr;

		MHalfedge* he_begin = face->halfEdge();
		MHalfedge* he = face->halfEdge();
		MHalfedge* he0 = nullptr; MHalfedge* he1 = nullptr;
		do {
			if (he->fromVertex() == v0)
				he0 = he;
			if (he->fromVertex() == v1)
				he1 = he;
			he = he->next();
		} while (he != he_begin);

		MHalfedge* he0_pre = he0->prev();
		MHalfedge* he1_pre = he1->prev();

		MEdge* e_new = addEdge(v1, v0);
		MHalfedge* he0_new = e_new->halfEdge();
		MHalfedge* he1_new = he0_new->pair();

		he0_new->setNext(he0); he0->setPrev(he0_new);
		he0_new->setPrev(he1_pre); he1_pre->setNext(he0_new);

		he1_new->setNext(he1); he1->setPrev(he1_new);
		he1_new->setPrev(he0_pre); he0_pre->setNext(he1_new);

		he0_new->setPolygon(face); face->setHalfedge(he0);

		MPolyFace* f_new = newPolyFace();
		f_new->setHalfedge(he1);

		he = he1;
		do {
			he->setPolygon(f_new);
			he = he->next();
		} while (he != he1);

		return e_new;
	}

	MEdge* PolyMesh::splitQuadrilateralTriangle(MPolyFace* face, bool isqulaty)
	{
		if (face->PolyNum() != 4)
		{
			Massage::Error("can not split Face not Quadrilateral, Place use splitFaceNGon.");
			return nullptr;
		}

		if (!isqulaty)
		{
			MHalfedge* he0 = face->halfEdge();
			MHalfedge* he1 = he0->next()->next();
			MVert* v0 = he0->fromVertex();
			MVert* v1 = he1->fromVertex();
			return splitFaceMakeEdge(face, v0, v1);
		}

		MHalfedge* he0 = face->halfEdge();	MVector3 vec0 = he0->tangent();
		MHalfedge* he1 = he0->next();		MVector3 vec1 = he0->tangent();
		MHalfedge* he2 = he1->next();		MVector3 vec2 = he0->tangent();
		MHalfedge* he3 = he2->next();		MVector3 vec3 = he0->tangent();

		double a0 = dot(vec0, -vec3);
		double a1 = dot(vec1, -vec0);
		double a2 = dot(vec2, -vec1);
		double a3 = dot(vec3, -vec2);

		if (a0 <= a1 && a0 <= a3 || a2 <= a1 && a2 <= a3)
		{
			MVert* v0 = he0->fromVertex();
			MVert* v1 = he2->fromVertex();
			return splitFaceMakeEdge(face, v0, v1);
		}
		if (a1 <= a0 && a1 <= a2 || a3 <= a0 && a3 <= a2)
		{
			MVert* v0 = he1->fromVertex();
			MVert* v1 = he3->fromVertex();
			return splitFaceMakeEdge(face, v0, v1);
		}

		return nullptr;
	}

	void PolyMesh::splitNGonTriangle(MPolyFace* face)
	{
		if (face->PolyNum() == 4)
		{
			splitQuadrilateralTriangle(face);
			return;
		}

		if (face->PolyNum() <= 3)
		{
			return;
		}


		MHalfedge* he0 = face->halfEdge();
		MHalfedge* he1 = he0->next()->next();

		MVert* v0 = he0->fromVertex();
		MVert* v1 = he1->fromVertex();
		MEdge* new_edge = splitFaceMakeEdge(face, v0, v1);

		if (new_edge == nullptr)
			return;

		he0 = new_edge->halfEdge();
		he1 = he0->pair();
		if (he0->polygon() != nullptr)
		{
			MPolyFace* new_face = he0->polygon();
			splitNGonTriangle(new_face);
		}
		if (he1->polygon() != nullptr)
		{
			MPolyFace* new_face = he1->polygon();
			splitNGonTriangle(new_face);
		}
		return;
	}

	bool PolyMesh::weldVertex(MVert* v_ori, MVert* v_tar, double tolerance)
	{
		if (v_ori == v_tar)
		{
			return false;
		}
		if (isConnected(v_ori, v_tar))
		{
			Massage::Warning("the splice vert is connected, using collapse");
			return false;
		}

		if (distance(v_ori->position(), v_tar->position()) > tolerance)
		{
			Massage::Warning("The two vertices which need to be spliced are too far");
			return false;
		}

		MHalfedge* v0_he_begin = v_ori->halfEdge();
		MHalfedge* v1_he_begin = v_tar->halfEdge();

		std::vector<MHalfedge*> v0_he_list = vertAdjacentHalfEdge(v_ori);
		std::vector<MHalfedge*> v1_he_list = vertAdjacentHalfEdge(v_tar);

		MHalfedge* v0_he = v0_he_begin;

		std::vector<MEdge*> edge_pair0; edge_pair0.clear();
		std::vector<MEdge*> edge_pair1; edge_pair1.clear();

		for (size_t i = 0; i < v0_he_list.size(); i++)
		{
			MHalfedge* v0_he = v0_he_list[i];
			MPoint3 p0 = v0_he->toVertex()->position();
			for (size_t j = 0; j < v1_he_list.size(); j++)
			{
				MHalfedge* v1_he = v1_he_list[i];
				MPoint3 p1 = v1_he->toVertex()->position();
				if (distance(p0, p1) <= tolerance)
				{
					edge_pair0.push_back(v0_he->edge());
					edge_pair1.push_back(v1_he->edge());
					break;
				}
			}
		}

		for (size_t i = 0; i < edge_pair0.size(); i++)
		{
			weldEdge(edge_pair0[i], edge_pair1[i]);
		}
		return true;
	}

	bool PolyMesh::weldEdge(MEdge* e0, MEdge* e1, double tolerance)
	{
		if (!isBoundary(e0) || !isBoundary(e1))
		{
			Massage::Warning("The two splice edge is not all boundary edge");
			return false;
		}

		MHalfedge* he00 = e0->halfEdge(); 
		if (!isBoundary(he00))
		{
			he00 = e0->halfEdge()->pair();
		}

		MHalfedge* he10 = e1->halfEdge();
		if (!isBoundary(he10))
		{
			he10 = e1->halfEdge()->pair();
		}

		MVector3 h0_target = he00->tangent();
		MVector3 h1_tagert = he10->tangent();

		double d_dot = dot(h0_target, h1_tagert);
		double d_toler = cos(10.0 / 180.0);

		if (d_dot<d_toler && d_dot >(-1 * d_toler))
		{
			Massage::Warning("The two splice edge is not parallel");
			return false;
		}

		if (d_dot > d_toler)
		{
			Massage::Warning("The two splice edge face with opposite normal");
			return false;
		}

		MHalfedge* he01 = he00->pair();
		MHalfedge* he11 = he10->pair();

		MVert* v0 = he00->fromVertex();
		MVert* v1 = he00->toVertex();

		MVert* v0_new = he10->fromVertex();
		MVert* v1_new = he10->toVertex();

		if (v0 != v1_new)
		{
			if(distance(v0->position(), v1_new->position()) > tolerance)
			{
				Massage::Warning("The two splice edge's endpoint is two far");
				return false;
			}
			std::vector<MHalfedge*> v0_he_list = vertAdjacentHalfEdge(v0);
			for (size_t i = 0; i < v0_he_list.size(); i++)
			{
				v0_he_list[i]->setVert(v1_new);
			}
			v0->setHalfedge(nullptr);
		}
		if (v1 != v0_new)
		{
			if (distance(v1->position(), v0_new->position()) > tolerance)
			{
				Massage::Warning("The two splice edge's endpoint is two far");
				return false;
			}
			std::vector<MHalfedge*> v1_he_list = vertAdjacentHalfEdge(v1);
			for (size_t i = 0; i < v1_he_list.size(); i++)
			{
				v1_he_list[i]->setVert(v0_new);
			}
			v1->setHalfedge(nullptr);
		}

		MHalfedge* he00_pre = he00->prev();		MHalfedge* he00_next = he00->next();
		MHalfedge* he01_pre = he01->prev();		MHalfedge* he01_next = he01->next();
		MHalfedge* he10_pre = he10->prev();		MHalfedge* he10_next = he10->next();
		MHalfedge* he11_pre = he11->prev();		MHalfedge* he11_next = he11->next();

		MPolyFace* f0 = he01->polygon();
		MPolyFace* f1 = he11->polygon();

		he10_pre->setNext(he00_next); he00_next->setPrev(he10_pre);
		he10_next->setPrev(he00_pre); he00_pre->setNext(he10_next);

		he10->setPrev(he01_pre);	he01_pre->setNext(he10);
		he10->setNext(he01_next);	he01_next->setPrev(he10);

		if (f0->halfEdge() == he01)
		{
			f0->setHalfedge(he10);
		}

		deleteHalfEdge_fromMesh(he00);
		deleteHalfEdge_fromMesh(he01);
		deleteEdge_fromMesh(e0);		

		if (v0->halfEdge() == nullptr)
			deleteVertex_fromMesh(v0);
		if (v1->halfEdge() == nullptr)
			deleteVertex_fromMesh(v1);

		v0_new->adjustOutgoingHalfedge();
		v1_new->adjustOutgoingHalfedge();

		return true;
	}

	bool PolyMesh::separateSingleVert(MVert* vert)
	{
		std::vector<MVert*> vert_list;

		return separateSingleVert(vert, vert_list);

		return false;
	}

	bool PolyMesh::separateSingleVert(MVert* vert, std::vector<MVert*>& new_vert_list)
	{
		if (!isBoundary(vert))
		{
			return false;
		}

		std::vector<MHalfedge*> v_he_list = vertAdjacentHalfEdge(vert);
		size_t boundary_edge = 0;
		for (MHalfedge* v_he : v_he_list)
		{
			MEdge* v_e = v_he->edge();
			if (isBoundary(v_e))
			{
				boundary_edge++;
			}
		}

		if (boundary_edge % 2 != 0)
		{
			Massage::Error("The topology is error -- the vert have odd boundary");
			return false;
		}
		if (boundary_edge == 2)
		{
			return false;
		}
		
		MPoint3 vp = vert->position();
		new_vert_list.clear(); new_vert_list.push_back(vert);

		std::vector<MHalfedge*> vhe_begin_list; vhe_begin_list.clear();
		std::vector<MHalfedge*> vhe_end_list; vhe_end_list.clear();

		MHalfedge* vhe_begin = vert->halfEdge();
		MHalfedge* vhe_end = vert->halfEdge();
		int boundary_halfedge = 0;
		while (1)
		{
			vhe_begin = vhe_end;
			do
			{
				if (vhe_begin->isBoundary())
					break;
				vhe_begin = vhe_begin->rotateNext();
			} while (true);
			vhe_begin_list.push_back(vhe_begin);

			vhe_end = vhe_begin;
			do
			{
				if (vhe_end->pair()->isBoundary())
					break;
				vhe_end = vhe_end->rotateNext();
			} while (true);
			vhe_end_list.push_back(vhe_end);

			boundary_halfedge++;
			if (boundary_halfedge == boundary_edge / 2)
				break;
		}

		for (size_t i = 0; i < vhe_begin_list.size(); i++)
		{
			std::vector<MHalfedge*> vhe_; vhe_.clear();
			MHalfedge* he_begin = vhe_begin_list[i];
			MHalfedge* he_end = vhe_end_list[i];
			for (MHalfedge* he = he_begin; he != he_end; he = he->rotateNext())
			{
				vhe_.push_back(he);
			}
			vhe_.push_back(he_end);

			if (i != 0)
			{
				MVert* v_new = addVertex(vp);
				v_new->setHalfedge(vhe_[0]);
				for (MHalfedge* he : vhe_)
				{
					he->setVert(v_new);
				}
				new_vert_list.push_back(v_new);
			}

			he_end = vhe_end_list[i]->pair();
			he_begin->setPrev(he_end); he_end->setNext(he_begin);
		}

		for (size_t i = 0; i < new_vert_list.size(); i++)
		{
			new_vert_list[i]->adjustOutgoingHalfedge();
		}

		return true;

	}

	bool PolyMesh::separateSingleEdge(MEdge* edge)
	{
		std::vector<MEdge*> edge_list;

		return separateSingleEdge(edge, edge_list);
	}

	bool PolyMesh::separateSingleEdge(MEdge* edge, std::vector<MEdge*>& new_edge_list)
	{
		if (isBoundary(edge))
		{
			Massage::Warning("Edge separate -- can not separate boundary edge");
			return false;
		}

		MHalfedge* he0 = edge->halfEdge();
		MHalfedge* he1 = edge->halfEdge()->pair();

		MPolyFace* ef0 = he0->polygon();
		MPolyFace* ef1 = he1->polygon();

		MVert* ev0 = he0->fromVertex();
		MVert* ev1 = he1->fromVertex();

		MHalfedge* he0_prev = he0->prev(); MHalfedge* he0_next = he0->next();
		MHalfedge* he1_prev = he1->prev(); MHalfedge* he1_next = he1->next();

		MHalfedge* he0_new = newHelfEdge();
		MHalfedge* he1_new = newHelfEdge();
		MEdge* e_new = newEdge(ev0, ev1);

		he0_new->setPair(he1_new);	he1_new->setPair(he0_new);
		he0_new->setVert(ev0);		he1_new->setVert(ev1);	
		he0_new->setEdge(e_new);	he1_new->setEdge(e_new);
		e_new->setHalfedge(he0_new);

		he0_new->setPolygon(nullptr); he1_new->setPolygon(ef1);
		if (ef1->halfEdge() == he1)
			ef1->setHalfedge(he1_new);
		he1->setPolygon(nullptr);

		he0_new->setNext(he1); he1->setPrev(he0_new);
		he0_new->setPrev(he1); he1->setNext(he0_new);

		he1_new->setPrev(he1_prev); he1_prev->setNext(he1_new);
		he1_new->setNext(he1_next); he1_next->setPrev(he1_new);

		new_edge_list.clear();
		new_edge_list.push_back(edge);
		new_edge_list.push_back(e_new);

		return true;
	}

	void PolyMesh::reverse_mesh()
	{
		if (isEmpty()) return;

		std::vector<MHalfedge*> he_prev_list; he_prev_list.clear(); he_prev_list.reserve(half_edges_.size());
		std::vector<MHalfedge*> he_next_list; he_next_list.clear(); he_next_list.reserve(half_edges_.size());
		std::vector<MVert*> he_tovert; he_tovert.clear(); he_tovert.reserve(half_edges_.size());

		for (size_t i = 0; i < half_edges_.size(); i++)
		{
			MHalfedge* he = half_edges_[i];
			he_prev_list.push_back(he->prev());
			he_next_list.push_back(he->next());
			he_tovert.push_back(he->toVertex());
		}

		for (size_t i = 0; i < half_edges_.size(); i++)
		{
			MHalfedge* he = half_edges_[i];
			he->setPrev(he_next_list[i]);
			he->setNext(he_prev_list[i]);
			he->setVert(he_tovert[i]);
		}
	}

	void PolyMesh::reverse_face(MPolyFace* face)
	{
		std::vector<MHalfedge*> fh_list; fh_list.clear(); fh_list.reserve(face->PolyNum());
		std::vector<MHalfedge*> fh_prev_list; fh_prev_list.clear(); fh_prev_list.reserve(face->PolyNum());
		std::vector<MHalfedge*> fh_next_list; fh_next_list.clear(); fh_next_list.reserve(face->PolyNum());
		std::vector<MVert*> fh_tovert; fh_tovert.clear(); fh_tovert.reserve(face->PolyNum());

		MHalfedge* he_begin = face->halfEdge();
		MHalfedge* he = he_begin;
		do {
			fh_list.push_back(he);
			fh_prev_list.push_back(he->prev());
			fh_tovert.push_back(he->toVertex());
			he = he->next();
			fh_next_list.push_back(he);
		} while (he != he_begin);

		for (size_t i = 0; i < fh_list.size(); i++)
		{
			fh_list[i]->setPrev(fh_next_list[i]);
			fh_list[i]->setNext(fh_prev_list[i]);
			fh_list[i]->setVert(fh_tovert[i]);
		}
		face->updataNormal();
	}

	bool PolyMesh::reverseIsolatedFace(MPolyFace* face)
	{
		if (!isIsolated(face))
		{
			Massage::Warning("The face is not isolate");
			return false;
		}

		std::vector<MHalfedge*> fh_list; fh_list.clear(); fh_list.reserve(face->PolyNum());
		std::vector<MHalfedge*> fh_prev_list; fh_prev_list.clear(); fh_prev_list.reserve(face->PolyNum());
		std::vector<MHalfedge*> fh_next_list; fh_next_list.clear(); fh_next_list.reserve(face->PolyNum());
		std::vector<MVert*> fh_tovert; fh_tovert.clear(); fh_tovert.reserve(face->PolyNum());

		std::vector<MHalfedge*> fhp_list; fhp_list.clear(); fhp_list.reserve(face->PolyNum());
		std::vector<MHalfedge*> fhp_prev_list; fhp_prev_list.clear(); fhp_prev_list.reserve(face->PolyNum());
		std::vector<MHalfedge*> fhp_next_list; fhp_next_list.clear(); fhp_next_list.reserve(face->PolyNum());
		std::vector<MVert*> fhp_tovert; fhp_tovert.clear(); fhp_tovert.reserve(face->PolyNum());

		MHalfedge* he_begin = face->halfEdge();
		MHalfedge* he = he_begin;
		MHalfedge* hep;
		do {
			fh_list.push_back(he);
			fh_prev_list.push_back(he->prev());
			fh_tovert.push_back(he->toVertex());

			hep = he->pair();
			fhp_list.push_back(hep);
			fhp_prev_list.push_back(hep->prev());
			fhp_next_list.push_back(hep->next());
			fhp_tovert.push_back(he->fromVertex());

			he = he->next();
			fh_next_list.push_back(he);
		} while (he != he_begin);

		for (size_t i = 0; i < fh_list.size(); i++)
		{
			fh_list[i]->setPrev(fh_next_list[i]);
			fh_list[i]->setNext(fh_prev_list[i]);
			fh_list[i]->setVert(fh_tovert[i]);
		}
		for (size_t i = 0; i < fhp_list.size(); i++)
		{
			fhp_list[i]->setPrev(fhp_next_list[i]);
			fhp_list[i]->setNext(fhp_prev_list[i]);
			fhp_list[i]->setVert(fhp_tovert[i]);
		}
		face->updataNormal();

		for (size_t i = 0; i < fh_list.size(); i++)
		{
			fh_tovert[i]->setHalfedge(fh_list[i]);
			fh_tovert[i]->adjustOutgoingHalfedge();
		}

		return true;
	}

	void PolyMesh::updateFacesNormal()
	{
		for (size_t i = 0; i < polygons_.size(); i++)
		{
			polygons_[i]->updataNormal();
		}
	}

	void PolyMesh::updateVerticesNormal(bool is_update_face)
	{
		if (!is_update_face)
			updateFacesNormal();

		for (size_t i = 0; i < vertices_.size(); i++)
		{
			if (!isIsolated(vertices_[i]))
			{
				MVector3 n(0, 0, 0);
				int num = 0;
				for (MPolyFace* face : vertAdjacentPolygon(vertices_[i]))
				{
					n += face->normal();
					num++;
				}
				n /= num;
				n.normalize();
				vertices_[i]->setNormal(n);
			}
		}
	}

	void PolyMesh::updateMeshNormal()
	{
		updateFacesNormal();
		updateVerticesNormal(true);
	}


	void PolyMesh::SplitFaceWithSingleFaceVertex(MPolyFace* face, MVert* v_new)
	{
		std::vector<MVert*> fv_vect; fv_vect.clear();
		MHalfedge* he = face->halfEdge();
		while (he->fromVertex() != v_new)
		{
			he = he->next();
		}
		assert(he->fromVertex() == v_new);
		MHalfedge* he_begin = he->prev();
		he_begin = he_begin->prev();
		he = he->next();
		do
		{
			fv_vect.push_back(he->toVertex());
			he = he->next();
		} while (he != he_begin);

		MPolyFace* newf = face;
		for (size_t i = 0; i < fv_vect.size(); i++)
		{
			MEdge* new_edge = splitFaceMakeEdge(newf, v_new, fv_vect[i]);
			MHalfedge* nhe0 = new_edge->halfEdge();
			MHalfedge* nhe1 = nhe0->pair();
			MPolyFace* nf0 = nhe0->polygon();
			MPolyFace* nf1 = nhe1->polygon();
			if (nf0->PolyNum() > 3)
				newf = nf0;
			else if (nf1->PolyNum() > 3)
				newf = nf1;
			else
				break;
		}
	}

	MVert* PolyMesh::splitEdgeSplitPolygon(MEdge* edge)
	{
		MHalfedge* he0 = edge->halfEdge();
		MHalfedge* he1 = he0->pair();

		MPolyFace* f0 = he0->polygon();
		MPolyFace* f1 = he1->polygon();

		MVert* v_new = splitEdgeMakeVertex(edge);
		if (f0 != nullptr)
		{
			SplitFaceWithSingleFaceVertex(f0, v_new);
		}

		if (f1 != nullptr)
		{
			SplitFaceWithSingleFaceVertex(f1, v_new);
		}

		return v_new;
	}

//=====================================================================
// Triangel Mesh API //并不局限于Triangle Mesh
//=====================================================================

	MVert* PolyMesh::splitEdgeTriangle(MEdge* edge)
	{
		MHalfedge* he0 = edge->halfEdge();
		MHalfedge* he1 = he0->pair();

		if (he0->isBoundary() && he1->isBoundary())
		{
			Massage::Warning("Can not split edge with two side are boundaries");
			return nullptr;
		}

#ifdef NDEBUG
#else
		if (!he0->isBoundary())
		{
			MPolyFace* f0 = he0->polygon();
			if (f0->PolyNum() != 3)
			{
				Massage::Error("Face must triangle"); return nullptr;
			}
		}
		if (!he1->isBoundary())
		{
			MPolyFace* f0 = he1->polygon();
			if (f0->PolyNum() != 3)
			{
				Massage::Error("Face must triangle"); return nullptr;
			}
		}
#endif // DEBUG

		MVert* v0 = he0->fromVertex();
		MVert* v1 = he0->toVertex();
	
		MVert* vt = newVertex();
		MPoint3 p0 = v0->position(); MPoint3 p1 = v1->position();
		double x = (p0.x() + p1.x()) / 2; double y = (p0.y() + p1.y()) / 2; double z = (p0.z() + p1.z()) / 2;
		vt->setPosition(x, y, z);

		MEdge* et = addEdge(vt, v1); 
		MHalfedge* het0 = et->halfEdge();
		MHalfedge* het1 = het0->pair();
		vt->setHalfedge(het0);

		he1->setVert(vt);

		if (!he0->isBoundary())
		{
			MPolyFace* f0 = he0->polygon();

			assert(f0->PolyNum() == 3);

			MHalfedge* he2 = he0->next();
			MHalfedge* he3 = he2->next();

			MVert* v2 = he2->toVertex();

			MEdge* ef = addEdge(vt, v2);
			MHalfedge* hef0 = ef->halfEdge();
			MHalfedge* hef1 = hef0->pair();

			MPolyFace* f1 = newPolyFace();

			f0->setHalfedge(he0); f1->setHalfedge(het0);

			he0->setPolygon(f0);
			he3->setPolygon(f0);
			hef0->setPolygon(f0);

			het0->setPolygon(f1);
			he2->setPolygon(f1);
			hef1->setPolygon(f1);

			het0->setNext(he2);		he2->setPrev(het0);
			he2->setNext(hef1);		hef1->setPrev(he2);
			hef1->setNext(het0);	het0->setPrev(hef1);

			he0->setNext(hef0);		hef0->setPrev(he0);
			hef0->setNext(he3);		he3->setPrev(hef0);
			he3->setNext(he0);		he0->setPrev(he3);
		}
		else
		{
			MHalfedge* he0_next = he0->next();
			het0->setNext(he0_next);	he0_next->setPrev(het0);
			he0->setNext(het0);			het0->setPrev(he0);
		}

		if (!he1->isBoundary())
		{
			MPolyFace* f0 = he1->polygon();

			assert(f0->PolyNum() == 3);

			MHalfedge* he2 = he1->next();
			MHalfedge* he3 = he2->next();

			MVert* v2 = he2->toVertex();

			MEdge* ef = addEdge(vt, v2);
			MHalfedge* hef0 = ef->halfEdge();
			MHalfedge* hef1 = hef0->pair();

			MPolyFace* f1 = newPolyFace();

			f0->setHalfedge(he1); f1->setHalfedge(het1);

			he1->setPolygon(f0);
			he2->setPolygon(f0);
			hef1->setPolygon(f0);

			het1->setPolygon(f1);
			he3->setPolygon(f1);
			hef0->setPolygon(f1);

			het1->setNext(hef0);	hef0->setPrev(het1);
			hef0->setNext(he3);		he3->setPrev(hef0);
			he3->setNext(het1);		het1->setPrev(he3);

			he1->setNext(he2);		he2->setPrev(he1);
			he2->setNext(hef1);		hef1->setPrev(he2);
			hef1->setNext(he1);		he1->setPrev(hef1);
		}
		else
		{
			MHalfedge* he1_prev = he1->prev();
			het1->setPrev(he1_prev);	he1_prev->setNext(het1);
			he1->setPrev(het1);			het1->setNext(he1);
		}

		if (v1->halfEdge() == he1)
			v1->setHalfedge(het1);

		v0->adjustOutgoingHalfedge();
		v1->adjustOutgoingHalfedge();
		vt->adjustOutgoingHalfedge();
		return vt;
	}

	void PolyMesh::collpaseEdge(MHalfedge* he)
	{
		MHalfedge* he0 = he;
		MHalfedge* he0_next = he->next();
		MHalfedge* he0_prev = he->prev();

		MHalfedge* he1 = he->pair();
		MHalfedge* he1_next = he1->next();
		MHalfedge* he1_prev = he1->prev();

		MPolyFace* f0 = he0->polygon();
		MPolyFace* f1 = he1->polygon();

		MVert* vb = he0->fromVertex();
		MVert* ve = he0->toVertex();

		std::vector<MHalfedge*> vbhe = vertAdjacentHalfEdge(vb);
		for (size_t i = 0; i < vbhe.size(); i++)
		{
			vbhe[i]->setVert(ve);
		}

		he0_prev->setNext(he0_next);	he0_next->setPrev(he0_prev);
		he1_prev->setNext(he1_next);	he1_next->setPrev(he1_prev);

		if (f0 != nullptr)
		{
			f0->setHalfedge(he0_next);
		}
		if (f1 != nullptr)
		{
			f1->setHalfedge(he1_next);
		}

		if (ve->halfEdge() == he1)
			ve->setHalfedge(he0_next);

		vb->setHalfedge(nullptr);

		MEdge* e = he0->edge();
		deleteEdge_fromMesh(e);
		deleteHalfEdge_fromMesh(he0);
		deleteHalfEdge_fromMesh(he1);
		deleteVertex_fromMesh(vb);

	}

	void PolyMesh::collpaseLoop(MHalfedge* he)
	{
		MHalfedge* he0 = he;
		MHalfedge* he1 = he0->next();

		MHalfedge* he0_p = he0->pair();
		MHalfedge* he1_p = he1->pair();

		MVert* v0 = he0->toVertex();
		MVert* v1 = he1->toVertex();

		MPolyFace* f0 = he0->polygon();
		MPolyFace* f1 = he0_p->polygon();

		// is it a loop ?
		assert(he1->next() == he0 && he1 != he0_p);

		MHalfedge* hf0p_next = he0_p->next();
		MHalfedge* hf0p_prev = he0_p->prev();

		he1->setNext(hf0p_next); hf0p_next->setPrev(he1);
		hf0p_prev->setNext(he1); he1->setPrev(hf0p_prev);

		he1->setPolygon(f1);

		v0->setHalfedge(he1);		v0->adjustOutgoingHalfedge();
		v1->setHalfedge(he1_p);		v1->adjustOutgoingHalfedge();

		if (f1 != nullptr && f1->halfEdge() == he0_p)
			f1->setHalfedge(he1);

		if (f0 != nullptr)
		{
			f0->setHalfedge(nullptr);
			deletePolyFace_fromMesh(f0);
		}
		deleteEdge_fromMesh(he0->edge());
		deleteHalfEdge_fromMesh(he0);
		deleteHalfEdge_fromMesh(he0_p);
	}

	void PolyMesh::collapse(MHalfedge* he)
	{
		MHalfedge* he0 = he;
		MHalfedge* he1 = he->pair();
		MHalfedge* he0_next = he0->next();
		MHalfedge* he1_next = he1->next();
		
		collpaseEdge(he);

		// remove loops
		if (he0_next->next()->next() == he0_next)
			collpaseLoop(he0_next->next());
		if (he1_next->next()->next() == he1_next)
			collpaseLoop(he1_next->next());
	}

	bool PolyMesh::is_collapse_ok(MHalfedge* v0v1)
	{
		MHalfedge* he0 = v0v1;
		MHalfedge* he1 = he0->pair();

		if (v0v1 == nullptr)
			return false;

		MVert* v0 = he0->fromVertex();
		MVert* v1 = he0->toVertex();

		bool v0v1_triangle = false;
		bool v1v0_triangle = false;

		if (!isBoundary(he0))
		{
			MPolyFace* f0 = he0->polygon();
			if (f0->PolyNum() == 3) v0v1_triangle = true;
		}
		if (!isBoundary(he1))
		{
			MPolyFace* f1 = he1->polygon();
			if (f1->PolyNum() == 3) v1v0_triangle = true;
		}

		MVert* v01n = he0->next()->toVertex();
		MVert* v10n = he1->next()->toVertex();

		if (v01n == nullptr || v10n == nullptr)
		{
			return false;
		}

		MVert* vt = nullptr;
		MVert* vr = nullptr;
		//For a triangular face, the other two edge cannot both be boundary edges
		if (!isBoundary(he0))
		{
			if (v0v1_triangle)
			{
				MHalfedge* het1 = he0->next();
				MHalfedge* het2 = het1->next();

				MEdge* e1 = het1->edge();
				MEdge* e2 = het2->edge();

				vt = het1->toVertex();

				if (isBoundary(e1) && isBoundary(e2))
					return false;
			}
		}

		if (!isBoundary(he1))
		{
			if (v1v0_triangle)
			{
				MHalfedge* het1 = he1->next();
				MHalfedge* het2 = het1->next();

				MEdge* e1 = het1->edge();
				MEdge* e2 = het2->edge();

				vr = het1->toVertex();

				if (isBoundary(e1) && isBoundary(e2))
					return false;
			}
		}

		// if vl and vr are equal and valid -> fail
		// The two faces share points other than the endpoints of this edge
		if (vt!= nullptr && (vt == vr)) return false;

		// edge between two boundary vertices should be a boundary edge
		if (isBoundary(v0) && isBoundary(v1) && !isBoundary(he0) && !isBoundary(he1))
			return false;

		//For triangular mesh, two vertices only share two common points
		std::vector<MVert*> v0_vector = vertAdjacentVertices(v0);
		std::vector<MVert*> v1_vector = vertAdjacentVertices(v1);
		std::vector<MVert*> intersect_vector; intersect_vector.clear(); 
		intersect_vector.reserve(v0_vector.size());
		for (size_t i = 0; i < v0_vector.size(); i++)
		{
			MVert* vv = v0_vector[i];
			if (!(v0v1_triangle && vv == v01n) && !(v0v1_triangle && vv == v10n))
			{
				for (size_t j = 0; j < v1_vector.size(); j++)
				{
					if (vv == v1_vector[j])
						return false;
				}
			}
		}

		//test for a face on the backside/other side that might degenerate
		if (v0v1_triangle)
		{
			MHalfedge* het0 = he0->next();
			MHalfedge* het1 = het0->next();

			MHalfedge* heo0 = het0->pair();
			MHalfedge* heo1 = het1->pair();

			if (heo0->polygon() == heo1->polygon())
				return false;
		}

		if (v1v0_triangle)
		{
			MHalfedge* het0 = he1->next();
			MHalfedge* het1 = het0->next();

			MHalfedge* heo0 = het0->pair();
			MHalfedge* heo1 = het1->pair();

			if (heo0->polygon() == heo1->polygon())
				return false;
		}

		// passed all tests
		return true;
	}

	bool PolyMesh::is_collapse_ok_Triangle(MHalfedge* v0v1)
	{
		MHalfedge* he0 = v0v1;
		MHalfedge* he1 = he0->pair();

		if (v0v1 == nullptr)
			return false;

		MVert* v0 = he0->fromVertex();
		MVert* v1 = he0->toVertex();

		if (v0 == nullptr || v1 == nullptr)
			return false;

		MVert* vt = nullptr;
		MVert* vr = nullptr;
		//For a triangular face, the other two edge cannot both be boundary edges
		if (!isBoundary(he0))
		{
			MHalfedge* het1 = he0->next();
			MHalfedge* het2 = het1->next();

			MEdge* e1 = het1->edge();
			MEdge* e2 = het2->edge();

			vt = het1->toVertex();

			if (isBoundary(e1) && isBoundary(e2))
				return false;
		}

		if (!isBoundary(he1))
		{
			MHalfedge* het1 = he1->next();
			MHalfedge* het2 = het1->next();

			MEdge* e1 = het1->edge();
			MEdge* e2 = het2->edge();

			vr = het1->toVertex();

			if (isBoundary(e1) && isBoundary(e2))
				return false;
		}

		// if vl and vr are equal and valid -> fail
		// The two faces share points other than the endpoints of this edge
		if (vt == vr) return false;

		// edge between two boundary vertices should be a boundary edge
		if (isBoundary(v0) && isBoundary(v1) && !isBoundary(he0) && !isBoundary(he1))
			return false;

		//For triangular mesh, two vertices only share two common points
		std::vector<MVert*> v0_vector = vertAdjacentVertices(v0);
		std::vector<MVert*> v1_vector = vertAdjacentVertices(v1);
		std::vector<MVert*> intersect_vector; intersect_vector.clear();
		intersect_vector.reserve(v0_vector.size());
		for (size_t i = 0; i < v0_vector.size(); i++)
		{
			MVert* vv = v0_vector[i];
			if (!(vv == vt) && !(vv == vr))
			{
				for (size_t j = 0; j < v1_vector.size(); j++)
				{
					if (vv == v1_vector[j])
						return false;
				}
			}
		}

		// passed all tests
		return true;
	}

	void PolyMesh::collapseTriangle(MHalfedge* he)
	{
		collapse(he);
	}

	void PolyMesh::flipEdgeTriangle(MEdge* edge)
	{
		// CAUTION : Flipping a halfedge may result in
		// a non-manifold mesh, hence check for yourself
		// whether this operation is allowed or not!
		assert(is_flip_ok_Triangle(edge));//let's make it sure it is actually checked
		assert(!isBoundary(edge));

		MHalfedge* he0 = edge->halfEdge();
		MHalfedge* he1 = he0->pair();

		MHalfedge* he0_n = he0->next();
		MHalfedge* he1_n = he1->next();

		MHalfedge* he0_nn = he0_n->next();
		MHalfedge* he1_nn = he1_n->next();

		MVert* v0 = he0->fromVertex();
		MVert* v2 = he0_nn->fromVertex();

		MVert* v1 = he1->fromVertex();
		MVert* v3 = he1_nn->fromVertex();

		MPolyFace* f0 = he0->polygon();
		MPolyFace* f1 = he1->polygon();

		he0->setVert(v2);
		he1->setVert(v3);

		he0->setNext(he1_nn);	he1_nn->setPrev(he0);
		he1_nn->setNext(he0_n);	he0_n->setPrev(he1_nn);
		he0_n->setNext(he0);	he0->setPrev(he0_n);

		he1->setNext(he0_nn);	he0_nn->setPrev(he1);
		he0_nn->setNext(he1_n);	he1_n->setPrev(he0_nn);
		he1_n->setNext(he1);	he1->setPrev(he1_n);

		f0->setHalfedge(he0);
		f1->setHalfedge(he1);

		he1_nn->setPolygon(f0);
		he0_nn->setPolygon(f1);

		if (v0->halfEdge() == he0)
			v0->setHalfedge(he1_n);
		if (v1->halfEdge() == he1)
			v1->setHalfedge(he0_n);
	}

	bool PolyMesh::is_flip_ok_Triangle(MEdge* edge)
	{
		if (isBoundary(edge)) return false;

		MHalfedge* he0 = edge->halfEdge();
		MHalfedge* he1 = he0->pair();

		// check if the flipped edge is already present in the mesh

		MVert* va = he0->next()->toVertex();
		MVert* vb = he1->next()->toVertex();

		if (va == vb)   // this is generally a bad sign !!!
			return false;

		for (VertexVertexIter vv_it = vv_iter(va); vv_it.isValid(); ++vv_it)
		{
			if (*vv_it == vb)
				return false;
		}

		return true;
	}

//=====================================================================
// Calculating location API
//=====================================================================

	MPoint3 PolyMesh::calculatFaceCenter(MPolyFace* face)
	{
		if (face != nullptr) return face->getFaceCenter();
		else return MPoint3(0, 0, 0);
	}

	MPoint3 PolyMesh::calculatEdgeCenter(MEdge* edge)
	{
		if (edge != nullptr) return edge->getCenter();
		else return MPoint3(0, 0, 0);
	}

//=====================================================================
// 纹理处理，渲染相关的API
//=====================================================================

	void PolyMesh::add_texture_information(int id, std::string name)
	{
		if (id < 0) return;

		size_t id_ = static_cast<size_t>(id);

		if (texture_name.size() < id_ + 1)
		{
			texture_name.resize(id_ + 1);
		}
		texture_name[id_] = name;
	}

	size_t PolyMesh::getFaceTexcoords(std::vector<Texcoord>& hehandles)
	{
		unsigned int count(0);
		hehandles.clear();
		for (HalfEdgeIter he_it = halfedge_begin(); he_it != halfedge_end(); ++he_it)
		{
			hehandles.push_back((*he_it)->getTexture());
			++count;
		}

		return count;
	}

}//namespace polymesh
}//namespaec acamcad