#pragma once

#include <vector>
#include "PolyMesh_Base.h"

namespace acamcad {
namespace polymesh {

class PolyMesh;

template<typename IH, typename OH>
class BaseIterator
{
public:
	const PolyMesh* mesh_;

protected:
	IH cur_;
	OH ref_;

public:
	BaseIterator(const PolyMesh* mesh, const IH in, const OH out) : mesh_(mesh), cur_(in), ref_(out) {
#ifdef DEBUG
		assert(cur_ != nullptr)
#endif // DEBUG
	}

	BaseIterator(const PolyMesh* mesh, const IH in) : mesh_(mesh), cur_(in), ref_(nullptr) {
#ifdef DEBUG
		assert(cur_ != nullptr)
#endif // DEBUG
	}

	BaseIterator() : mesh_(nullptr), cur_(nullptr), ref_(nullptr) {
	}
	virtual ~BaseIterator() {}

	bool operator == (const BaseIterator& rhs) const {
		return (this->cur_ == rhs.cur_() && this->ref_ == rhs.ref_ && this->mesh_ == rhs.mesh_);
	}
	bool operator != (const BaseIterator& rhs) const {
		return !this->operator==(rhs);
	}

	const OH operator*() const {
		return ref_;
	}

	BaseIterator& operator=(const BaseIterator& rhs) {
		this->cur_ = rhs.cur_pointer();
		this->ref_handle_ = rhs.ref_pointer();
		this->mesh_ = rhs.mesh();
		return *this;
	}

	const OH cur_pointer() const {
		return cur_;
	}
	const IH ref_pointer() const {
		return ref_;
	}
	const PolyMesh* mesh() const {
		return mesh_;
	}

	bool isValid() const {
		return ref_ != nullptr;
	}
};

/**
* Iterate vertex outgoing halfedge
*/
class VertexOHalfEdgeIter : public BaseIterator< MVert*, MHalfedge* >
{
public:
	typedef BaseIterator< MVert*, MHalfedge*> BaseIter;

	VertexOHalfEdgeIter(MVert* vert, const PolyMesh* _mesh);

	VertexOHalfEdgeIter& operator++();
	VertexOHalfEdgeIter& operator--();

	VertexOHalfEdgeIter operator++(int) {
		VertexOHalfEdgeIter cpy = *this;
		++(*this);
		return cpy;
	}
	VertexOHalfEdgeIter operator--(int) {
		VertexOHalfEdgeIter cpy = *this;
		--(*this);
		return cpy;
	}

private:
	MHalfedge* original_;
};
/**
* Iterate vertex neighborhood
*/
class VertexVertexIter : public BaseIterator< MVert*, MVert* >
{
public:
	typedef BaseIterator< MVert*, MVert*> BaseIter;

	VertexVertexIter(MVert* vert, const PolyMesh* _mesh);

	VertexVertexIter& operator++();
	VertexVertexIter& operator--();

	VertexVertexIter operator++(int) {
		VertexVertexIter cpy = *this;
		++(*this);
		return cpy;
	}
	VertexVertexIter operator--(int) {
		VertexVertexIter cpy = *this;
		--(*this);
		return cpy;
	}

private:
	MHalfedge* voh_;
	MHalfedge* original_;
};
/**
* Iterate vertex neighboring edges
*/
class VertexEdgeIter : public BaseIterator< MVert*, MEdge* >
{
public:
	typedef BaseIterator< MVert*, MEdge*> BaseIter;

	VertexEdgeIter(MVert* vert, const PolyMesh* _mesh);

	VertexEdgeIter& operator++();
	VertexEdgeIter& operator--();

	VertexEdgeIter operator++(int) {
		VertexEdgeIter cpy = *this;
		++(*this);
		return cpy;
	}
	VertexEdgeIter operator--(int) {
		VertexEdgeIter cpy = *this;
		--(*this);
		return cpy;
	}

private:
	MHalfedge* voh_;
	MHalfedge* original_;
};
/**
* Iterate vertex neighboring faces
*/
class VertexFaceIter : public BaseIterator< MVert*, MPolyFace* >
{
public:
	typedef BaseIterator< MVert*, MPolyFace*> BaseIter;

	VertexFaceIter(MVert* vert, const PolyMesh* _mesh);

	VertexFaceIter& operator++();
	VertexFaceIter& operator--();

	VertexFaceIter operator++(int) {
		VertexFaceIter cpy = *this;
		++(*this);
		return cpy;
	}
	VertexFaceIter operator--(int) {
		VertexFaceIter cpy = *this;
		--(*this);
		return cpy;
	}

private:
	MHalfedge* voh_;
	MHalfedge* original_;
};


/**
* Iterate face halfedges
*/
class FaceHalfEdgeIter : public BaseIterator< MPolyFace*, MHalfedge* >
{
public:
	typedef BaseIterator< MPolyFace*, MHalfedge*> BaseIter;

	FaceHalfEdgeIter(MPolyFace* poly_face, const PolyMesh* _mesh);

	FaceHalfEdgeIter& operator++();
	FaceHalfEdgeIter& operator--();

	FaceHalfEdgeIter operator++(int) {
		FaceHalfEdgeIter cpy = *this;
		++(*this);
		return cpy;
	}
	FaceHalfEdgeIter operator--(int) {
		FaceHalfEdgeIter cpy = *this;
		--(*this);
		return cpy;
	}

private:
	MHalfedge* original_;
};
/**
* Iterate face vertices
*/
class FaceVertexIter : public BaseIterator< MPolyFace*, MVert* >
{
public:
	typedef BaseIterator< MPolyFace*, MVert*> BaseIter;

	FaceVertexIter(MPolyFace* poly_face, const PolyMesh* _mesh);

	FaceVertexIter& operator++();
	FaceVertexIter& operator--();

	FaceVertexIter operator++(int) {
		FaceVertexIter cpy = *this;
		++(*this);
		return cpy;
	}
	FaceVertexIter operator--(int) {
		FaceVertexIter cpy = *this;
		--(*this);
		return cpy;
	}

private:
	MHalfedge* poly_he_;
	MHalfedge* original_;
};
/**
* Iterate face edges
*/
class FaceEdgeIter : public BaseIterator< MPolyFace*, MEdge* >
{
public:
	typedef BaseIterator< MPolyFace*, MEdge*> BaseIter;

	FaceEdgeIter(MPolyFace* poly_face, const PolyMesh* _mesh);

	FaceEdgeIter& operator++();
	FaceEdgeIter& operator--();

	FaceEdgeIter operator++(int) {
		FaceEdgeIter cpy = *this;
		++(*this);
		return cpy;
	}
	FaceEdgeIter operator--(int) {
		FaceEdgeIter cpy = *this;
		--(*this);
		return cpy;
	}

private:
	MHalfedge* poly_he_;
	MHalfedge* original_;
};
/**
*  Iterate face neighboring faces
*/
class FaceFaceIter : public BaseIterator< MPolyFace*, MPolyFace* >
{
public:
	typedef BaseIterator< MPolyFace*, MPolyFace*> BaseIter;

	FaceFaceIter(MPolyFace* poly_face, const PolyMesh* _mesh);

	FaceFaceIter& operator++();
	FaceFaceIter& operator--();

	FaceFaceIter operator++(int) {
		FaceFaceIter cpy = *this;
		++(*this);
		return cpy;
	}
	FaceFaceIter operator--(int) {
		FaceFaceIter cpy = *this;
		--(*this);
		return cpy;
	}

private:
	MHalfedge* poly_he_;
	MHalfedge* original_;
};

}//namespace polymesh
}//namespace acamcad