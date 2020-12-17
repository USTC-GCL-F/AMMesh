#include "PolyMesh/PolyMesh.h"
#include "PolyMesh/PolyMeshIterators.h"

namespace acamcad {
namespace polymesh {

//=====================================================================
//VertexOHalfEdgeIter
//=====================================================================
VertexOHalfEdgeIter::VertexOHalfEdgeIter(MVert* vert, const PolyMesh* _mesh) :
	BaseIter(_mesh, vert)
{
	ref_ = vert->halfEdge();
	original_ = vert->halfEdge();
}

VertexOHalfEdgeIter& VertexOHalfEdgeIter::operator++()
{
	ref_ = ref_->rotateNext();
	if (ref_ == original_) ref_ = nullptr;

	return *this;
}

VertexOHalfEdgeIter& VertexOHalfEdgeIter::operator--()
{
	ref_ = ref_->rotatePrev();
	if (ref_ == original_) ref_ = nullptr;

	return *this;
}


//=====================================================================
//VertexVertexIter
//=====================================================================
VertexVertexIter::VertexVertexIter(MVert* vert, const PolyMesh* _mesh) :
	BaseIter(_mesh, vert)
{
	voh_ = vert->halfEdge();
	original_ = vert->halfEdge();
	ref_ = vert->halfEdge()->toVertex();
}

VertexVertexIter& VertexVertexIter::operator++()
{
	voh_ = voh_->rotateNext();
	ref_ = voh_->toVertex();
	if (voh_ == original_) ref_ = nullptr;

	return *this;
}

VertexVertexIter& VertexVertexIter::operator--()
{
	voh_ = voh_->rotatePrev();
	ref_ = voh_->toVertex();
	if (voh_ == original_) ref_ = nullptr;

	return *this;
}


//=====================================================================
//VertexEdgeIter
//=====================================================================
VertexEdgeIter::VertexEdgeIter(MVert* vert, const PolyMesh* _mesh) :
	BaseIter(_mesh, vert)
{
	voh_ = vert->halfEdge();
	original_ = vert->halfEdge();
	ref_ = vert->halfEdge()->edge();
}

VertexEdgeIter& VertexEdgeIter::operator++()
{
	voh_ = voh_->rotateNext();
	ref_ = voh_->edge();
	if (voh_ == original_) ref_ = nullptr;

	return *this;
}

VertexEdgeIter& VertexEdgeIter::operator--()
{
	voh_ = voh_->rotatePrev();
	ref_ = voh_->edge();
	if (voh_ == original_) ref_ = nullptr;

	return *this;
}


//=====================================================================
//VertexFaceIter
//=====================================================================
VertexFaceIter::VertexFaceIter(MVert* vert, const PolyMesh* _mesh) :
	BaseIter(_mesh, vert)
{
	voh_ = vert->halfEdge();
	original_ = vert->halfEdge();
	ref_ = voh_->polygon();
	while (ref_ == nullptr)
	{
		voh_ = voh_->rotateNext();
		ref_ = voh_->polygon();
		if (voh_ == original_)
		{
			ref_ = nullptr;
			break;
		}
	}
}

VertexFaceIter& VertexFaceIter::operator++()
{
	voh_ = voh_->rotateNext();
	ref_ = voh_->polygon();
	while (ref_ == nullptr && voh_ != original_)
	{
		voh_ = voh_->rotateNext();
		ref_ = voh_->polygon();
	}
	if (voh_ == original_) ref_ = nullptr;
	return *this;
}

VertexFaceIter& VertexFaceIter::operator--()
{
	voh_ = voh_->rotatePrev();
	ref_ = voh_->polygon();

	while (ref_ == nullptr && voh_ != original_)
	{
		voh_ = voh_->rotatePrev();
		ref_ = voh_->polygon();
	}
	if (voh_ == original_) ref_ = nullptr;

	return *this;
}


//=====================================================================
//FaceHalfEdgeIter
//=====================================================================
FaceHalfEdgeIter::FaceHalfEdgeIter(MPolyFace* poly_face, const PolyMesh* _mesh) :
	BaseIter(_mesh, poly_face)
{
	ref_ = poly_face->halfEdge();
	original_ = poly_face->halfEdge();
}

FaceHalfEdgeIter& FaceHalfEdgeIter::operator++()
{
	ref_ = ref_->next();
	if (ref_ == original_) ref_ = nullptr;

	return *this;
}

FaceHalfEdgeIter& FaceHalfEdgeIter::operator--()
{
	ref_ = ref_->prev();
	if (ref_ == original_) ref_ = nullptr;

	return *this;
}


//=====================================================================
//FaceVertexIter
//=====================================================================
FaceVertexIter::FaceVertexIter(MPolyFace* poly_face, const PolyMesh* _mesh) :
	BaseIter(_mesh, poly_face)
{
	poly_he_ = poly_face->halfEdge();
	original_ = poly_face->halfEdge();
	ref_ = poly_he_->toVertex();
}

FaceVertexIter& FaceVertexIter::operator++()
{
	poly_he_ = poly_he_->next();
	ref_ = poly_he_->toVertex();
	if (poly_he_ == original_) ref_ = nullptr;

	return *this;
}

FaceVertexIter& FaceVertexIter::operator--()
{
	poly_he_ = poly_he_->prev();
	ref_ = poly_he_->toVertex();
	if (poly_he_ == original_) ref_ = nullptr;

	return *this;
}


//=====================================================================
//FaceEdgeIter
//=====================================================================
FaceEdgeIter::FaceEdgeIter(MPolyFace* poly_face, const PolyMesh* _mesh) :
	BaseIter(_mesh, poly_face)
{
	poly_he_ = poly_face->halfEdge();
	original_ = poly_face->halfEdge();
	ref_ = poly_he_->edge();
}

FaceEdgeIter& FaceEdgeIter::operator++()
{
	poly_he_ = poly_he_->next();
	ref_ = poly_he_->edge();
	if (poly_he_ == original_) ref_ = nullptr;

	return *this;
}

FaceEdgeIter& FaceEdgeIter::operator--()
{
	poly_he_ = poly_he_->prev();
	ref_ = poly_he_->edge();
	if (poly_he_ == original_) ref_ = nullptr;

	return *this;
}


//=====================================================================
//FaceEdgeIter
//=====================================================================
FaceFaceIter::FaceFaceIter(MPolyFace* poly_face, const PolyMesh* _mesh) :
	BaseIter(_mesh, poly_face)
{
	poly_he_ = poly_face->halfEdge();
	original_ = poly_face->halfEdge();
	ref_ = poly_he_->pair()->polygon();
	while (ref_ == nullptr)
	{
		poly_he_ = poly_he_->next();
		ref_ = poly_he_->pair()->polygon();
		if (poly_he_ == original_)
		{
			ref_ = nullptr;
			break;
		}
	}
}

FaceFaceIter& FaceFaceIter::operator++()
{
	poly_he_ = poly_he_->next();
	ref_ = poly_he_->pair()->polygon();

	while (ref_ == nullptr && poly_he_ != original_)
	{
		poly_he_ = poly_he_->next();
		ref_ = poly_he_->pair()->polygon();
	}

	if (poly_he_ == original_) ref_ = nullptr;

	return *this;
}

FaceFaceIter& FaceFaceIter::operator--()
{
	poly_he_ = poly_he_->prev();
	ref_ = poly_he_->pair()->polygon();

	while (ref_ == nullptr && poly_he_ != original_)
	{
		poly_he_ = poly_he_->prev();
		ref_ = poly_he_->pair()->polygon();
	}

	if (poly_he_ == original_) ref_ = nullptr;

	return *this;
}

}//namespace polymesh
}//namespace acamcad