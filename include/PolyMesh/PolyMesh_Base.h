#pragma once

#include "../Math/MPoint3.h"
#include "../Math/MVector3.h"
#include "../Math/Mrgb.h"

#include "assert.h"

namespace acamcad {
namespace polymesh{
	
class MVert;
class MEdge;
class MHalfedge;
class MPolyFace;

/**
* the class of Vert in Half_edge structure
* all index is begin with 0;
*/
class MVert
{
private:
	int index_;
	bool flag_show_;
	bool flag_select_;

	float color_[4];	
	float texcoord_[3];

	MPoint3 point_;
	MVector3 normal_;

	MHalfedge* he_;
public:
	MVert() : index_(-1), point_(0, 0, 0), normal_(0, 0, 0), he_(nullptr), 
		flag_show_(false), flag_select_(false) 
	{
		color_[0] = 0; color_[1] = 0; color_[2] = 0; color_[3] = 0;
		texcoord_[0] = 0; texcoord_[1] = 0; texcoord_[2] = 0;
	}
	MVert(double x, double y, double z) : index_(-1), point_(x, y, z), normal_(0, 0, 0), he_(nullptr), flag_show_(false), flag_select_(false)
	{
		color_[0] = 0; color_[1] = 0; color_[2] = 0; color_[3] = 0;
		texcoord_[0] = 0; texcoord_[1] = 0; texcoord_[2] = 0;
	}
	~MVert() { index_ = -1; }

public:
	MHalfedge* const halfEdge() { return he_; }
	const MHalfedge* const halfEdge() const { return he_; }

	void setHalfedge(MHalfedge* he) { he_ = he; }

	void setPosition(MPoint3 new_point) { point_ = new_point; }
	void setPosition(double x, double y, double z) { point_ = MPoint3(x, y, z); }
	MPoint3 position() { return point_; }
	const MPoint3& position() const { return point_; }

	double x() const { return point_.x(); }
	double y() const { return point_.y(); }
	double z() const { return point_.z(); }
	double nx() const { return normal_.x(); }
	double ny() const { return normal_.y(); }
	double nz() const { return normal_.z(); }

	void setNormal(MVector3& new_vec) { normal_ = new_vec; }
	void setNormal(double nx, double ny, double nz) { normal_ = MVector3(nx, ny, nz); }
	MVector3 normal() { return normal_; }

	int index() const { return index_; }
	void set_index(int index) { index_ = index; }

	bool isVisibility() { return flag_show_; }
	void setVisibility(bool visi) { flag_show_ = visi; }

	bool isSelected() { return flag_select_; }
	void setSelected(bool select) { flag_show_ ? flag_select_ = select : flag_select_ = false; }

	//The default setting of color is all 0.
	MRGBAf getColor_with_alpha() { return MRGBAf(color_[0], color_[1], color_[2], color_[3]); }
	MRGBf getColor() { return MRGBf(color_[0], color_[1], color_[2]); }
	
	//void setColor(float a[]) { color_[0] = a[0]; color_[1] = a[1]; color_[2] = a[2]; }
	//void setColor_with_alpha(float a[]) { color_[0] = a[0]; color_[1] = a[1]; color_[2] = a[2], color_[3] = a[3]; }

	void setColor(float r, float g, float b) { color_[0] = r; color_[1] = g; color_[2] = b; }
	void setColor(float r, float g, float b, float a) { color_[0] = r; color_[1] = g; color_[2] = b; color_[3] = a; }

	void setColor(const MRGBf& c) { color_[0] = c.r; color_[1] = c.g; color_[2] = c.b; }
	void setColor(const MRGBAf& c) { color_[0] = c.r; color_[1] = c.g; color_[2] = c.b; color_[3] = c.a; }

	void setTexture(float u, float v) { texcoord_[0] = u, texcoord_[1] = v; }
	void setTexture(float u, float v, float w) { texcoord_[0] = u, texcoord_[1] = v; texcoord_[2] = w; }
	void setTexture(Texcoord& t) { texcoord_[0] = t[0]; texcoord_[1] = t[1]; texcoord_[2] = t[2]; }

	//The default setting of Texture is all 0.
	Texcoord getTexture() { return Texcoord(texcoord_[0], texcoord_[1]); }
	Texcoord getTextureUVW() { return Texcoord(texcoord_[0], texcoord_[1], texcoord_[2]); }

	bool isIsolated() const { return he_ == nullptr; }

	//Try to set the half of the point as the boundary half. Be sure call it before adding faces by yourself.
	void adjustOutgoingHalfedge();
};

/**
* the class of Edge in Half_edge structure;
* the halfedge index is not from the edge index
* One can not select a halfedge or see a halfedge;
*/
class MHalfedge
{
private:
	int index_;

	MVert* v_;
	MEdge* e_;
	MPolyFace* poly_face_;

	MHalfedge* next_, * prev_;

	MHalfedge* pair_;

	float texcoord_[3];
public:
	MHalfedge() : index_(-1), next_(nullptr), prev_(nullptr), pair_(nullptr), 
		v_(nullptr), e_(nullptr), poly_face_(nullptr) 
	{
		texcoord_[0] = 0; texcoord_[1] = 0; texcoord_[2] = 0;
	}
	MHalfedge(MHalfedge* next, MHalfedge* prev, MHalfedge* pair, MVert* v, MEdge* e, MPolyFace* p)
		: index_(-1), next_(next), prev_(prev), pair_(pair), v_(v), e_(e), poly_face_(p) 
	{
		texcoord_[0] = 0; texcoord_[1] = 0; texcoord_[2] = 0;
	}

	~MHalfedge() { index_ = -1; };
public:
	MHalfedge* const next() { return next_; }
	MHalfedge* const prev() { return prev_; }
	MHalfedge* const pair() { return pair_; }
	MVert* const fromVertex() { return v_; }
	MVert* const toVertex() { return next()->fromVertex(); }
	MEdge* const edge() { return e_; }
	MPolyFace* const polygon() { return poly_face_; }

	MHalfedge* const rotateNext() { return pair()->next(); }
	MHalfedge* const rotatePrev() { return prev()->pair(); }

	const MHalfedge* const next() const { return next_; }
	const MHalfedge* const prev() const { return prev_; }
	const MHalfedge* const pair() const { return pair_; }
	const MVert* const fromVertex() const { return v_; }
	const MVert* const toVertex() const { return next()->fromVertex(); }
	const MEdge* const edge() const { return e_; }
	const MPolyFace* const polygon() const { return poly_face_; }

	void setNext(MHalfedge* next) { next_ = next; }
	void setPrev(MHalfedge* prev) { prev_ = prev; }
	void setPair(MHalfedge* pair) { pair_ = pair; }
	void setVert(MVert* vert) { v_ = vert; }
	void setEdge(MEdge* edge) { e_ = edge; }
	void setPolygon(MPolyFace* poly) { poly_face_ = poly; }

	bool isBoundary() const { return poly_face_ == nullptr; }

	int index() { return index_; }
	//int edge_index() { return index_ / 2; }
	void set_index(int index) { index_ = index; }

	///get the direction of the edge, from fromVectex to toVertex;
	MVector3 tangent()
	{
		MVector3 t = toVertex()->position() - fromVertex()->position();
		t.normalize();
		return t;
	}

	void setTexture(float u, float v) { texcoord_[0] = u, texcoord_[1] = v; }
	void setTexture(float u, float v, float w) { texcoord_[0] = u, texcoord_[1] = v; texcoord_[2] = w; }
	void setTexture(Texcoord& t) { texcoord_[0] = t[0]; texcoord_[1] = t[1]; texcoord_[2] = t[2]; }

	Texcoord getTexture() { return Texcoord(texcoord_[0], texcoord_[1]); }
	Texcoord getTextureUVW() { return Texcoord(texcoord_[0], texcoord_[1], texcoord_[2]); }
};

/**
* the class of Edge in Half_edge structure
*/
class MEdge
{
private:
	int index_;
	bool flag_show_;
	bool flag_select_;

	MVert* v1_; MVert* v2_;

	MHalfedge* he_;
public:
	MEdge() : index_(-1), v1_(nullptr), v2_(nullptr), he_(nullptr), 
		flag_show_(false), flag_select_(false) {}
	MEdge(MVert* v1, MVert* v2) : index_(-1), v1_(v1), v2_(v2), he_(nullptr), 
		flag_show_(false), flag_select_(false) {}
	MEdge(MVert* v1, MVert* v2, MHalfedge* he) : index_(-1), v1_(v1), v2_(v2), he_(he), 
		flag_show_(false), flag_select_(false) {}

	~MEdge() { index_ = -1; };

public:
	MHalfedge* const halfEdge() { return he_; }
	const MHalfedge* const halfEdge() const { return const_cast<MEdge*>(this)->halfEdge(); }

	void setHalfedge(MHalfedge* he) { he_ = he; }
	void setVert(MVert* v1, MVert* v2) { v1_ = v1; v2_ = v2; }
	void updateVert() { v1_ = he_->fromVertex(), v2_ = he_->toVertex(); }

	int index() const { return index_; }
	void set_index(int index) { index_ = index; }

	bool isVisibility() const { return flag_show_; }
	void setVisibility(bool visi) { flag_show_ = visi; }

	bool isSelected() const { return flag_select_; }
	void setSelected(bool select) { flag_show_ ? flag_select_ = select : flag_select_ = false; }

	///get Vertex of the edge, the 0 is the first, the 1 is the second, the return is not orderd;
	MVert* getVert(int edge_v)
	{
		updateVert();
		if (edge_v == 0) return v1_;
		else if (edge_v == 1) return v2_;
		else return nullptr;
	}

	const MVert* getVert(int edge_v) const 
	{
		return const_cast<MEdge*>(this)->getVert(edge_v);
	}

	double length() 
	{
		updateVert();
		MVector3 t = v1_->position() - v2_->position();
		return t.norm();
	}

	MPoint3 getCenter()
	{
		updateVert();
		return v1_->position() * 0.5 + v2_->position() * 0.5;
	}

	//MPoint3 interpolate(const double& t)
	//{
	//	updateVert();
	//	return v1_->position() * t + v2_->position() * (1 - t);
	//}
	/////get the direction of the edge, from v1 to v2;
	//MVector3 tangent() const
	//{
	//	MVector3 t = v2_->position() - v1_->position();
	//	t.normalize();
	//	return t;
	//}
};

/**
* the class of PolyFace in Half_edge structure;
* all index is begin with 0
*/
class MPolyFace
{
private:
	int index_;
	bool flag_show_;
	bool flag_select_;

	MHalfedge* he_begin_;

	MVector3 normal_;		/* face normal */
	int polynum_;			/* number of vertices in the face */
	int material_index_;	/* material index, 暂时替代OpenMesh中的texindex */
	float color_[4];

public:
	MPolyFace() : index_(-1), he_begin_(nullptr), normal_(0.0), polynum_(0), material_index_(0),
		flag_show_(false), flag_select_(false) 
	{
		color_[0] = 0; color_[1] = 0; color_[2] = 0; color_[3] = 0;
	}
	MPolyFace(MHalfedge* he) : index_(-1), he_begin_(he), normal_(0.0), polynum_(0), material_index_(0),
		flag_show_(false), flag_select_(false) 
	{
		color_[0] = 0; color_[1] = 0; color_[2] = 0; color_[3] = 0;
	}
	~MPolyFace() { index_ = -1; }

public:
	MHalfedge* const halfEdge() { return he_begin_; }
	const MHalfedge* const halfEdge() const { return const_cast<MPolyFace*>(this)->halfEdge(); }

	void setHalfedge(MHalfedge* he) { he_begin_ = he; }

	unsigned int PolyNum() { 
		updatePolyNum();
		return polynum_; 
	}

	//be sure you have updatePoluNum
	unsigned int PolyNum() const { 
		return polynum_;
	}

	void setNormal(MVector3 new_vec) { normal_ = new_vec; }
	void setNormal(double nx, double ny, double nz) { normal_ = MVector3(nx, ny, nz); }
	MVector3 normal() { return normal_; }

	int index() const { return index_; }
	void set_index(int index) { index_ = index; }

	bool isVisibility() const { return flag_show_; }
	void setVisibility(bool visi) { flag_show_ = visi; }

	bool isSelected() const { return flag_select_; }
	void setSelected(bool select) { flag_show_ ? flag_select_ = select : flag_select_ = false; }

	int updatePolyNum();
	void updataNormal();
	void update();				/* update polynum and normal */

	MPoint3 getFaceCenter();

	//The default setting of color is all 0.
	MRGBAf getColor_with_alpha() { return MRGBAf(color_[0], color_[1], color_[2], color_[3]); }
	MRGBf getColor() { return MRGBf(color_[0], color_[1], color_[2]); }

	void setColor(float r, float g, float b) { color_[0] = r; color_[1] = g; color_[2] = b; }
	void setColor(float r, float g, float b, float a) { color_[0] = r; color_[1] = g; color_[2] = b; color_[3] = a; }

	void setColor(MRGBf& c) { color_[0] = c.r; color_[1] = c.g; color_[2] = c.b; }
	void setColor(MRGBAf& c) { color_[0] = c.r; color_[1] = c.g; color_[2] = c.b; color_[3] = c.a; }

	void setMaterialIndex(int id) { material_index_ = id; }
	int meterialIndex() const { return material_index_; }
};

}//polymesh
}//acamcad