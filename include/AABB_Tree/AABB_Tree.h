#pragma once

/*
This Class AABB_Tree is for fast point-triangle soup nearest distance query

AABB_Tree(std::vector<Vertex> &vertices_list);

In AABB_Tree vertices_list is a vector size=3n store n triangle vertices(tri_1_p1,tri_1_p2,tri_1_p3,tri_2_p1...tri_n_p3)

float findNearstPoint(Vector3f p, Vector3f &nearestP,int *face_id=NULL);

where p is query point, and nearest point will be stored in nearestP, if face_id not equal to NULL, the corresponding primitive id of nearest point will be 
stored in face_id(id order follow the vertices_list storage order form 0 to n-1)
*/

#include "TinyVector.h"
#include <vector>

typedef Vector3f Vertex;
struct  AABB
{
	Vertex min, max;
	AABB();
	AABB(Vertex* vertex_list, int num);
	AABB(std::vector<Vertex>& vertices_list);
	Vertex getCenter();
};

struct AABBTreeInfo
{
	//heuristics 
	int max_tree_depth;		//max depth the tree can reach 
	//min number of vertices each leaf can store 
	int min_vertices;

	int curr_max_depth;
	//ensures that an AABB is never generated that 
	//is over min_vertices. Normally, this algorithm 
	//is not required because the best axis 
	//algorithm normally produces a perfectly balanced tree. 
	bool m_bPrune;
	int left_children, right_children;
};

struct AABBNode
{
	Vector3f min, max;
	AABBNode *left, *right;
	std::vector<Vector3f> m_pVerts;	//vertices stored within this node 
	std::vector<int> face_id;
	unsigned int m_nNumVerts; //num of vertices stored (3->n) 
	AABB m_box;

	AABBNode::AABBNode(std::vector<Vertex>& vertexList, std::vector<int>& face_id_list, AABBTreeInfo& treeInfo, int depth);
	int AABBNode::FindBestAxis(std::vector<Vertex>& vertexList);
	void AABBNode::SetBounds(AABB box);
	void AABBNode::BuildTree(std::vector<Vertex>& vertexList, std::vector<int>& face_id_list, AABBTreeInfo& treeInfo, int depth);
};


class AABB_Tree
{
public:
	AABB_Tree(std::vector<Vertex> &vertices_list);
	~AABB_Tree();
	float findNearstPoint(Vector3f p, Vector3f &nearestP);

private:
	AABBNode *root;
	AABBTreeInfo treeInfo;
	int leafNodeNum;
	int all_face_num;
	std::vector<Vertex> vertices_list_copy;

	float point_line_distance(Vector3f p, Vector3f a, Vector3f b, Vector3f &nearestP, float ref_dist);
	float point_tri_distance_refine(Vector3f p, Vector3f a, Vector3f b, Vector3f c, Vector3f &nearestP, float ref_dist);
	float point_AABB_distance(Vector3f p, AABB box);
	bool inside_segment(Vector3f &p, Vector3f a, Vector3f b);
	
	void Traverse_Search(AABBNode *,Vector3f &,Vector3f&,float&,int&);
	float search_entire_node(AABBNode *, Vector3f &,Vector3f&,float&,int&);	
	
};

