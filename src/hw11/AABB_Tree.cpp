#include "AABB_Tree\AABB_Tree.h"


AABB::AABB()
{
	min = Vertex(0, 0, 0);
	max = Vertex(0, 0, 0);
}

AABB::AABB(Vertex* vertex_list, int num)
{
	min = vertex_list[0];
	max = min;
	for (int i = 1; i < num; i++)
	{
		if (vertex_list[i][0] < min[0])
			min[0] = vertex_list[i][0];
		if (vertex_list[i][1] < min[1])
			min[1] = vertex_list[i][1];
		if (vertex_list[i][2] < min[2])
			min[2] = vertex_list[i][2];
		if (vertex_list[i][0] > max[0])
			max[0] = vertex_list[i][0];
		if (vertex_list[i][1] > max[1])
			max[1] = vertex_list[i][1];
		if (vertex_list[i][2] > max[2])
			max[2] = vertex_list[i][2];
	}
}

AABB::AABB(std::vector<Vertex>& vertices_list)
{
	min = vertices_list[0];
	max = min;
	for (int i = 1; i < vertices_list.size(); i++)
	{
		if (vertices_list[i][0] < min[0])
			min[0] = vertices_list[i][0];
		if (vertices_list[i][1] < min[1])
			min[1] = vertices_list[i][1];
		if (vertices_list[i][2] < min[2])
			min[2] = vertices_list[i][2];
		if (vertices_list[i][0] > max[0])
			max[0] = vertices_list[i][0];
		if (vertices_list[i][1] > max[1])
			max[1] = vertices_list[i][1];
		if (vertices_list[i][2] > max[2])
			max[2] = vertices_list[i][2];
	}
}

Vertex AABB::getCenter()
{
	return Vertex((min[0] + max[0]) / 2, (min[1] + max[1]) / 2, (min[2] + max[2]) / 2);
}


AABBNode::AABBNode(std::vector<Vertex>& vertexList, std::vector<int>& face_id_list, AABBTreeInfo& treeInfo, int depth)
{
	left = NULL;
	right = NULL;
	m_box = AABB();
	m_pVerts.clear();
	face_id.clear();
	m_nNumVerts = vertexList.size();
	BuildTree(vertexList, face_id_list, treeInfo, depth);
}

int AABBNode::FindBestAxis(std::vector<Vertex>& vertexList)
{
	size_t vertexNum = vertexList.size();

	//divide this box into two boxes - pick a better axis 
	int iAxis = 0;
	int iAxisResult[3]; //stores how close end result is, the lower the better  

	Vertex center = m_box.getCenter();

	for (iAxis = 0; iAxis < 3; iAxis++)
	{
		int left = 0, right = 0;
		Vertex v[3];
		int count = 0;
		for (int i = 0; i < vertexNum; i++)
		{
			v[count] = vertexList[i];
			if (count == 2)
			{
				float faceCenter[3];
				faceCenter[0] = (v[0][0] + v[1][0] + v[2][0]) / 3.0f;
				faceCenter[1] = (v[0][1] + v[1][1] + v[2][1]) / 3.0f;
				faceCenter[2] = (v[0][2] + v[1][2] + v[2][2]) / 3.0f;

				if (faceCenter[iAxis] <= center[iAxis])
				{
					left++;
				}
				else
				{
					right++;
				}

				count = 0;
			}
			else
				count++;
		} // vertices  

		iAxisResult[iAxis] = abs(left - right);
	} //axis  

	int index = 0;
	int result = iAxisResult[0];
	for (int i = 1; i < 3; i++)
	{
		if (iAxisResult[i] < result)
		{
			result = iAxisResult[i];
			index = i;
		}
	}

	return index;
}

void AABBNode::SetBounds(AABB box)
{
	min = box.min;
	max = box.max;
	m_box = box;
}

void AABBNode::BuildTree(std::vector<Vertex>& vertexList, std::vector<int>& face_id_list, AABBTreeInfo& treeInfo, int depth)
{
	int vertexNum = (int)vertexList.size();

	// Build the node bounding box based from the triangle list 
	AABB Box(vertexList);

	//debug box bounds 
	SetBounds(Box);

	if (depth + 1 > treeInfo.curr_max_depth)
		treeInfo.curr_max_depth = depth + 1;

	bool bMakeChildren = false;

	if (vertexNum > treeInfo.min_vertices && depth < treeInfo.max_tree_depth)
	{
		bMakeChildren = true;
	}

	if (bMakeChildren)
	{
		// Find the longest axii of the node's box 
		int iAxis = FindBestAxis(vertexList);
		//Log("Longest axis: %d\n", iAxis);  

		//Get the Arrays for min, max dimensions 
		float* min = &Box.min[0];
		float* max = &Box.max[0];

		//get center of the box for longest axis 
		Vertex center = Box.getCenter();

		int count = 0;
		Vertex v[3];
		std::vector<Vertex> leftSide;
		std::vector<Vertex> rightSide;
		std::vector<int> leftSide_face_id;
		std::vector<int> rightSide_face_id;

		int leftCount = 0, rightCount = 0; //debug  

		//btw, things that could go wrong- if the mesh doesn't 
		//send over the triangles correctly, then you might see 
		//huge boxes that are misaligned (bad leaves). 
		//things to check is making sure the vertex buffer is 
		//correctly aligned along the adjancey buffers, etc 
		for (int i = 0; i < vertexNum; i++)
		{
			v[count] = vertexList[i];

			if (count == 2)
			{
				float faceCenter[3];
				faceCenter[0] = (v[0][0] + v[1][0] + v[2][0]) / 3.0f;
				faceCenter[1] = (v[0][1] + v[1][1] + v[2][1]) / 3.0f;
				faceCenter[2] = (v[0][2] + v[1][2] + v[2][2]) / 3.0f;
				//WVector faceCenter(x,y,z);  

				if (faceCenter[iAxis] <= center[iAxis]) //fSplit 
				{
					//Store the verts to the left. 
					leftSide.push_back(v[0]);
					leftSide.push_back(v[1]);
					leftSide.push_back(v[2]);
					leftSide_face_id.push_back(face_id_list[i / 3]);
					leftCount++;
				}
				else
				{
					//Store the verts to the right. 
					rightSide.push_back(v[0]);
					rightSide.push_back(v[1]);
					rightSide.push_back(v[2]);
					rightSide_face_id.push_back(face_id_list[i / 3]);
					rightCount++;
				}

				count = 0;
			}
			else
				count++;
		}

		if (treeInfo.m_bPrune && (leftCount == 0 || rightCount == 0))
		{
			//okay, now it's time to cheat. we couldn't use 
			//the best axis to split the 
			//box so now we'll resort to brute force hacks.... 
			leftSide.clear();
			rightSide.clear();
			leftSide_face_id.clear();
			rightSide_face_id.clear();

			int leftMaxIndex = vertexNum / 2; //left side  

			int count = 0;
			Vertex v[3];
			for (int i = 0; i < vertexNum; i++)
			{
				v[count] = vertexList[i];
				if (count == 2)
				{
					if (i < leftMaxIndex)
					{
						//left node 
						leftSide.push_back(v[0]);
						leftSide.push_back(v[1]);
						leftSide.push_back(v[2]);
						leftSide_face_id.push_back(face_id_list[i / 3]);
					}
					else
					{
						rightSide.push_back(v[0]);
						rightSide.push_back(v[1]);
						rightSide.push_back(v[2]);
						rightSide_face_id.push_back(face_id_list[i / 3]);
					}

					count = 0;
				}
				else
					count++;
			}
		}

		if (leftSide.size() > 0 && rightSide.size() > 0)
		{
			assert(leftSide.size() % 3 == 0);
			assert(rightSide.size() % 3 == 0);

			//Build child nodes 
			if (leftSide.size() > 0)
			{
				treeInfo.left_children++;
				left = new AABBNode(leftSide, leftSide_face_id, treeInfo, depth + 1);
			}
			if (rightSide.size() > 0)
			{
				treeInfo.right_children++;
				right = new AABBNode(rightSide, rightSide_face_id, treeInfo, depth + 1);
			}
		}
		else
		{
			//should never happen 
			bMakeChildren = false;
		}
	}

	if (!bMakeChildren)
	{
		//Store the data directly if you want.... 
		for (int i = 0; i < vertexList.size(); i++)
		{
			m_pVerts.push_back(vertexList[i]);
		}
		for (int i = 0; i < face_id_list.size(); i++)
		{
			face_id.push_back(face_id_list[i]);
		}

	}
}

AABB_Tree::AABB_Tree(std::vector<Vertex> &vertices_list)
{
	treeInfo.max_tree_depth = 100;
	treeInfo.min_vertices = 72;
	treeInfo.curr_max_depth = 0;
	treeInfo.m_bPrune = true;
	leafNodeNum = 0;
	all_face_num = 0;
	std::vector<int> face_id_list;
	face_id_list.clear();
	for (int i = 0; i < vertices_list.size() / 3; i++)
		face_id_list.push_back(i);

	vertices_list_copy.clear();
	for (int i = 0; i < vertices_list.size(); i++)
		vertices_list_copy.push_back(vertices_list[i]);

	root = new AABBNode(vertices_list, face_id_list,treeInfo, 0);
}

AABB_Tree::~AABB_Tree()
{

}

float AABB_Tree::point_line_distance(Vector3f p, Vector3f a, Vector3f b, Vector3f &nearestP,float ref_dist)
{
	//ref_dist is used for efficiency
	//reduce some computation
	Vector3f ab = b - a;
	Vector3f pa = a - p;
	Vector3f p_inline;
	float point_line_dist;
	float rst_dist;
	ab.Normalize();
	point_line_dist = (pa - pa.Dot(ab)*ab).Length();
	p_inline = p + pa - pa.Dot(ab)*ab;

	if (ref_dist >= 0 && point_line_dist > ref_dist)
	{
		//reduction of computation
		return point_line_dist;
	}
	
	if (inside_segment(p_inline, a, b))
	{
		nearestP = p_inline;
		rst_dist = point_line_dist;
	}
	else
	{
		nearestP = a;
		rst_dist = (a - p).Length();
		if ((b - p).Length() < rst_dist)
		{
			rst_dist = (b - p).Length();
			nearestP = b;
		}
	}
	return rst_dist;
}

bool AABB_Tree::inside_segment(Vector3f &p, Vector3f a, Vector3f b)
{
	Vector3f pa, pb;
	pa = a - p;
	pb = b - p;
	pa.Normalize();
	pb.Normalize();
	if (fabs(fabs(pa.Dot(pb)) - 1) > 1e-6)
		return false;
	if (pa.Dot(pb) < -0.5)
		return true;
	return false;
}

float AABB_Tree::point_AABB_distance(Vector3f p, AABB box)
{
	float minx, miny, minz, maxx, maxy, maxz;
	float x, y, z;
	float mindist;
	x = p[0];
	y = p[1];
	z = p[2];
	minx = box.min[0];
	miny = box.min[1];
	minz = box.min[2];
	maxx = box.max[0];
	maxy = box.max[1];
	maxz = box.max[2];
	Vector3f corner[8];
	if (x >= minx && x <= maxx && y >= miny && y <= maxy && z >= minz && z <= maxz)
	{
		//inside AABB
		return -1;
	}
	mindist = 0;
	if (x < minx)
	{
		mindist += (x - minx)*(x - minx);
	}
	else if (x>maxx)
	{
		mindist += (x - maxx)*(x - maxx);
	}

	if (y < miny)
	{
		mindist += (miny - y)*(miny - y);
	}
	else if (y>maxy)
	{
		mindist += (y - maxy)*(y - maxy);
	}

	if (z < minz)
	{
		mindist += (minz - z)*(minz - z);
	}
	else if (z>maxz)
	{
		mindist += (z - maxz)*(z - maxz);
	}

	return sqrtf(mindist);
}


float AABB_Tree::findNearstPoint(Vector3f p, Vector3f &nearestP)
{
	float global_min_dist;
	Vector3f globalVertex;
	int globalFaceID;
	global_min_dist = (vertices_list_copy[0] - p).Length();
	globalVertex = vertices_list_copy[0];
	globalFaceID = 0;
	
	Traverse_Search(root,p,globalVertex,global_min_dist,globalFaceID);
	nearestP = globalVertex;
	return global_min_dist;
}

void AABB_Tree::Traverse_Search(AABBNode *node,Vector3f &p,Vector3f & globalVertex,float& global_min_dist,int& globalFaceID)
{
	if (node == NULL)
		return;
	if (point_AABB_distance(p, node->m_box) > global_min_dist)
		return;
	if (node->left == NULL && node->right == NULL)
	{
		//search entire node
		search_entire_node(node, p, globalVertex, global_min_dist, globalFaceID);
	}
	if (node->left != NULL)
		Traverse_Search(node->left, p, globalVertex, global_min_dist, globalFaceID);
	if (node->right != NULL)
		Traverse_Search(node->right, p, globalVertex, global_min_dist, globalFaceID);
}

float AABB_Tree::search_entire_node(AABBNode *node, Vector3f &p, Vector3f & globalVertex, float &global_min_dist, int& globalFaceID)
{
	float min_dist,temp_rst;
	int cur_face_id;
	Vector3f min_point;
	Vector3f rst;
	if (node == NULL)
		return -1;
	if (node->m_nNumVerts < 3)
		return -1;
	
	min_dist = point_tri_distance_refine(p, node->m_pVerts[0], node->m_pVerts[1], node->m_pVerts[2], min_point, -1);
	cur_face_id = node->face_id[0];

	for (int i = 1; i < node->m_nNumVerts / 3; i++)
	{
		temp_rst = point_tri_distance_refine(p, node->m_pVerts[3 * i], node->m_pVerts[3 * i + 1], node->m_pVerts[3 * i + 2], rst, min_dist);
		if (temp_rst < min_dist)
		{
			min_dist = temp_rst;
			min_point = rst;
			cur_face_id = node->face_id[i];
		}
	}
	if (min_dist < global_min_dist)
	{
		global_min_dist = min_dist;
		globalVertex = min_point;
		globalFaceID = cur_face_id;
	}
	return global_min_dist;
}

float AABB_Tree::point_tri_distance_refine(Vector3f p, Vector3f a, Vector3f b, Vector3f c, Vector3f &nearestP, float ref_dist)
{
	Vector3f center = Vector3f(a[0] + b[0] + c[0], a[1] + b[1] + c[1], a[2] + b[2] + c[2]);
	Vector3f ab, ac, bc;
	Vector3f nab, nbc, nac;
	Vector3f tri_face_normal, temp, p_projected;
	float nearestDist;
	float dist_to_plane;
	int state_code;
	center /= 3;
	ab = b - a;
	ac = c - a;
	bc = c - b;
	ab.Normalize();
	ac.Normalize();
	bc.Normalize();
	tri_face_normal = ab.Cross(ac);
	tri_face_normal.Normalize();

	temp = a - p;
	dist_to_plane = fabs(temp.Dot(tri_face_normal));
	if (ref_dist >= 0 && ref_dist<dist_to_plane)
	{
		//reduction for efficiency
		return dist_to_plane;
	}

	nab = center - a;
	nbc = center - b;
	nac = center - a;
	nab = nab - nab.Dot(ab)*ab;
	nbc = nbc - nbc.Dot(bc)*bc;
	nac = nac - nac.Dot(ac)*ac;
	nab.Normalize();
	nbc.Normalize();
	nac.Normalize();
	p_projected = p + temp.Dot(tri_face_normal)*tri_face_normal;

	float d1, d2, d3, d4, d5;
	d1 = tri_face_normal.Dot(ab);
	d2 = tri_face_normal.Dot(ac);
	d3 = tri_face_normal.Dot(bc);
	d4 = tri_face_normal.Dot(p_projected - c);
	d5 = tri_face_normal.Dot(p_projected - b);

	state_code = 0;
	if ((p_projected - a).Dot(nab) > 1e-6)
		state_code += 1;
	if ((p_projected - b).Dot(nbc) > 1e-6)
		state_code += 2;
	if ((p_projected - c).Dot(nac) > 1e-6)
		state_code += 4;
	if (state_code == 7)
	{
		//inside triangle
		nearestP = p_projected;
		nearestDist = dist_to_plane;
	}
	else
	{
		//out of triangle
		//find nearest point on edges or vertices
		Vector3f tempP;
		/*switch (state_code)
		{
		case 1:
			nearestDist = point_line_distance(p, a, b, tempP, -1);
			nearestP = tempP;
			break;
		case 2:
			nearestDist = point_line_distance(p, b, c, tempP, -1);
			nearestP = tempP;
			break;
		case 3:
			nearestDist = (p - b).Length();
			nearestP = b;
			break;
		case 4:
			nearestDist = point_line_distance(p, a, c, tempP, -1);
			nearestP = tempP;
			break;
		case 5:
			nearestDist = (p - a).Length();
			nearestP = a;
			break;
		case 6:
			nearestDist = (p - c).Length();
			nearestP = c;
			break;
		default:
			//never goes here
			break;
		}*/
		float local_dist;
		Vector3f local_rst;
		switch (state_code)
		{
		case 1:
			//check ac bc
			nearestDist = point_line_distance(p, a, c, tempP, -1);
			nearestP = tempP;

			local_dist = point_line_distance(p, b, c, local_rst, nearestDist);
			if (local_dist < nearestDist)
			{
				nearestDist = local_dist;
				nearestP = local_rst;
			}
			break;
		case 2:
			//check ab ac
			nearestDist = point_line_distance(p, a, c, tempP, -1);
			nearestP = tempP;

			local_dist = point_line_distance(p, a, b, local_rst, nearestDist);
			if (local_dist < nearestDist)
			{
				nearestDist = local_dist;
				nearestP = local_rst;
			}
			break;
		case 3:
			//check ac
			nearestDist = point_line_distance(p, a, c, tempP, -1);
			nearestP = tempP;
			break;
		case 4:
			//check bc ab
			nearestDist = point_line_distance(p, b, c, tempP, -1);
			nearestP = tempP;

			local_dist = point_line_distance(p, a, b, local_rst, nearestDist);
			if (local_dist < nearestDist)
			{
				nearestDist = local_dist;
				nearestP = local_rst;
			}
			break;
		case 5:
			//check bc
			nearestDist = point_line_distance(p, b, c, tempP, -1);
			nearestP = tempP;
			break;
		case 6:
			//check ab
			nearestDist = point_line_distance(p, a, b, tempP, -1);
			nearestP = tempP;
			break;
		default:
			//never goes here
			//check ab ac bc
			nearestDist = point_line_distance(p, b, c, tempP, -1);
			nearestP = tempP;

			local_dist = point_line_distance(p, a, b, local_rst, nearestDist);
			if (local_dist < nearestDist)
			{
				nearestDist = local_dist;
				nearestP = local_rst;
			}

			local_dist = point_line_distance(p, a, c, local_rst, nearestDist);
			if (local_dist < nearestDist)
			{
				nearestDist = local_dist;
				nearestP = local_rst;
			}
			break;
		}
	}
	return nearestDist;
}