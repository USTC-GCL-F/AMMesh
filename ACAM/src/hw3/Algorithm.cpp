#include "Algorithm.h"

//计算数组lmk之间的点的路径
void Dijkstra(PolyMesh& Mesh, int s_p, int e_p, std::vector<int>& path)
{
	std::priority_queue<node> que;

	std::vector<double> distance(Mesh.numVertices(), DBL_MAX);
	std::vector<int> v_p(Mesh.numVertices(), -1);
	std::vector<int> is_visited(Mesh.numVertices(), 0);
	v_p[s_p] = s_p;
	distance[s_p] = 0;
	que.push(node(s_p, 0));

	while (!que.empty())
	{
		node tmp = que.top();
		que.pop();
		if (is_visited[tmp.id])
			continue;
		if (tmp.id == e_p)
			break;
		is_visited[tmp.id] = 1;
		MVert* v1 = Mesh.vert(tmp.id);
		for (VertexVertexIter vv_it = Mesh.vv_iter(v1); vv_it.isValid(); ++vv_it)
		{
			MVert* v2 = *vv_it;
			MEdge* e12 = Mesh.edgeBetween(v1, v2);
			if (distance[v1->index()] + e12->length() < distance[v2->index()])
			{
				distance[v2->index()] = distance[v1->index()] + e12->length();
				que.push(node(v2->index(), distance[v2->index()]));
				v_p[v2->index()] = tmp.id;
			}
		}
	}
	path.clear();
	path.push_back(e_p);
	do
	{
		e_p = v_p[e_p];
		path.push_back(e_p);
	} while (e_p != v_p[e_p]);
}

void Dijkstra_group(PolyMesh& Mesh, std::vector<int>& lmk, std::vector<std::vector<int>>& path)
{
	std::sort(lmk.begin(), lmk.end());
	std::priority_queue<PathInfo> complete_graph;
	//#pragma omp parallel for
	for (int i = 0; i < lmk.size() - 1; i++)
	{
		std::vector<int> is_lmk(Mesh.numVertices(), 0);
		for (int j = i; j < lmk.size(); j++)
			is_lmk[lmk[j]] = 1;
		int count = lmk.size() - i;
		int s_p = lmk[i];//start_point
		std::vector<int> is_visited(Mesh.numVertices(), 0);
		std::vector<double> distance(Mesh.numVertices(), DBL_MAX);
		std::priority_queue<node> que;
		std::vector<int> v_p(Mesh.numVertices(), -1);
		v_p[s_p] = s_p;
		distance[s_p] = 0;
		que.push(node(s_p, 0));

		while (count != 0)
		{
			node tmp = que.top();
			que.pop();
			if (is_visited[tmp.id] == 1)
			{
				continue;
			}
			if (is_lmk[tmp.id] == 1)
			{
				count--;
				is_lmk[tmp.id] = 0;
			}
			is_visited[tmp.id] = 1;
			MVert* v1 = Mesh.vert(tmp.id);
			for (VertexVertexIter vv_it = Mesh.vv_iter(v1); vv_it.isValid(); ++vv_it)
			{
				MVert* v2 = *vv_it;
				MEdge* e12 = Mesh.edgeBetween(v1, v2);
				if (distance[v1->index()] + e12->length() < distance[v2->index()])
				{
					distance[v2->index()] = distance[v1->index()] + e12->length();
					que.push(node(v2->index(), distance[v2->index()]));
					v_p[v2->index()] = tmp.id;
				}
			}
		}
		for (int j = i + 1; j < lmk.size(); j++)
		{
			std::vector<int> p;
			int e_p = lmk[j];
			double l = distance[e_p];
			p.push_back(e_p);
			do
			{
				e_p = v_p[e_p];
				p.push_back(e_p);
			} while (e_p != v_p[e_p]);
			PathInfo p_info(s_p, lmk[j], l);
			p_info.path = p;
			complete_graph.push(p_info);
		}
	}
	std::vector<PathInfo> Mst;
	int nv = lmk.size();
	std::vector<int> spanning_tree_current(nv);
	for (int i = 0; i < nv; ++i)
	{
		spanning_tree_current[i] = i;
	}
	int index = 0;
	int j = 0;
	while (index < nv - 1)
	{
		PathInfo tmp = complete_graph.top();
		complete_graph.pop();
		int m_id = -1, n_id = -1;
		for (int u = 0; u < nv; u++)
		{
			if (lmk[u] == tmp.s_p)
				m_id = u;
			if (lmk[u] == tmp.e_p)
				n_id = u;
		}
		m_id = spanning_tree_current[m_id];
		n_id = spanning_tree_current[n_id];
		if (m_id < n_id)
			std::swap(m_id, n_id);
		if (m_id != n_id)
		{
			path.push_back(tmp.path);
			index++;
			for (int i = 0; i < nv; ++i)
			{
				if (spanning_tree_current[i] == n_id)
					spanning_tree_current[i] = m_id;
			}
		}
	}
}