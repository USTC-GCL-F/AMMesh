#include "PolyMesh\IOManager.h"
#include <fstream>

#define PI 3.14159265359

using namespace acamcad;
using namespace polymesh;

MVector3 cal_circum_enter(const MVector3& a, const MVector3& b, const MVector3& c)
{
	MVector3 ac = c - a, ab = b - a;
	MVector3 abXac = cross(ab, ac), abXacXab = cross(abXac, ab), acXabXac = cross(ac, abXac);
	return a + (abXacXab * ac.normSq() + acXabXac * ab.normSq()) / (2.0 * abXac.normSq());
}

void cal_local_ave_region(PolyMesh* const mesh, std::vector<double> &vertexLAR)
{
	for (MPolyFace* fh : mesh->polyfaces())
	{
		// judge if it's obtuse
		bool isObtuseAngle = false;
		MVert *obtuseVertexHandle;
		MHalfedge *he = fh->halfEdge();
		MHalfedge *he_next = he->next(), *he_prev = he->prev();
		MVert *v_from_he = he->fromVertex(), *v_from_he_next = he_next->fromVertex(), *v_from_he_prev = he_prev->fromVertex();
		MVector3 vec_he_nor = he->tangent(), vec_he_next_nor = he_next->tangent(), vec_he_prev_nor = he_prev->tangent();
		if (vectorAngle(vec_he_nor, -vec_he_prev_nor) > PI / 2.0)
		{
			isObtuseAngle = true;
			obtuseVertexHandle = v_from_he;
		}
		else if (vectorAngle(vec_he_next_nor, -vec_he_nor) > PI / 2.0)
		{
			isObtuseAngle = true;
			obtuseVertexHandle = v_from_he_next;
		}
		else if (vectorAngle(vec_he_prev_nor, -vec_he_next_nor) > PI / 2.0)
		{
			isObtuseAngle = true;
			obtuseVertexHandle = v_from_he_prev;
		}

		// calculate area
		if (isObtuseAngle)
		{
			double faceArea = 0.5*norm(cross(v_from_he_next->position() - v_from_he->position(), v_from_he_prev->position() - v_from_he->position()));
			for (MVert* fv : mesh->polygonVertices(fh))
			{
				if (fv == obtuseVertexHandle)
					vertexLAR[fv->index()] += faceArea / 2.0;
				else
					vertexLAR[fv->index()] += faceArea / 4.0;
			}
		}
		else
		{
			MVector3 cc = cal_circum_enter(v_from_he->position(), v_from_he_next->position(), v_from_he_prev->position());
			for (MHalfedge* fhh : mesh->polygonHalfedges(fh))
			{
				MVector3 edgeMidpoint = 0.5*(fhh->fromVertex()->position() + fhh->toVertex()->position());
				double edgeLength = fhh->edge()->length();
				double partArea = 0.5 * edgeLength * (edgeMidpoint - cc).norm();
				vertexLAR[fhh->fromVertex()->index()] += 0.5*partArea;
				vertexLAR[fhh->toVertex()->index()] += 0.5*partArea;
			}
		}
	}
}

void cal_mean_curvature(PolyMesh* const mesh, const std::vector<double> &vertexLAR)
{
	std::ofstream	f1("./MeanCurvature.txt");
	if (!f1)
		return;
	std::ofstream	f2("./AbsoluteMeanCurvature.txt");
	if (!f2)
		return;
	for (MVert* vh : mesh->vertices())
	{
		MVector3 p_temp = { 0.0,0.0,0.0 }, p_vh = vh->position();
		for (auto voh_it = mesh->voh_iter(vh); voh_it.isValid(); ++voh_it)
		{
			if (!(*voh_it)->isBoundary())
			{
				MHalfedge* next_voh = (*voh_it)->next();
				MVert* to_voh = (*voh_it)->toVertex(), *to_next_voh = next_voh->toVertex();
				MVector3 p_to_voh = to_voh->position(), p_to_next_voh = to_next_voh->position();
				double angle_voh = vectorAngle(p_vh - p_to_voh, p_to_next_voh - p_to_voh),
					angle_next_voh = vectorAngle(p_to_voh - p_to_next_voh, p_vh - p_to_next_voh);
				p_temp += (p_to_next_voh - p_vh) / tan(angle_voh);
				p_temp += (p_to_voh - p_vh) / tan(angle_next_voh);
			}
		}
		p_temp /= 4 * vertexLAR[vh->index()];
		if (dot(p_temp, vh->normal()) > 0)
			f1 << -p_temp.norm() << std::endl;
		else
			f1 << p_temp.norm() << std::endl;
		f2 << p_temp.norm() << std::endl;
	}
	f1.close();
	f2.close();
	std::cout << "Calculate Mean Curvature Done" << std::endl;
	std::cout << "Calculate Absolute Mean Curvature Done" << std::endl;
}

void cal_gaussian_curvature(PolyMesh* const mesh, const std::vector<double> &vertexLAR)
{
	std::ofstream	f1("./GaussianCurvature.txt");
	if (!f1)
		return;
	for (MVert* vh : mesh->vertices())
	{
		double angle_temp = 2 * PI;
		MVector3  p_vh = vh->position();
		for (auto voh_it = mesh->voh_iter(vh); voh_it.isValid(); ++voh_it)
		{
			if (!(*voh_it)->isBoundary())
			{
				MHalfedge* next_voh = (*voh_it)->next();
				MVert* to_voh = (*voh_it)->toVertex(), *to_next_voh = next_voh->toVertex();
				MVector3 p_to_voh = to_voh->position(), p_to_next_voh = to_next_voh->position();
				double angle = vectorAngle(p_to_voh - p_vh, p_to_next_voh - p_vh);
				angle_temp -= angle;
			}
		}
		angle_temp /= vertexLAR[vh->index()];
		f1 << angle_temp << std::endl;
	}
	f1.close();
	std::cout << "Calculate Gaussian Curvature Done" << std::endl;
}

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

void main(int argc, char** argv)
{
	if (argc != 2)
	{
		std::cout << "========== Hw2 Usage  ==========\n";
		std::cout << std::endl;
		std::cout << "Input:	ACAM_mesh_HW2.exe	mesh.obj\n";
		std::cout << std::endl;
		std::cout << "=================================================\n";
		return;
	}

	//¶ÁÈëÍø¸ñ
	std::string mesh_path = argv[1];
	PolyMesh* mesh = new PolyMesh();
	loadMesh(mesh_path, mesh);
	//create a basic mesh
//	testBasicTriangle();

	//mesh load an write , now only support obj/off
	//PolyMesh* mesh = new PolyMesh();
	//loadMesh("PumpkinMesh.obj", mesh);

	std::cout << "The curvature has area weight" << std::endl;
	std::vector<double> vertexLAR(mesh->numVertices(), 0.0);
	cal_local_ave_region(mesh, vertexLAR);
	cal_mean_curvature(mesh, vertexLAR);
	cal_gaussian_curvature(mesh, vertexLAR);
}