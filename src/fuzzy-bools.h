#pragma once

#include <queue>

#include "geometry.h"
#include "shared-position.h"
#include "clip-mesh.h"

namespace fuzzybools
{
	inline Geometry BooleanResult(const BooleanOperator op, const Geometry &A, const Geometry &B)
	{
		fuzzybools::SharedPosition sp;
		sp.Construct(A, B);

		auto bvh1 = fuzzybools::MakeBVH(A);
		auto bvh2 = fuzzybools::MakeBVH(B);

		auto geom = Normalize(sp);

		return fuzzybools::ClipBooleanResult(op, geom, bvh1, bvh2);
	}

	inline Geometry Union(const Geometry &A, const Geometry &B)
	{
		return fuzzybools::BooleanResult(BooleanOperator::UNION, A, B);
	}

	inline Geometry Intersection(const Geometry &A, const Geometry &B)
	{
		return fuzzybools::BooleanResult(BooleanOperator::INTERSECTION, A, B);
	}

	inline Geometry Difference(const Geometry &A, const Geometry &B)
	{
		return fuzzybools::BooleanResult(BooleanOperator::DIFFERENCE, A, B);
	}

	inline std::pair<Geometry, Geometry> SplitBoundriesInIntersectionAndDifference(const Geometry &A, const Geometry &B)
	{
		fuzzybools::SharedPosition sp;
		sp.Construct(A, B);

		auto bvh1 = fuzzybools::MakeBVH(A);
		auto bvh2 = fuzzybools::MakeBVH(B);

		auto geom = Normalize(sp);

		return fuzzybools::SplitBoundaryInIntersectionAndDifference(geom, bvh1, bvh2);
	}

	std::vector<Geometry> SplitUnionGeometryInSpacesGeometries(const Geometry &unionGeom)
	{
		std::vector<Geometry> spaceGeoms;

		fuzzybools::SharedPosition sp;
		sp.AddSingleGeometry(unionGeom);

		std::vector<Triangle> geomTriangles = sp.A.triangles;
		std::vector<bool> visited(geomTriangles.size(), false);
		for (size_t triangleId = 0; triangleId < geomTriangles.size(); triangleId++)
		{
			if (visited[triangleId])
				continue;

			Geometry spaceGeom;
			std::queue<size_t> q;
			q.push(triangleId);

			while (!q.empty())
			{
				size_t currentId = q.front();
				q.pop();
				if (visited[currentId])
					continue;

				visited[currentId] = true;

				Triangle triangle = geomTriangles[currentId];
				glm::dvec3 a = sp.points[triangle.a].location3D;
				glm::dvec3 b = sp.points[triangle.b].location3D;
				glm::dvec3 c = sp.points[triangle.c].location3D;
				spaceGeom.AddFace(a, b, c);

				for (const auto &neigthbourTriangle : sp.A.GetNeighbourTriangles(triangle))
				{
					for (int i = 0; i < neigthbourTriangle.second.size(); i++)
					{
						size_t neighbourId = neigthbourTriangle.second[i];
						if (visited[neighbourId])
							continue;

						q.push(neighbourId);
					}
				}
			}

			if (spaceGeom.Volume() < 0)
			{
				spaceGeom.Flip();
				spaceGeoms.push_back(spaceGeom);
			}
		}

		return spaceGeoms;
	}

	std::vector<Geometry> SplitGeometryByContiguousAndCoplanarFaces(const Geometry &geom)
	{
		std::vector<Geometry> newGeoms;

		fuzzybools::SharedPosition sp;
		sp.AddSingleGeometry(geom);

		std::vector<Triangle> geomTriangles = sp.A.triangles;
		std::vector<bool> visited(geomTriangles.size(), false);
		for (size_t triangleId = 0; triangleId < geomTriangles.size(); triangleId++)
		{
			if (visited[triangleId])
				continue;

			Geometry newGeom;
			std::queue<size_t> q;
			q.push(triangleId);

			while (!q.empty())
			{
				size_t currentId = q.front();
				q.pop();
				if (visited[currentId])
					continue;

				visited[currentId] = true;

				Triangle triangle = geomTriangles[currentId];
				glm::dvec3 norm = sp.GetNormal(triangle);

				glm::dvec3 a = sp.points[triangle.a].location3D;
				glm::dvec3 b = sp.points[triangle.b].location3D;
				glm::dvec3 c = sp.points[triangle.c].location3D;
				newGeom.AddFace(a, b, c);

				for (auto &neigthbourTriangle : sp.A.GetNeighbourTriangles(triangle))
				{
					for (int i = 0; i < neigthbourTriangle.second.size(); i++)
					{
						size_t neighbourId = neigthbourTriangle.second[i];

						if (std::fabs(glm::dot(norm, sp.GetNormal(geomTriangles[neighbourId])) - 1) > EPS_BIG)
							continue;

						if (visited[neighbourId])
							continue;

						q.push(neighbourId);
					}
				}
			}

			newGeoms.push_back(newGeom);
		}

		return newGeoms;
	}

	std::vector<Geometry> GetSpacesGeomsByBuildingElementsGeoms(const std::vector<Geometry> &buildingElementsGeoms)
	{
		Geometry unionGeom;
		for (auto &buildingElementsGeom : buildingElementsGeoms)
		{
			unionGeom = Union(buildingElementsGeom, unionGeom);
		}

		std::vector<Geometry> spaceGeoms = SplitUnionGeometryInSpacesGeometries(unionGeom);

		int k = 0;

		std::vector<std::pair<size_t, Geometry>> firstLevelBoundaries[buildingElementsGeoms.size()];
		for (size_t j = 0; j < spaceGeoms.size(); j++)
		{
			auto geom = spaceGeoms[j];

			size_t spaceOffset = 0;
			std::ofstream spaceOfs("Space_" + std::to_string(j) + ".obj", std::ofstream::out);
			spaceOfs << fuzzybools::ToObj(geom, spaceOffset);
			spaceOfs.close();

			for (size_t i = 0; i < buildingElementsGeoms.size(); i++)
			{
				auto intersectionAndDifferenceGeom = SplitBoundriesInIntersectionAndDifference(geom, buildingElementsGeoms[i]);
				for (auto firstLevelBoundaryGeom : SplitGeometryByContiguousAndCoplanarFaces(intersectionAndDifferenceGeom.first))
				{
					size_t firstOffset = 0;
					std::ofstream firstOfs("First_" + std::to_string(k) + ".obj", std::ofstream::out);
					firstOfs << fuzzybools::ToObj(firstLevelBoundaryGeom, firstOffset);
					firstOfs.close();

					++k;

					firstLevelBoundaries[i].emplace_back(j, firstLevelBoundaryGeom);
				}

				geom = intersectionAndDifferenceGeom.second;
				if (geom.IsEmpty())
					break;
			}
		}

		std::cout << k << std::endl;

		return spaceGeoms;
	}
}