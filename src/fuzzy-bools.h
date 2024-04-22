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

	struct GeometryWithId
	{
		size_t id;
		Geometry geometry;
	};

	struct BuildingElement : GeometryWithId
	{
		float thickness;
		std::vector<size_t> firstLevelBoundaries;
	};

	struct SpaceOrBuilding : GeometryWithId
	{
		bool isSpace;
	};

	struct FirstLevelBoundary : GeometryWithId
	{
		size_t buildingElement;
		size_t space;
	};

	enum class BoundaryType
	{
		INTERNAL,
		EXTERNAL,
		NOTDEFINED
	};

	struct SecondLevelBoundary : FirstLevelBoundary
	{
		BoundaryType boundaryType;
	};

	std::vector<SpaceOrBuilding> GetSpacesAndBuildings(const Geometry &unionGeom)
	{
		std::vector<SpaceOrBuilding> spacesAndBuildings;

		fuzzybools::SharedPosition sp;
		sp.AddSingleGeometry(unionGeom);

		std::vector<Triangle> geomTriangles = sp.A.triangles;
		std::vector<bool> visited(geomTriangles.size(), false);
		for (size_t triangleId = 0; triangleId < geomTriangles.size(); triangleId++)
		{
			if (visited[triangleId])
				continue;

			Geometry spaceOrBuildingGeom;
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
				spaceOrBuildingGeom.AddFace(a, b, c);

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

			SpaceOrBuilding spaceOrBuilding;
			spaceOrBuilding.id = spacesAndBuildings.size();
			spaceOrBuilding.geometry = spaceOrBuildingGeom;
			spaceOrBuilding.isSpace = (spaceOrBuildingGeom.Volume() < 0);

			spacesAndBuildings.push_back(spaceOrBuilding);
		}

		return spacesAndBuildings;
	}

	inline std::pair<Geometry, Geometry> SplitBoundariesInIntersectionAndDifference(const Geometry &A, const Geometry &B)
	{
		fuzzybools::SharedPosition sp;
		sp.Construct(A, B);

		auto bvh1 = fuzzybools::MakeBVH(A);
		auto bvh2 = fuzzybools::MakeBVH(B);

		auto geom = Normalize(sp);

		return fuzzybools::SplitBoundaryInIntersectionAndDifference(geom, bvh1, bvh2);
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

	std::vector<FirstLevelBoundary> GetFirstLevelBoundaries(std::vector<BuildingElement> &buildingElements, const std::vector<SpaceOrBuilding> &spaceAndBuildings)
	{
		std::vector<FirstLevelBoundary> firstLevelBoundaries;

		for (size_t spaceId = 0; spaceId < spaceAndBuildings.size(); spaceId++)
		{
			auto geom = spaceAndBuildings[spaceId].geometry;

			for (size_t buildingElementId = 0; buildingElementId < buildingElements.size(); buildingElementId++)
			{
				auto intersectionAndDifferenceGeom = SplitBoundariesInIntersectionAndDifference(geom, buildingElements[buildingElementId].geometry);
				for (auto firstLevelBoundaryGeom : SplitGeometryByContiguousAndCoplanarFaces(intersectionAndDifferenceGeom.first))
				{
					size_t firstOffset = 0;
					std::ofstream firstOfs("First_Space" + std::to_string(spaceId) + "_BuildingElement" + std::to_string(buildingElementId) + ".obj", std::ofstream::out);
					firstOfs << fuzzybools::ToObj(firstLevelBoundaryGeom, firstOffset);
					firstOfs.close();

					FirstLevelBoundary firstLevelBoundary;
					firstLevelBoundary.id = firstLevelBoundaries.size();
					firstLevelBoundary.geometry = firstLevelBoundaryGeom;
					firstLevelBoundary.buildingElement = buildingElementId;
					firstLevelBoundary.space = spaceId;

					firstLevelBoundaries.push_back(firstLevelBoundary);
					buildingElements[buildingElementId].firstLevelBoundaries.push_back(firstLevelBoundary.id);
				}

				geom = intersectionAndDifferenceGeom.second;
				if (geom.IsEmpty())
					break;
			}
		}

		return firstLevelBoundaries;
	}

	void GetSpacesGeomsByBuildingElementsGeoms(const std::vector<Geometry> &buildingElementsGeoms)
	{
		std::vector<BuildingElement> buildingElements(buildingElementsGeoms.size());

		Geometry unionGeom;
		for (size_t buildingElementId = 0; buildingElementId < buildingElementsGeoms.size(); buildingElementId++)
		{
			buildingElements[buildingElementId].id = buildingElementId;
			buildingElements[buildingElementId].geometry = buildingElementsGeoms[buildingElementId];

			unionGeom = Union(buildingElements[buildingElementId].geometry, unionGeom);
		}

		auto spacesAndBuildings = GetSpacesAndBuildings(unionGeom);
		auto firstLevelBoundaries = GetFirstLevelBoundaries(buildingElements, spacesAndBuildings);
	}
}