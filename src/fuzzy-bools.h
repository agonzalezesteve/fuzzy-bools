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
		glm::vec3 point;
		glm::vec3 normal;
		size_t buildingElement;
		size_t space;
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
				auto intersectionAndDifferenceGeoms = SplitBoundariesInIntersectionAndDifference(geom, buildingElements[buildingElementId].geometry);
				for (auto firstLevelBoundaryGeom : SplitGeometryByContiguousAndCoplanarFaces(intersectionAndDifferenceGeoms.first))
				{
					Face tri = firstLevelBoundaryGeom.GetFace(0);
					auto a = firstLevelBoundaryGeom.GetPoint(tri.i0);
					auto b = firstLevelBoundaryGeom.GetPoint(tri.i1);
					auto c = firstLevelBoundaryGeom.GetPoint(tri.i2);
					glm::dvec3 norm;
					computeSafeNormal(a, b, c, norm, EPS_SMALL);

					FirstLevelBoundary firstLevelBoundary;
					firstLevelBoundary.id = firstLevelBoundaries.size();
					firstLevelBoundary.geometry = firstLevelBoundaryGeom;
					firstLevelBoundary.point = a;
					firstLevelBoundary.normal = norm;
					firstLevelBoundary.buildingElement = buildingElementId;
					firstLevelBoundary.space = spaceId;

					firstLevelBoundaries.push_back(firstLevelBoundary);
					buildingElements[buildingElementId].firstLevelBoundaries.push_back(firstLevelBoundary.id);
				}

				geom = intersectionAndDifferenceGeoms.second;
				if (geom.IsEmpty())
					break;
			}
		}

		return firstLevelBoundaries;
	}

	std::vector<FirstLevelBoundary> GetSecondLevelBoundaries(std::vector<BuildingElement> &buildingElements, const std::vector<FirstLevelBoundary> &firstLevelBoundaries)
	{
		std::vector<FirstLevelBoundary> secondLevelBoundaries;

		for (size_t buildingElementId = 0; buildingElementId < buildingElements.size(); buildingElementId++)
		{
			BuildingElement buildingElement = buildingElements[buildingElementId];

			double maxArea = -1.0;
			for (int i = 0; i < buildingElement.firstLevelBoundaries.size(); i++)
			{
				FirstLevelBoundary firstLevelBoundary = firstLevelBoundaries[buildingElement.firstLevelBoundaries[i]];
				if (firstLevelBoundary.geometry.IsEmpty())
					break;

				for (int j = i + 1; j < buildingElement.firstLevelBoundaries.size(); j++)
				{
					FirstLevelBoundary otherFirstLevelBoundary = firstLevelBoundaries[buildingElement.firstLevelBoundaries[j]];
					if (glm::dot(firstLevelBoundary.normal, otherFirstLevelBoundary.normal) + 1 > EPS_SMALL)
						continue;

					double distance = glm::dot(firstLevelBoundary.normal, firstLevelBoundary.point) - glm::dot(firstLevelBoundary.normal, otherFirstLevelBoundary.point);
					if (distance < EPS_SMALL)
						continue;

					auto intersectionAndDifferenceGeoms = SplitBoundariesInIntersectionAndDifference(firstLevelBoundary.geometry, otherFirstLevelBoundary.geometry.Translate((float)distance * firstLevelBoundary.normal));

					for (auto secondLevelBoundaryGeom : SplitGeometryByContiguousAndCoplanarFaces(intersectionAndDifferenceGeoms.first))
					{
						FirstLevelBoundary secondLevelBoundary;
						secondLevelBoundary.id = secondLevelBoundaries.size();
						secondLevelBoundary.geometry = secondLevelBoundaryGeom;
						secondLevelBoundary.point = firstLevelBoundary.point;
						secondLevelBoundary.normal = firstLevelBoundary.normal;
						secondLevelBoundary.buildingElement = buildingElementId;
						secondLevelBoundary.space = firstLevelBoundary.space;
						secondLevelBoundaries.push_back(secondLevelBoundary);

						FirstLevelBoundary otherSecondLevelBoundary;
						otherSecondLevelBoundary.id = secondLevelBoundaries.size();
						otherSecondLevelBoundary.geometry = secondLevelBoundary.geometry.Translate((float)distance * otherFirstLevelBoundary.normal);
						otherSecondLevelBoundary.geometry.Flip();
						otherSecondLevelBoundary.point = otherFirstLevelBoundary.point;
						otherSecondLevelBoundary.normal = otherFirstLevelBoundary.normal;
						otherSecondLevelBoundary.buildingElement = buildingElementId;
						otherSecondLevelBoundary.space = otherFirstLevelBoundary.space;
						secondLevelBoundaries.push_back(otherSecondLevelBoundary);

						auto area = secondLevelBoundary.geometry.Area();
						if (area > maxArea)
						{
							buildingElement.thickness = distance;
							maxArea = area;
						}
					}

					firstLevelBoundary.geometry = intersectionAndDifferenceGeoms.second;
					intersectionAndDifferenceGeoms = SplitBoundariesInIntersectionAndDifference(otherFirstLevelBoundary.geometry, intersectionAndDifferenceGeoms.first.Translate((float)distance * otherFirstLevelBoundary.normal));
					otherFirstLevelBoundary.geometry = intersectionAndDifferenceGeoms.second;

					if (firstLevelBoundary.geometry.IsEmpty())
						break;
				}
			}
		}

		return secondLevelBoundaries;
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
		auto secondLevelBoundaries = GetSecondLevelBoundaries(buildingElements, firstLevelBoundaries);

		for (int i = 0; i < spacesAndBuildings.size(); i++)
		{
			auto spaceOrBuilding = spacesAndBuildings[i];

			size_t offset = 0;
			std::ofstream ofs("Space_" + std::to_string(i) + ".obj", std::ofstream::out);
			ofs << fuzzybools::ToObj(spaceOrBuilding.geometry, offset);
			ofs.close();
		}

		for (int i = 0; i < firstLevelBoundaries.size(); i++)
		{
			auto firstLevelBoundary = firstLevelBoundaries[i];

			std::cout << firstLevelBoundary.id << ", " << firstLevelBoundary.buildingElement << ", " << firstLevelBoundary.space << std::endl;
		}

		for (int i = 0; i < secondLevelBoundaries.size(); i += 2)
		{
			auto secondLevelBoundary = secondLevelBoundaries[i];
			auto otherSecondLevelBoundary = secondLevelBoundaries[i + 1];

			double distance = glm::dot(secondLevelBoundary.normal, secondLevelBoundary.point) - glm::dot(secondLevelBoundary.normal, otherSecondLevelBoundary.point);

			std::cout << secondLevelBoundary.id << ", " << secondLevelBoundary.buildingElement << ", " << secondLevelBoundary.space << ", " << distance << std::endl;
			std::cout << otherSecondLevelBoundary.id << ", " << otherSecondLevelBoundary.buildingElement << ", " << otherSecondLevelBoundary.space << ", " << distance << std::endl;
		}
	}
}