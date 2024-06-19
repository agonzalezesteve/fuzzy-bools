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
		bool isVoid;
		std::vector<size_t> voids;
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

	enum class BoundaryCondition
	{
		INTERNAL,
		EXTERNAL,
		NOTDEFINED
	};

	struct SecondLevelBoundary : FirstLevelBoundary
	{
		BoundaryCondition boundaryCondition;
		int parentBoundary;

		std::string BoundaryCondition()
		{
			switch (boundaryCondition)
			{
			case BoundaryCondition::INTERNAL:
				return "INTERNAL";
			case BoundaryCondition::EXTERNAL:
				return "EXTERNAL";
			case BoundaryCondition::NOTDEFINED:
				return "NOTDEFINED";
			default:
				return "UNKNOWN";
			}
		}
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
					for (size_t i = 0; i < neigthbourTriangle.second.size(); i++)
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

	inline std::pair<Geometry, Geometry> SplitFirstBoundaryInIntersectionAndDifference(const Geometry &A, const Geometry &B)
	{
		fuzzybools::SharedPosition sp;
		sp.Construct(A, B);

		auto bvh1 = fuzzybools::MakeBVH(A);
		auto bvh2 = fuzzybools::MakeBVH(B);

		auto geom = Normalize(sp);

		auto firstBoundarySplitted = fuzzybools::SplitFirstBoundary(geom, bvh1, bvh2);
		return {firstBoundarySplitted.first, firstBoundarySplitted.second.first};
	}

	std::vector<Geometry> SplitGeometryByContiguousAndCoplanarFaces(const Geometry &geom)
	{
		std::vector<Geometry> newGeoms;
		if (geom.IsEmpty())
			return newGeoms;

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
					for (size_t i = 0; i < neigthbourTriangle.second.size(); i++)
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
				auto &buildingElement = buildingElements[buildingElementId];
				if (buildingElement.isVoid)
					continue;

				auto intersectionAndDifferenceGeoms = SplitFirstBoundaryInIntersectionAndDifference(geom, buildingElement.geometry);
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
					buildingElement.firstLevelBoundaries.push_back(firstLevelBoundary.id);
				}

				geom = intersectionAndDifferenceGeoms.second;
				if (geom.IsEmpty())
					break;
			}
		}

		return firstLevelBoundaries;
	}

	void CorrectInternalSecondLevelBoundaries(std::vector<SecondLevelBoundary> &secondLevelBoundaries, const size_t buildingElementId, const std::vector<BuildingElement> &buildingElements)
	{
		int secondLevelBoundaryId = secondLevelBoundaries.size() - 1;

		while (secondLevelBoundaryId >= 0)
		{
			auto &secondLevelBoundary = secondLevelBoundaries[secondLevelBoundaryId];
			if (secondLevelBoundary.buildingElement != buildingElementId)
				break;

			switch (secondLevelBoundary.boundaryCondition)
			{
			case BoundaryCondition::INTERNAL:
			{
				auto &otherSecondLevelBoundary = secondLevelBoundaries[secondLevelBoundaryId - 1];
				double internalSecondLevelBoundaryDistance = glm::dot(otherSecondLevelBoundary.normal, otherSecondLevelBoundary.point) - glm::dot(otherSecondLevelBoundary.normal, secondLevelBoundary.point);
				if (std::fabs(internalSecondLevelBoundaryDistance - buildingElements[buildingElementId].thickness) > EPS_SMALL)
				{
					secondLevelBoundary.boundaryCondition = BoundaryCondition::NOTDEFINED;
					otherSecondLevelBoundary.boundaryCondition = BoundaryCondition::NOTDEFINED;
				}

				secondLevelBoundaryId -= 2;
				break;
			}
			case BoundaryCondition::EXTERNAL:
			{
				secondLevelBoundaryId -= 2;
				break;
			}
			default:
				secondLevelBoundaryId -= 1;
				break;
			}
		}
	}

	inline Geometry IntersectFirstBoundaryWithSecondGeometry(const Geometry &A, const Geometry &B)
	{
		fuzzybools::SharedPosition sp;
		sp.Construct(A, B);

		auto bvh1 = fuzzybools::MakeBVH(A);
		auto bvh2 = fuzzybools::MakeBVH(B);

		auto geom = Normalize(sp);

		return fuzzybools::SplitFirstBoundary(geom, bvh1, bvh2).second.second;
	}

	void AddVoids(const std::vector<BuildingElement> &buildingElements, const size_t buildingElementId, std::vector<SecondLevelBoundary> &secondLevelBoundaries)
	{
		auto buildingElement = buildingElements[buildingElementId];
		for (size_t i = 0; i < buildingElement.voids.size(); ++i)
		{
			auto voidElement = buildingElements[buildingElement.voids[i]];

			int secondLevelBoundaryId = secondLevelBoundaries.size() - 1;
			bool exitLoop = false;
			while (secondLevelBoundaryId >= 0 && !exitLoop)
			{
				auto parentBoundary = secondLevelBoundaries[secondLevelBoundaryId];
				switch (parentBoundary.boundaryCondition)
				{
				case BoundaryCondition::INTERNAL:
				case BoundaryCondition::EXTERNAL:
				{
					auto secondLevelBoundaryGeom = IntersectFirstBoundaryWithSecondGeometry(parentBoundary.geometry, voidElement.geometry);
					if (!secondLevelBoundaryGeom.IsEmpty())
					{
						SecondLevelBoundary secondLevelBoundary;
						secondLevelBoundary.id = secondLevelBoundaries.size();
						secondLevelBoundary.geometry = secondLevelBoundaryGeom;
						secondLevelBoundary.point = secondLevelBoundary.geometry.GetPoint(secondLevelBoundary.geometry.GetFace(0).i0);
						secondLevelBoundary.normal = parentBoundary.normal;
						secondLevelBoundary.buildingElement = voidElement.id;
						secondLevelBoundary.space = parentBoundary.space;
						secondLevelBoundary.boundaryCondition = parentBoundary.boundaryCondition;
						secondLevelBoundary.parentBoundary = parentBoundary.id;
						secondLevelBoundaries.push_back(secondLevelBoundary);

						auto otherParentBoundary = secondLevelBoundaries[secondLevelBoundaryId - 1];
						double parentBoundaryDistance = glm::dot(parentBoundary.normal, parentBoundary.point) - glm::dot(parentBoundary.normal, otherParentBoundary.point);

						SecondLevelBoundary otherSecondLevelBoundary;
						otherSecondLevelBoundary.id = secondLevelBoundaries.size();
						otherSecondLevelBoundary.geometry = secondLevelBoundary.geometry.Translate((float)parentBoundaryDistance * otherParentBoundary.normal);
						otherSecondLevelBoundary.geometry.Flip();
						otherSecondLevelBoundary.point = otherSecondLevelBoundary.geometry.GetPoint(otherSecondLevelBoundary.geometry.GetFace(0).i0);
						otherSecondLevelBoundary.normal = otherParentBoundary.normal;
						otherSecondLevelBoundary.buildingElement = voidElement.id;
						otherSecondLevelBoundary.space = otherParentBoundary.space;
						otherSecondLevelBoundary.boundaryCondition = otherParentBoundary.boundaryCondition;
						otherSecondLevelBoundary.parentBoundary = otherParentBoundary.id;
						secondLevelBoundaries.push_back(otherSecondLevelBoundary);

						exitLoop = true;
					}

					secondLevelBoundaryId -= 2;
					break;
				}
				case BoundaryCondition::NOTDEFINED:
				default:
					secondLevelBoundaryId -= 1;
					break;
				}
			}
		}
	}

	std::vector<SecondLevelBoundary> GetSecondLevelBoundaries(std::vector<BuildingElement> &buildingElements, const std::vector<SpaceOrBuilding> &spaceAndBuildings, std::vector<FirstLevelBoundary> &firstLevelBoundaries)
	{
		std::vector<SecondLevelBoundary> secondLevelBoundaries;

		for (size_t buildingElementId = 0; buildingElementId < buildingElements.size(); buildingElementId++)
		{
			auto &buildingElement = buildingElements[buildingElementId];
			if (buildingElement.isVoid)
				continue;

			double maxArea = -1.0;
			for (size_t i = 0; i < buildingElement.firstLevelBoundaries.size(); i++)
			{
				auto &firstLevelBoundary = firstLevelBoundaries[buildingElement.firstLevelBoundaries[i]];
				if (firstLevelBoundary.geometry.IsEmpty())
					continue;

				for (size_t j = i + 1; j < buildingElement.firstLevelBoundaries.size(); j++)
				{
					auto &otherFirstLevelBoundary = firstLevelBoundaries[buildingElement.firstLevelBoundaries[j]];
					if (glm::dot(firstLevelBoundary.normal, otherFirstLevelBoundary.normal) + 1 > EPS_SMALL)
						continue;

					double firstLevelBoundaryDistance = glm::dot(firstLevelBoundary.normal, firstLevelBoundary.point) - glm::dot(firstLevelBoundary.normal, otherFirstLevelBoundary.point);
					if (firstLevelBoundaryDistance < EPS_SMALL)
						continue;

					auto intersectionAndDifferenceGeoms = SplitFirstBoundaryInIntersectionAndDifference(firstLevelBoundary.geometry, otherFirstLevelBoundary.geometry.Translate((float)firstLevelBoundaryDistance * firstLevelBoundary.normal));

					for (auto secondLevelBoundaryGeom : SplitGeometryByContiguousAndCoplanarFaces(intersectionAndDifferenceGeoms.first))
					{
						BoundaryCondition boundaryCondition = ((!spaceAndBuildings[firstLevelBoundary.space].isSpace || !spaceAndBuildings[otherFirstLevelBoundary.space].isSpace) ? BoundaryCondition::EXTERNAL : BoundaryCondition::INTERNAL);

						SecondLevelBoundary secondLevelBoundary;
						secondLevelBoundary.id = secondLevelBoundaries.size();
						secondLevelBoundary.geometry = secondLevelBoundaryGeom;
						secondLevelBoundary.point = secondLevelBoundary.geometry.GetPoint(secondLevelBoundary.geometry.GetFace(0).i0);
						secondLevelBoundary.normal = firstLevelBoundary.normal;
						secondLevelBoundary.buildingElement = buildingElementId;
						secondLevelBoundary.space = firstLevelBoundary.space;
						secondLevelBoundary.boundaryCondition = boundaryCondition;
						secondLevelBoundary.parentBoundary = -1;
						secondLevelBoundaries.push_back(secondLevelBoundary);

						SecondLevelBoundary otherSecondLevelBoundary;
						otherSecondLevelBoundary.id = secondLevelBoundaries.size();
						otherSecondLevelBoundary.geometry = secondLevelBoundary.geometry.Translate((float)firstLevelBoundaryDistance * otherFirstLevelBoundary.normal);
						otherSecondLevelBoundary.geometry.Flip();
						otherSecondLevelBoundary.point = otherSecondLevelBoundary.geometry.GetPoint(otherSecondLevelBoundary.geometry.GetFace(0).i0);
						otherSecondLevelBoundary.normal = otherFirstLevelBoundary.normal;
						otherSecondLevelBoundary.buildingElement = buildingElementId;
						otherSecondLevelBoundary.space = otherFirstLevelBoundary.space;
						otherSecondLevelBoundary.boundaryCondition = boundaryCondition;
						otherSecondLevelBoundary.parentBoundary = -1;
						secondLevelBoundaries.push_back(otherSecondLevelBoundary);

						auto area = secondLevelBoundary.geometry.Area();
						if (area > maxArea)
						{
							buildingElement.thickness = firstLevelBoundaryDistance;
							maxArea = area;
						}
					}

					firstLevelBoundary.geometry = intersectionAndDifferenceGeoms.second;
					intersectionAndDifferenceGeoms = SplitFirstBoundaryInIntersectionAndDifference(otherFirstLevelBoundary.geometry, intersectionAndDifferenceGeoms.first.Translate((float)firstLevelBoundaryDistance * otherFirstLevelBoundary.normal));
					otherFirstLevelBoundary.geometry = intersectionAndDifferenceGeoms.second;

					if (firstLevelBoundary.geometry.IsEmpty())
						break;
				}

				for (auto secondLevelBoundaryGeom : SplitGeometryByContiguousAndCoplanarFaces(firstLevelBoundary.geometry))
				{
					SecondLevelBoundary secondLevelBoundary;
					secondLevelBoundary.id = secondLevelBoundaries.size();
					secondLevelBoundary.geometry = secondLevelBoundaryGeom;
					secondLevelBoundary.point = secondLevelBoundary.geometry.GetPoint(secondLevelBoundary.geometry.GetFace(0).i0);
					secondLevelBoundary.normal = firstLevelBoundary.normal;
					secondLevelBoundary.buildingElement = buildingElementId;
					secondLevelBoundary.space = firstLevelBoundary.space;
					secondLevelBoundary.boundaryCondition = BoundaryCondition::NOTDEFINED;
					secondLevelBoundary.parentBoundary = -1;
					secondLevelBoundaries.push_back(secondLevelBoundary);
				}
			}

			CorrectInternalSecondLevelBoundaries(secondLevelBoundaries, buildingElementId, buildingElements);
			AddVoids(buildingElements, buildingElementId, secondLevelBoundaries);
		}

		return secondLevelBoundaries;
	}

	std::vector<std::vector<glm::dvec3>> SplitGeometryInClosedWires(const Geometry &A)
	{
		std::vector<std::vector<glm::dvec3>> wires;

		for (auto contiguousAndCoplanarFaces : SplitGeometryByContiguousAndCoplanarFaces(A))
		{
			std::vector<glm::dvec3> wire;

			fuzzybools::SharedPosition sp;
			sp.AddSingleGeometry(contiguousAndCoplanarFaces);

			auto contoursA = sp.A.GetContourSegments();
			for (auto &[planeId, contours] : contoursA)
			{
				for (auto &segment : contours)
				{
					wire.push_back(sp.points[segment.first].location3D);
				}
			}

			wires.push_back(wire);
		}

		return wires;
	}

	void GetSpacesGeomsByBuildingElements(std::vector<BuildingElement> &buildingElements)
	{
		Geometry unionGeom;
		for (size_t buildingElementId = 0; buildingElementId < buildingElements.size(); buildingElementId++)
		{
			auto buildingElement = buildingElements[buildingElementId];
			if (buildingElement.isVoid)
				continue;

			unionGeom = Union(buildingElement.geometry, unionGeom);
		}

		auto spacesAndBuildings = GetSpacesAndBuildings(unionGeom);
		auto firstLevelBoundaries = GetFirstLevelBoundaries(buildingElements, spacesAndBuildings);
		auto secondLevelBoundaries = GetSecondLevelBoundaries(buildingElements, spacesAndBuildings, firstLevelBoundaries);

		for (size_t spaceOrBuildingId = 0; spaceOrBuildingId < spacesAndBuildings.size(); spaceOrBuildingId++)
		{
			auto spaceOrBuilding = spacesAndBuildings[spaceOrBuildingId];

			size_t offset = 0;
			std::ofstream ofs("Space_" + std::to_string(spaceOrBuildingId) + ".obj", std::ofstream::out);
			ofs << fuzzybools::ToObj(spaceOrBuilding.geometry, offset);
			ofs.close();
		}

		for (size_t firstLevelBoundaryId = 0; firstLevelBoundaryId < firstLevelBoundaries.size(); firstLevelBoundaryId++)
		{
			auto firstLevelBoundary = firstLevelBoundaries[firstLevelBoundaryId];

			std::cout << firstLevelBoundary.id << ", " << firstLevelBoundary.buildingElement << ", " << firstLevelBoundary.space << std::endl;
		}

		int secondLevelBoundaryId = 0;
		while (secondLevelBoundaryId < secondLevelBoundaries.size())
		{
			auto secondLevelBoundary = secondLevelBoundaries[secondLevelBoundaryId];

			switch (secondLevelBoundary.boundaryCondition)
			{
			case BoundaryCondition::INTERNAL:
			{
				auto otherSecondLevelBoundary = secondLevelBoundaries[secondLevelBoundaryId + 1];

				std::cout << secondLevelBoundary.id << ", " << secondLevelBoundary.buildingElement << ", " << secondLevelBoundary.space << ", " << secondLevelBoundary.BoundaryCondition() << std::endl;
				std::cout << otherSecondLevelBoundary.id << ", " << otherSecondLevelBoundary.buildingElement << ", " << otherSecondLevelBoundary.space << ", " << otherSecondLevelBoundary.BoundaryCondition() << std::endl;

				secondLevelBoundaryId += 2;
				break;
			}
			case BoundaryCondition::EXTERNAL:
			{
				if (spacesAndBuildings[secondLevelBoundary.space].isSpace)
				{
					std::cout << secondLevelBoundary.id << ", " << secondLevelBoundary.buildingElement << ", " << secondLevelBoundary.space << ", " << secondLevelBoundary.BoundaryCondition() << std::endl;
				}

				auto otherSecondLevelBoundary = secondLevelBoundaries[secondLevelBoundaryId + 1];
				if (spacesAndBuildings[otherSecondLevelBoundary.space].isSpace)
				{
					std::cout << otherSecondLevelBoundary.id << ", " << otherSecondLevelBoundary.buildingElement << ", " << otherSecondLevelBoundary.space << ", " << otherSecondLevelBoundary.BoundaryCondition() << std::endl;
				}

				secondLevelBoundaryId += 2;
				break;
			}
			case BoundaryCondition::NOTDEFINED:
			{
				if (spacesAndBuildings[secondLevelBoundary.space].isSpace)
				{
					std::cout << secondLevelBoundary.id << ", " << secondLevelBoundary.buildingElement << ", " << secondLevelBoundary.space << ", " << secondLevelBoundary.BoundaryCondition() << std::endl;
				}

				secondLevelBoundaryId += 1;
				break;
			}
			default:
				secondLevelBoundaryId += 1;
				break;
			}
		}
	}
}