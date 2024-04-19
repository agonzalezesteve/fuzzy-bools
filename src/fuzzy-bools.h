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

		return fuzzybools::clipBooleanResult(op, geom, bvh1, bvh2);
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

	std::vector<Geometry> SplitGeomByContiguousFaces(const Geometry &geom)
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
				glm::dvec3 a = sp.points[triangle.a].location3D;
				glm::dvec3 b = sp.points[triangle.b].location3D;
				glm::dvec3 c = sp.points[triangle.c].location3D;
				newGeom.AddFace(a, b, c);

				for (const auto &item : sp.A.GetNeighbourTriangles(triangle))
				{
					for (int i = 0; i < item.second.size(); i++)
					{
						size_t neighbourId = item.second[i];
						if (visited[neighbourId])
							continue;

						q.push(neighbourId);
					}
				}
			}

			if (newGeom.Volume() < 0)
			{
				newGeom.Flip();
			}

			newGeoms.push_back(newGeom);
		}

		return newGeoms;
	}

	std::vector<Geometry> JoinGeomsAndSplitByContiguousFaces(const std::vector<Geometry> &geoms)
	{
		Geometry unionGeom;

		for (auto &geom : geoms)
		{
			unionGeom = Union(geom, unionGeom);
		}

		int i = 0;
		std::vector<Geometry> joinGeoms = SplitGeomByContiguousFaces(unionGeom);
		for (auto &geom : joinGeoms)
		{
			size_t offset = 0;
			std::ofstream ofs(std::to_string(i) + ".obj", std::ofstream::out);
			ofs << fuzzybools::ToObj(geom, offset);
			ofs.close();

			++i;
		}

		return joinGeoms;
	}
}