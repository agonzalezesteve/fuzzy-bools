#include <iostream>
#include "fuzzy-bools.h"

std::string GetVersion()
{
    return "1";
}

int main(int argc, char *argv[])
{
    // Your code here

    std::cout << "Fuzzy Bools Version: " << GetVersion() << std::endl;

    glm::dvec3 points[10][8] = {
        {
            glm::dvec3(0.0, 0.0, 0.0),
            glm::dvec3(0.0, 0.0, 0.1),
            glm::dvec3(0.0, 20.0, 0.1),
            glm::dvec3(0.0, 20.0, 0.0),
            glm::dvec3(30.0, 20.0, 0.1),
            glm::dvec3(30.0, 20.0, 0.0),
            glm::dvec3(30.0, 0.0, 0.1),
            glm::dvec3(30.0, 0.0, 0.0),
        },
        {
            glm::dvec3(0.0, 0.0, 0.1),
            glm::dvec3(0.0, 0.0, 2.9),
            glm::dvec3(0.0, 0.1, 2.9),
            glm::dvec3(0.0, 0.1, 0.1),
            glm::dvec3(30.0, 0.1, 2.9),
            glm::dvec3(30.0, 0.1, 0.1),
            glm::dvec3(30.0, 0.0, 2.9),
            glm::dvec3(30.0, 0.0, 0.1),
        },
        {
            glm::dvec3(0.0, 19.9, 0.1),
            glm::dvec3(0.0, 19.9, 2.9),
            glm::dvec3(0.0, 20.0, 2.9),
            glm::dvec3(0.0, 20.0, 0.1),
            glm::dvec3(30.0, 20.0, 2.9),
            glm::dvec3(30.0, 20.0, 0.1),
            glm::dvec3(30.0, 19.9, 2.9),
            glm::dvec3(30.0, 19.9, 0.1),
        },
        {
            glm::dvec3(0.0, 0.1, 0.1),
            glm::dvec3(0.0, 0.1, 2.9),
            glm::dvec3(0.0, 19.9, 2.9),
            glm::dvec3(0.0, 19.9, 0.1),
            glm::dvec3(0.1, 19.9, 2.9),
            glm::dvec3(0.1, 19.9, 0.1),
            glm::dvec3(0.1, 0.1, 2.9),
            glm::dvec3(0.1, 0.1, 0.1),
        },
        {
            glm::dvec3(29.9, 0.1, 0.1),
            glm::dvec3(29.9, 0.1, 2.9),
            glm::dvec3(29.9, 19.9, 2.9),
            glm::dvec3(29.9, 19.9, 0.1),
            glm::dvec3(30.0, 19.9, 2.9),
            glm::dvec3(30.0, 19.9, 0.1),
            glm::dvec3(30.0, 0.1, 2.9),
            glm::dvec3(30.0, 0.1, 0.1),
        },
        {
            glm::dvec3(19.9, 0.1, 0.1),
            glm::dvec3(19.9, 0.1, 2.9),
            glm::dvec3(19.9, 19.9, 2.9),
            glm::dvec3(19.9, 19.9, 0.1),
            glm::dvec3(20.0, 19.9, 2.9),
            glm::dvec3(20.0, 19.9, 0.1),
            glm::dvec3(20.0, 0.1, 2.9),
            glm::dvec3(20.0, 0.1, 0.1),
        },
        {
            glm::dvec3(20.0, 9.9, 0.1),
            glm::dvec3(20.0, 9.9, 2.9),
            glm::dvec3(20.0, 10.0, 2.9),
            glm::dvec3(20.0, 10.0, 0.1),
            glm::dvec3(29.9, 10.0, 2.9),
            glm::dvec3(29.9, 10.0, 0.1),
            glm::dvec3(29.9, 9.9, 2.9),
            glm::dvec3(29.9, 9.9, 0.1),
        },
        {
            glm::dvec3(0.0, 0.0, 2.9),
            glm::dvec3(0.0, 0.0, 3.0),
            glm::dvec3(0.0, 20.0, 3.0),
            glm::dvec3(0.0, 20.0, 2.9),
            glm::dvec3(30.0, 20.0, 3.0),
            glm::dvec3(30.0, 20.0, 2.9),
            glm::dvec3(30.0, 0.0, 3.0),
            glm::dvec3(30.0, 0.0, 2.9),
        },
        {
            glm::dvec3(-1.5, 9.5, 0.5),
            glm::dvec3(-1.5, 9.5, 1.0),
            glm::dvec3(-1.5, 10.5, 1.0),
            glm::dvec3(-1.5, 10.5, 0.5),
            glm::dvec3(1.5, 10.5, 1.0),
            glm::dvec3(1.5, 10.5, 0.5),
            glm::dvec3(1.5, 9.5, 1.0),
            glm::dvec3(1.5, 9.5, 0.5),
        },
        {
            glm::dvec3(19.5, 14.5, 0.5),
            glm::dvec3(19.5, 14.5, 1.0),
            glm::dvec3(19.5, 15.5, 1.0),
            glm::dvec3(19.5, 15.5, 0.5),
            glm::dvec3(20.5, 15.5, 1.0),
            glm::dvec3(20.5, 15.5, 0.5),
            glm::dvec3(20.5, 14.5, 1.0),
            glm::dvec3(20.5, 14.5, 0.5),
        }
    };

    std::vector<fuzzybools::Geometry> geoms;
    for (auto &boxPoints : points)
    {
        fuzzybools::Geometry geom;

        geom.AddFace(boxPoints[3], boxPoints[0], boxPoints[1]);
        geom.AddFace(boxPoints[3], boxPoints[1], boxPoints[2]);
        geom.AddFace(boxPoints[5], boxPoints[3], boxPoints[2]);
        geom.AddFace(boxPoints[5], boxPoints[2], boxPoints[4]);
        geom.AddFace(boxPoints[7], boxPoints[5], boxPoints[4]);
        geom.AddFace(boxPoints[7], boxPoints[4], boxPoints[6]);
        geom.AddFace(boxPoints[0], boxPoints[7], boxPoints[6]);
        geom.AddFace(boxPoints[0], boxPoints[6], boxPoints[1]);
        geom.AddFace(boxPoints[3], boxPoints[5], boxPoints[7]);
        geom.AddFace(boxPoints[3], boxPoints[7], boxPoints[0]);
        geom.AddFace(boxPoints[6], boxPoints[4], boxPoints[2]);
        geom.AddFace(boxPoints[1], boxPoints[6], boxPoints[2]);

        geoms.push_back(geom);
    }

    std::vector<fuzzybools::BuildingElement> buildingElements(geoms.size());
    for (int buildingElementId = 0; buildingElementId < geoms.size(); ++buildingElementId)
    {
        buildingElements[buildingElementId].id = buildingElementId;
        buildingElements[buildingElementId].geometry = geoms[buildingElementId];
    }
    buildingElements[8].isVoid = true;
    buildingElements[3].voids.push_back(8);
    buildingElements[9].isVoid = true;
    buildingElements[5].voids.push_back(9);

    fuzzybools::GetSpacesGeomsByBuildingElements(buildingElements);

    // auto resultUnion = fuzzybools::Union(A, B);

    // size_t offsetUnion = 0;

    // std::ofstream ofsUnion("union.obj", std::ofstream::out);
    // ofsUnion << fuzzybools::ToObj(resultUnion, offsetUnion);
    // ofsUnion.close();

    // auto resultIntersection = fuzzybools::Intersection(A, B);

    // size_t offsetIntersection = 0;

    // std::ofstream ofsIntersection("intersection.obj", std::ofstream::out);
    // ofsIntersection << fuzzybools::ToObj(resultIntersection, offsetIntersection);
    // ofsIntersection.close();

    // auto resultDifference = fuzzybools::Difference(A, B);

    // size_t offsetDifference = 0;

    // std::ofstream ofsDifference("difference.obj", std::ofstream::out);
    // ofsDifference << fuzzybools::ToObj(resultDifference, offsetDifference);
    // ofsDifference.close();

    return 0;
}