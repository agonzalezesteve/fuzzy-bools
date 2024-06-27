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

    glm::dvec3 points[6][8] = {
        {
            glm::dvec3(0.0, 0.0, 0.0),
            glm::dvec3(0.0, 0.0, 0.1),
            glm::dvec3(0.0, 4.0, 0.1),
            glm::dvec3(0.0, 4.0, 0.0),
            glm::dvec3(5.0, 4.0, 0.1),
            glm::dvec3(5.0, 4.0, 0.0),
            glm::dvec3(5.0, 0.0, 0.1),
            glm::dvec3(5.0, 0.0, 0.0),
        },
        {
            glm::dvec3(0.0, 0.0, 0.1),
            glm::dvec3(0.0, 0.0, 2.9),
            glm::dvec3(0.0, 0.1, 2.9),
            glm::dvec3(0.0, 0.1, 0.1),
            glm::dvec3(5.0, 0.1, 2.9),
            glm::dvec3(5.0, 0.1, 0.1),
            glm::dvec3(5.0, 0.0, 2.9),
            glm::dvec3(5.0, 0.0, 0.1),
        },
        {
            glm::dvec3(0.0, 3.9, 0.1),
            glm::dvec3(0.0, 3.9, 2.9),
            glm::dvec3(0.0, 4.0, 2.9),
            glm::dvec3(0.0, 4.0, 0.1),
            glm::dvec3(5.0, 4.0, 2.9),
            glm::dvec3(5.0, 4.0, 0.1),
            glm::dvec3(5.0, 3.9, 2.9),
            glm::dvec3(5.0, 3.9, 0.1),
        },
        {
            glm::dvec3(0.0, 0.1, 0.1),
            glm::dvec3(0.0, 0.1, 2.9),
            glm::dvec3(0.0, 3.9, 2.9),
            glm::dvec3(0.0, 3.9, 0.1),
            glm::dvec3(0.1, 3.9, 2.9),
            glm::dvec3(0.1, 3.9, 0.1),
            glm::dvec3(0.1, 0.1, 2.9),
            glm::dvec3(0.1, 0.1, 0.1),
        },
        {
            glm::dvec3(4.9, 0.1, 0.1),
            glm::dvec3(4.9, 0.1, 2.9),
            glm::dvec3(4.9, 3.9, 2.9),
            glm::dvec3(4.9, 3.9, 0.1),
            glm::dvec3(5.0, 3.9, 2.9),
            glm::dvec3(5.0, 3.9, 0.1),
            glm::dvec3(5.0, 0.1, 2.9),
            glm::dvec3(5.0, 0.1, 0.1),
        },
        {
            glm::dvec3(0.0, 0.0, 2.9),
            glm::dvec3(0.0, 0.0, 3.0),
            glm::dvec3(0.0, 4.0, 3.0),
            glm::dvec3(0.0, 4.0, 2.9),
            glm::dvec3(5.0, 4.0, 3.0),
            glm::dvec3(5.0, 4.0, 2.9),
            glm::dvec3(5.0, 0.0, 3.0),
            glm::dvec3(5.0, 0.0, 2.9),
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

    // std::vector<fuzzybools::BuildingElement> buildingElements(geoms.size());
    // for (int buildingElementId = 0; buildingElementId < geoms.size(); ++buildingElementId)
    // {
    //     buildingElements[buildingElementId].id = buildingElementId;
    //     buildingElements[buildingElementId].geometry = geoms[buildingElementId];
    // }

    // fuzzybools::GetSpacesGeomsByBuildingElements(buildingElements);

    return 0;
}