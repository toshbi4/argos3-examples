#ifndef PATHPLANNING_H
#define PATHPLANNING_H

#include <cstdlib>
#include <stdint.h>
#include <vector>

#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/vector3.h>

using namespace argos;

struct FreeRectangle {
   CVector2 firstCoord;
   CVector2 secondCoord;
   uint8_t type = 0; // 0 - unloading, 1 - loading
};

struct PathPlanning {

public:

   struct RoutePoint {

      RoutePoint(CVector2 aCoords, uint8_t aType):
         coords {aCoords},
         type {aType}{
      }

      CVector2 coords;
      uint8_t type; // 0 - common, 1 - load, 2 - unload.
   };

   PathPlanning():
      routesCreated {0},
      pointsCount{2}
   {

   }

   uint16_t routesCreated;
   uint16_t pointsCount;
   std::vector<RoutePoint> m_cGoalsPos;

};

#endif //PATHPLANNING_H
