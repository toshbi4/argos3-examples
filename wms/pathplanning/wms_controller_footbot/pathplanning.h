#ifndef PATHPLANNING_H
#define PATHPLANNING_H

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

class PathPlanning {

public:

   struct RoutePoint {

      RoutePoint(CVector2 aCoords, uint8_t aType):
         coords {aCoords},
         type {aType}{
      }

      CVector2 coords;
      uint8_t type; // 0 - common, 1 - load, 2 - unload.
   };

   PathPlanning();
   void init(std::vector<FreeRectangle> freeSpace,
             CVector3 startPos,
             bool aHasCargo,
             uint8_t aMotionType,
             CVector2 *aLoadCoords = nullptr,
             CVector2 *aUnloadCoords = nullptr);

   std::vector<RoutePoint> getGoals();
   uint16_t getPointsCount();

   uint16_t getRoutesCreated(){
       return routesCreated;
   }

private:

   std::vector<RoutePoint> robotPath(std::vector<FreeRectangle> freeSpace,
                                     bool aHasCargo,
                                     uint8_t aMotionType,
                                     CVector2 *aLoadCoords = nullptr,
                                     CVector2 *aUnloadCoords = nullptr);

   uint8_t var;
   uint16_t pointsCount;
   CRandom::CRNG* m_pcRNG;
   std::vector<RoutePoint> m_cGoalsPos;
   CVector3 startRobotPos;
   uint16_t routesCreated;
};

#endif //PATHPLANNING_H
