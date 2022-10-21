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

   PathPlanning();
   void init(std::vector<FreeRectangle> freeSpace,
             CVector3 startPos,
             bool aHasCargo,
             uint8_t aMotionType);
   void test(std::vector<FreeRectangle> freeSpace,
             CVector3 startPos,
             bool aHasCargo,
             uint8_t aMotionType);
   std::vector<CVector2> robotPath(std::vector<FreeRectangle> freeSpace,
                                   bool aHasCargo,
                                   uint8_t aMotionType);
   std::vector<CVector2> getGoals();
   uint16_t getPointsCount();

   uint16_t getRoutesCreated(){
       return routesCreated;
   }

private:

   uint8_t var;
   uint16_t pointsCount;
   CRandom::CRNG* m_pcRNG;
   std::vector<CVector2> m_cGoalsPos;
   CVector3 startRobotPos;
   uint16_t routesCreated;
};

#endif //PATHPLANNING_H
