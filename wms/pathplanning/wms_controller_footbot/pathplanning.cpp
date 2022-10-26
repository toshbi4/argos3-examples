#include <cstdlib>

#include "pathplanning.h"

PathPlanning::PathPlanning():
   var{0},
   pointsCount{2},
   m_pcRNG(NULL),
   routesCreated {0}
{

}

void PathPlanning::init(std::vector<FreeRectangle> aFreeSpace,
                        CVector3 startPos,
                        bool aHasCargo,
                        uint8_t aMotionType,
                        CVector2 *aLoadCoords,
                        CVector2 *aUnloadCoords){

   startRobotPos = startPos;
   /* Distribute uniformly the items in the environment */
   m_cGoalsPos = robotPath(aFreeSpace, aHasCargo, aMotionType, aLoadCoords, aUnloadCoords);
}

std::vector<PathPlanning::RoutePoint> PathPlanning::robotPath(std::vector<FreeRectangle> freeSpace,
                                                              bool aHasCargo,
                                                              uint8_t aMotionType,
                                                              CVector2 *aLoadCoords,
                                                              CVector2 *aUnloadCoords){

   m_pcRNG = CRandom::CreateRNG("argos");
   std::vector<RoutePoint> oneRobotPath;
   CVector2 loadCoords;
   CVector2 unloadCoords;

   ++routesCreated;

   if (aLoadCoords == nullptr){

      uint8_t freeRectangleID = freeSpace.size() - 1;
      loadCoords = CVector2(m_pcRNG->Uniform(CRange(freeSpace[freeRectangleID].firstCoord.GetX(),
                                                    freeSpace[freeRectangleID].secondCoord.GetX())),
                            m_pcRNG->Uniform(CRange(freeSpace[freeRectangleID].firstCoord.GetY(),
                                                    freeSpace[freeRectangleID].secondCoord.GetY())));
      freeRectangleID = rand() % (freeSpace.size() - 1);
      unloadCoords = CVector2(m_pcRNG->Uniform(CRange(freeSpace[freeRectangleID].firstCoord.GetX(),
                                                      freeSpace[freeRectangleID].secondCoord.GetX())),
                              m_pcRNG->Uniform(CRange(freeSpace[freeRectangleID].firstCoord.GetY(),
                                                      freeSpace[freeRectangleID].secondCoord.GetY())));
      std::cout << "Random goals." << std::endl;

   } else {

      loadCoords = *aLoadCoords;
      unloadCoords = *aUnloadCoords;
      std::cout << "Predefined goals." << std::endl;

   }

   if (aMotionType == 1){ // diagonal
    oneRobotPath.push_back(RoutePoint(CVector2(4.2f, startRobotPos.GetY()), 0));
    oneRobotPath.push_back(RoutePoint(loadCoords, 1));
    oneRobotPath.push_back(RoutePoint(CVector2(4.0f, unloadCoords.GetY()), 0));
    oneRobotPath.push_back(RoutePoint(unloadCoords, 2));
    pointsCount = 4;
   } else { // perpendicular
    oneRobotPath.push_back(RoutePoint(CVector2(loadCoords.GetX(), startRobotPos.GetY()), 0));
    oneRobotPath.push_back(RoutePoint(loadCoords, 1));
    oneRobotPath.push_back(RoutePoint(CVector2(loadCoords.GetX(), unloadCoords.GetY()), 0));
    oneRobotPath.push_back(RoutePoint(CVector2(4.0f, unloadCoords.GetY()), 0));
    oneRobotPath.push_back(RoutePoint(unloadCoords, 2));
    pointsCount = 5;
   }


   // std::cout << freeRectangleID << std::endl;
   std::cout << "GoalPos: " << oneRobotPath[1].coords.GetX() << " " << oneRobotPath[1].coords.GetY() << std::endl;

   return oneRobotPath;
}

std::vector<PathPlanning::RoutePoint> PathPlanning::getGoals(){
   return m_cGoalsPos;
}

uint16_t PathPlanning::getPointsCount(){
   return pointsCount;
}
