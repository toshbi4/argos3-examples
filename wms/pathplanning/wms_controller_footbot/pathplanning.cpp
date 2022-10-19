#include <cstdlib>

#include "pathplanning.h"

PathPlanning::PathPlanning():
   var{0},
   pointsCount{2},
   m_pcRNG(NULL),
   routesCreated {0}
{

}

void PathPlanning::init(std::vector<FreeRectangle> freeSpace,
                        CVector3 startPos,
                        bool aHasCargo,
                        uint8_t aMotionType){

   startRobotPos = startPos;
   /* Distribute uniformly the items in the environment */
   m_cGoalsPos = robotPath(freeSpace, aHasCargo, aMotionType);
   ++routesCreated;
}

std::vector<CVector2> PathPlanning::robotPath(std::vector<FreeRectangle> freeSpace,
                                              bool aHasCargo,
                                              uint8_t aMotionType){

   m_pcRNG = CRandom::CreateRNG("argos");
   std::vector<CVector2> oneRobotPath;

   if (aHasCargo){
      uint8_t freeRectangleID = rand() % (freeSpace.size() - 1);
      oneRobotPath.push_back(CVector2(m_pcRNG->Uniform(CRange(freeSpace[freeRectangleID].firstCoord.GetX(),
                                                              freeSpace[freeRectangleID].secondCoord.GetX())),
                                      m_pcRNG->Uniform(CRange(freeSpace[freeRectangleID].firstCoord.GetY(),
                                                              freeSpace[freeRectangleID].secondCoord.GetY()))));
      if (aMotionType == 1){ // diagonal
         oneRobotPath.insert(oneRobotPath.begin(), CVector2(4.0f, oneRobotPath[0].GetY()));
         pointsCount = 2;
      } else { // perpendicular
         oneRobotPath.insert(oneRobotPath.begin(), CVector2(4.0f, oneRobotPath[0].GetY()));
         oneRobotPath.insert(oneRobotPath.begin(), CVector2(startRobotPos.GetX(), oneRobotPath[0].GetY()));
         pointsCount = 3;
      }
   } else {
       uint8_t freeRectangleID = freeSpace.size() - 1;

       if (aMotionType == 1){ // diagonal
          oneRobotPath.insert(oneRobotPath.begin(), CVector2(4.2f, startRobotPos.GetY()));
          pointsCount = 2;
       } else { // perpendicular
          oneRobotPath.insert(oneRobotPath.begin(), CVector2(oneRobotPath[0].GetX(), startRobotPos.GetY()));
          pointsCount = 2;
       }
   }


   // std::cout << freeRectangleID << std::endl;
   // std::cout << "GoalPos: " << oneRobotPath[0].GetX() << " " << oneRobotPath[0].GetY() << std::endl;

   return oneRobotPath;
}

std::vector<CVector2> PathPlanning::getGoals(){
   return m_cGoalsPos;
}

uint16_t PathPlanning::getPointsCount(){
   return pointsCount;
}
