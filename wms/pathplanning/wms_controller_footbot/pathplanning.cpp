#include <cstdlib>

#include "pathplanning.h"

PathPlanning::PathPlanning():
   var{0},
   m_pcRNG(NULL)
{

}

void PathPlanning::init(uint16_t robots_num, uint16_t pointsCount, std::vector<FreeRectangle> freeSpace){

   /* Distribute uniformly the items in the environment */
   for(UInt32 i = 0; i < robots_num; ++i) {

      m_cGoalsPos.push_back(robotPath(i, freeSpace));

   }
}

std::vector<CVector2> PathPlanning::robotPath(uint16_t robot_id, std::vector<FreeRectangle> freeSpace){

   m_pcRNG = CRandom::CreateRNG("argos");
   std::vector<CVector2> oneRobotPath;
   uint8_t freeRectangleID = /*rand() % freeSpace.size() - 1*/ robot_id;

   oneRobotPath.push_back(CVector2(m_pcRNG->Uniform(CRange(freeSpace[freeRectangleID].firstCoord.GetX(),
                                                           freeSpace[freeRectangleID].secondCoord.GetX())),
                                   m_pcRNG->Uniform(CRange(freeSpace[freeRectangleID].firstCoord.GetY(),
                                                           freeSpace[freeRectangleID].secondCoord.GetY()))));

   oneRobotPath.insert(oneRobotPath.begin(), CVector2(m_pcRNG->Uniform(
                                             CRange(freeSpace[freeSpace.size()-1].firstCoord.GetX(),
                                                              freeSpace[freeSpace.size()-1].secondCoord.GetX())),
                                             oneRobotPath[0].GetY()));

   return oneRobotPath;
}

std::vector<std::vector<CVector2>> PathPlanning::getGoals(){
   return m_cGoalsPos;
}
