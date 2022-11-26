#include <cstdlib>

#include "pathplanning.h"

int16_t realToCellCoordX(float x){
   return round((x + 0.2f) / 0.4f);
}
int16_t realToCellCoordY(float y){
   return round((y + 0.1f) / 0.4f);
}
float cellToRealCoordX(int16_t x){
   return x * 0.4f - 0.2f;
}
float cellToRealCoordY(int16_t y){
   return y * 0.4f - 0.1f;
}

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
   std::cout << routesCreated << std::endl;

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
      // std::cout << "Random goals." << std::endl;

   } else {

      loadCoords = *aLoadCoords;
      unloadCoords = *aUnloadCoords;
      // std::cout << "Predefined goals." << std::endl;

   }

   if (aMotionType == 1){ // diagonal

    pointsCount = 0;

    //std::cout << "cell load: " << realToCellCoordX(loadCoords.GetX()) << std::endl;
    int8_t sign = ((realToCellCoordX(loadCoords.GetX()) - realToCellCoordX(startRobotPos.GetX())) > 0) ? 1 : -1;
    for (int xcell=realToCellCoordX(startRobotPos.GetX());
         xcell != realToCellCoordX(loadCoords.GetX()) + sign;
         xcell += sign){

       //std::cout << xcell << std::endl;
       int16_t ycell = realToCellCoordY(startRobotPos.GetY());
       oneRobotPath.push_back(RoutePoint(CVector2(cellToRealCoordX(xcell), cellToRealCoordY(ycell)), 0));
       pointsCount += 1;

    }

    sign = ((realToCellCoordY(loadCoords.GetY()) - realToCellCoordY(oneRobotPath.back().coords.GetY())) > 0) ? 1 : -1;
    for (int ycell=realToCellCoordY(oneRobotPath.back().coords.GetY());
         ycell != realToCellCoordY(loadCoords.GetY()) + sign;
         ycell += sign){

       int16_t xcell = realToCellCoordX(oneRobotPath.back().coords.GetX());
       oneRobotPath.push_back(RoutePoint(CVector2(cellToRealCoordX(xcell), cellToRealCoordY(ycell)), 0));
       pointsCount += 1;

    }

    oneRobotPath.push_back(RoutePoint(loadCoords, 1));
    pointsCount += 1;

    sign = ((realToCellCoordY(unloadCoords.GetY()) - realToCellCoordY(loadCoords.GetY())) > 0) ? 1 : -1;
    for (int ycell=realToCellCoordY(loadCoords.GetY());
         ycell != realToCellCoordY(unloadCoords.GetY()) + sign;
         ycell += sign){

       int16_t xcell = realToCellCoordX(loadCoords.GetX());
       oneRobotPath.push_back(RoutePoint(CVector2(cellToRealCoordX(xcell), cellToRealCoordY(ycell)), 0));
       pointsCount += 1;

    }

    sign = ((realToCellCoordX(unloadCoords.GetX()) - realToCellCoordX(oneRobotPath.back().coords.GetX())) > 0) ? 1 : -1;
    for (int xcell=realToCellCoordX(oneRobotPath.back().coords.GetX());
         xcell != realToCellCoordX(unloadCoords.GetX()) + sign;
         xcell += sign){

       int16_t ycell = realToCellCoordY(oneRobotPath.back().coords.GetY());
       oneRobotPath.push_back(RoutePoint(CVector2(cellToRealCoordX(xcell), cellToRealCoordY(ycell)), 0));
       pointsCount += 1;

    }

    oneRobotPath.push_back(RoutePoint(unloadCoords, 2));
    pointsCount += 1;
   } else { // perpendicular
    oneRobotPath.push_back(RoutePoint(CVector2(loadCoords.GetX(), startRobotPos.GetY()), 0));
    oneRobotPath.push_back(RoutePoint(loadCoords, 1));
    oneRobotPath.push_back(RoutePoint(CVector2(loadCoords.GetX(), unloadCoords.GetY()), 0));
    oneRobotPath.push_back(RoutePoint(CVector2(4.0f, unloadCoords.GetY()), 0));
    oneRobotPath.push_back(RoutePoint(unloadCoords, 2));
    pointsCount = 5;
   }


   // std::cout << freeRectangleID << std::endl;
   // std::cout << "GoalPos: " << oneRobotPath[1].coords.GetX() << " " << oneRobotPath[1].coords.GetY() << std::endl;

   return oneRobotPath;
}

std::vector<PathPlanning::RoutePoint> PathPlanning::getGoals(){
   return m_cGoalsPos;
}

uint16_t PathPlanning::getPointsCount(){
   return pointsCount;
}
