#include "pathplanning.h"

PathPlanning::PathPlanning(CRange<Real> xR, CRange<Real> yR):
    var{0},
    m_pcRNG(NULL),
    xRange(xR),
    yRange(yR)
{

}

void PathPlanning::init(uint16_t robots_num, uint16_t pointsCount){

    m_pcRNG = CRandom::CreateRNG("argos");
    std::vector<CVector2> oneRobotPath;

    /* Distribute uniformly the items in the environment */
    for(UInt32 i = 0; i < robots_num; ++i) {

       oneRobotPath.clear();

       for (UInt32 j = 0; j < pointsCount; ++j){
           oneRobotPath.push_back(CVector2(m_pcRNG->Uniform(xRange),
                                           m_pcRNG->Uniform(yRange)));
       }

       m_cGoalsPos.push_back(oneRobotPath);
    }
}

std::vector<std::vector<CVector2>> PathPlanning::getGoals(){
    return m_cGoalsPos;
}

void PathPlanning::reachedPoint(uint16_t robot_id){
    //m_cGoalsPos[robot_id] = CVector2(m_pcRNG->Uniform(xRange),
    //                                 m_pcRNG->Uniform(yRange));
}
