#include "pathplanning.h"

PathPlanning::PathPlanning(CRange<Real> xR, CRange<Real> yR):
    var{0},
    m_pcRNG(NULL),
    xRange(xR),
    yRange(yR)
{

}

void PathPlanning::init(uint16_t robots_num){

    m_pcRNG = CRandom::CreateRNG("argos");

    /* Distribute uniformly the items in the environment */
    for(UInt32 i = 0; i < robots_num; ++i) {
       m_cGoalsPos.push_back(CVector2(m_pcRNG->Uniform(xRange),
                             m_pcRNG->Uniform(yRange)));
    }
}

std::vector<CVector2> PathPlanning::getGoals(){
    return m_cGoalsPos;
}

void PathPlanning::reachedPoint(uint16_t robot_id){
    //m_pcRNG = CRandom::CreateRNG("argos");
    m_cGoalsPos[robot_id] = CVector2(5, 0);
}
