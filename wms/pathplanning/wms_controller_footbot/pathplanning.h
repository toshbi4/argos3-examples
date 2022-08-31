#ifndef PATHPLANNING_H
#define PATHPLANNING_H

#include <stdint.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <vector>

using namespace argos;

class PathPlanning {

public:

    PathPlanning(CRange<Real> xRange, CRange<Real> yRange);
    void init(uint16_t robots_num);
    std::vector<CVector2> getGoals();
    void reachedPoint(uint16_t robot_id);

private:

    uint8_t var;
    CRandom::CRNG* m_pcRNG;
    std::vector<CVector2> m_cGoalsPos;
    CRange<Real> xRange;
    CRange<Real> yRange;

};

#endif //PATHPLANNING_H
