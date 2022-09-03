#ifndef PATHPLANNING_H
#define PATHPLANNING_H

#include <stdint.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <vector>

using namespace argos;

struct FreeRectangle {
     CVector2 firstCoord;
     CVector2 secondCoord;
     uint8_t type = 0; // 0 - unloading, 1 - loading
};

class PathPlanning {

public:

    PathPlanning();
    void init(uint16_t robots_num, uint16_t pointsCount, std::vector<FreeRectangle> freeSpace);
    std::vector<std::vector<CVector2>> getGoals();
    void reachedPoint(uint16_t robot_id);

private:

    uint8_t var;
    CRandom::CRNG* m_pcRNG;
    std::vector<std::vector<CVector2>> m_cGoalsPos;

};

#endif //PATHPLANNING_H
