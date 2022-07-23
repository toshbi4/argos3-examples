#ifndef FORAGING_LOOP_FUNCTIONS_H
#define FORAGING_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <chrono>

using namespace argos;

class WmsLoopFunctions : public CLoopFunctions {

public:

   using mcs = std::chrono::microseconds;

   WmsLoopFunctions();
   virtual ~WmsLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();
   virtual CColor GetFloorColor(const CVector2& c_position_on_plane);
   virtual void PreStep();

private:

   // Get time stamp in microseconds.
   std::chrono::microseconds micros()
   {
       std::chrono::microseconds us = std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::high_resolution_clock::now().time_since_epoch());
       return us;
   }

   Real m_fFoodSquareRadius;
   CRange<Real> m_cForagingArenaSideX, m_cForagingArenaSideY;
   std::vector<CVector2> m_cGoalsPos;
   CFloorEntity* m_pcFloor;
   CRandom::CRNG* m_pcRNG;

   std::string m_strOutput;
   std::ofstream m_cOutput;

   UInt32 m_unCollectedFood;
   std::chrono::microseconds start;
};

#endif
