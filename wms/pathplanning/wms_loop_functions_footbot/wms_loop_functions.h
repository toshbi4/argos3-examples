#ifndef FORAGING_LOOP_FUNCTIONS_H
#define FORAGING_LOOP_FUNCTIONS_H

#include <chrono>

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>
#include <wms/pathplanning/wms_controller_footbot/pathplanning.h>

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

   void createScene();
   void createBorder(CVector2 firstCoordinate, CVector2 secondCoordinate);

   // Get time stamp in microseconds.
   std::chrono::microseconds micros()
   {
       std::chrono::microseconds us = std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::high_resolution_clock::now().time_since_epoch());
       return us;
   }

   Real m_fFoodSquareRadius;
   CRange<Real> m_cForagingArenaSideX, m_cForagingArenaSideY;
   CFloorEntity* m_pcFloor;

//   std::string m_strOutput;
//   std::ofstream m_cOutput;

   PathPlanning pathPlanning;
   uint16_t pointsCount;
   std::chrono::microseconds start;

   std::vector<FreeRectangle> freeSpace;
   uint8_t borderIdNumber;
};

#endif