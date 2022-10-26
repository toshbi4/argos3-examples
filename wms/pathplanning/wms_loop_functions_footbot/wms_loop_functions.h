#ifndef FORAGING_LOOP_FUNCTIONS_H
#define FORAGING_LOOP_FUNCTIONS_H

#include <chrono>

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>

using namespace argos;

class WmsLoopFunctions : public CLoopFunctions {

public:

   using mcs = std::chrono::microseconds;

   WmsLoopFunctions();
   ~WmsLoopFunctions();

   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();
   virtual CColor GetFloorColor(const CVector2& c_position_on_plane);
   virtual void PreStep();

   typedef std::map<CFootBotEntity*, std::vector<CVector3> > TWaypointMap;
   TWaypointMap m_tWaypoints;
   inline const TWaypointMap& GetWaypoints() const {
      return m_tWaypoints;
   }

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
   uint8_t motionType; // 0 - perpendicular, 1 - diagonal
   CRange<Real> m_cForagingArenaSideX, m_cForagingArenaSideY;
   CFloorEntity* m_pcFloor;

   std::chrono::microseconds start;

   std::vector<FreeRectangle> freeSpace;
   uint8_t borderIdNumber;
   uint16_t loadedRobots;
   std::vector<CVector2> loadPoints;
   std::vector<CVector2> unloadPoints;
   bool goalsPredefined;
   uint16_t taskNumber;
};

#endif
