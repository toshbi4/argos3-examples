#include "wms_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <wms/wms_footbot/wms_controller_footbot/wms_controller.h>

/****************************************/
/****************************************/

WmsLoopFunctions::WmsLoopFunctions() :
   m_cForagingArenaSideX(-10.0f, 10.0f),
   m_cForagingArenaSideY(-7.5f, 7.5f),
   m_pcFloor(NULL),
   m_pcRNG(NULL),
   m_unCollectedFood(0)
{
}

/****************************************/
/****************************************/

void WmsLoopFunctions::Init(TConfigurationNode& t_node) {

    try {

      TConfigurationNode& tWms = GetNode(t_node, "wms");
      /* Get a pointer to the floor entity */
      m_pcFloor = &GetSpace().GetFloorEntity();
      /* Get the number of food items we want to be scattered from XML */
      UInt32 unFoodItems;
      GetNodeAttribute(tWms, "items", unFoodItems);
      /* Get the number of food items we want to be scattered from XML */
      GetNodeAttribute(tWms, "radius", m_fFoodSquareRadius);
      m_fFoodSquareRadius *= m_fFoodSquareRadius;
      /* Create a new RNG */
      m_pcRNG = CRandom::CreateRNG("argos");


      CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
      std::cout << m_cFootbots.size() << std::endl;
      uint16_t robots_num = m_cFootbots.size();

      /* Distribute uniformly the items in the environment */
      for(UInt32 i = 0; i < robots_num; ++i) {
         m_cGoalsPos.push_back(CVector2(m_pcRNG->Uniform(m_cForagingArenaSideX),
                               m_pcRNG->Uniform(m_cForagingArenaSideY)));
      }
      /* Get the output file name from XML */
      GetNodeAttribute(tWms, "output", m_strOutput);
      /* Open the file, erasing its contents */
      m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
      m_cOutput << "# clock\twalking\tresting\tcollected_food\tenergy" << std::endl;

      TConfigurationNodeIterator itDistr;
      for(itDistr = itDistr.begin(&t_node);
          itDistr != itDistr.end();
          ++itDistr) {

          /* Get current node */
          TConfigurationNode& tDistr = *itDistr;

          if(itDistr->Value() == "workspace") {
              std::cout << "workspase was found." << std::endl;
              CVector2 cCenter;
              GetNodeAttribute(tDistr, "center", cCenter);
              std::cout << cCenter.X << std::endl;
          }
      }

   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
   }
}

/****************************************/
/****************************************/

void WmsLoopFunctions::Reset() {
   /* Zero the counters */
   m_unCollectedFood = 0;
   /* Close the file */
   m_cOutput.close();
   /* Open the file, erasing its contents */
   m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
   m_cOutput << "# clock\twalking\tresting\tcollected_food\tenergy" << std::endl;
   /* Distribute uniformly the items in the environment */
//   for(UInt32 i = 0; i < m_cGoalsPos.size(); ++i) {
//      m_cGoalsPos[i] = goals[i];
//   }
}

/****************************************/
/****************************************/

void WmsLoopFunctions::Destroy() {
   /* Close the file */
   m_cOutput.close();
}

/****************************************/
/****************************************/

CColor WmsLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
//   if(c_position_on_plane.GetX() > 8.0f) {
//      return CColor::GRAY50;
//   }
   for(UInt32 i = 0; i < m_cGoalsPos.size(); ++i) {
      if((c_position_on_plane - m_cGoalsPos[i]).SquareLength() < m_fFoodSquareRadius) {
         return CColor::BLACK;
      }
   }
   return CColor::WHITE;
}

/****************************************/
/****************************************/

void WmsLoopFunctions::PreStep() {
   /* Logic to pick and drop food items */
   /*
    * If a robot is in the nest, drop the food item
    * If a robot is on a food item, pick it
    * Each robot can carry only one food item per time
    */
   UInt32 unWalkingFBs = 0;
   UInt32 unRestingFBs = 0;
   /* Check whether a robot is on a food item */
   CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

   uint16_t robot_id = -1;
   for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
       it != m_cFootbots.end();
       ++it) {
       robot_id += 1;
      /* Get handle to foot-bot entity and controller */
      CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
      WmsController& cController = dynamic_cast<WmsController&>(cFootBot.GetControllableEntity().GetController());
      /* Count how many foot-bots are in which state */
      if(! cController.IsResting()) ++unWalkingFBs;
      else ++unRestingFBs;
      /* Get the position of the foot-bot on the ground as a CVector2 */
      CVector2 cPos;
      CQuaternion cOrient;

      cPos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
               cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
      cOrient = cFootBot.GetEmbodiedEntity().GetOriginAnchor().Orientation;



      cController.setCoordinates(cPos, cOrient, m_cGoalsPos[robot_id]);

      /* Get food data */
      WmsController::SFoodData& sFoodData = cController.GetFoodData();
      /* The foot-bot has a food item */
      if(sFoodData.HasFoodItem) {
         /* Check whether the foot-bot is in the nest */
         if(cPos.GetX() > 8.0f) {
            /* Place a new food item on the ground */
            m_cGoalsPos[sFoodData.FoodItemIdx].Set(/*m_pcRNG->Uniform(m_cForagingArenaSideX)*/5.0,
                                                  /*m_pcRNG->Uniform(m_cForagingArenaSideY)*/0.0);
            /* Drop the food item */
            sFoodData.HasFoodItem = false;
            sFoodData.FoodItemIdx = 0;
            ++sFoodData.TotalFoodItems;
            /* Increase the energy and food count */
            ++m_unCollectedFood;
            /* The floor texture must be updated */
            m_pcFloor->SetChanged();
         }
      }
      else {
         /* The foot-bot has no food item */
         /* Check whether the foot-bot is out of the nest */
         if(cPos.GetX() < 8.0f) {
            /* Check whether the foot-bot is on a food item */
            bool bDone = false;
            for(size_t i = 0; i < m_cGoalsPos.size() && !bDone; ++i) {
               if((cPos - m_cGoalsPos[i]).SquareLength() < m_fFoodSquareRadius) {
                  /* If so, we move that item out of sight */
                  m_cGoalsPos[i].Set(100.0f, 100.f);
                  /* The foot-bot is now carrying an item */
                  sFoodData.HasFoodItem = true;
                  sFoodData.FoodItemIdx = i;
                  /* The floor texture must be updated */
                  m_pcFloor->SetChanged();
                  /* We are done */
                  bDone = true;
               }
            }
         }
      }
   }
   /* Update energy expediture due to walking robots */
   /* Output stuff to file */
   m_cOutput << GetSpace().GetSimulationClock() << "\t"
             << unWalkingFBs << "\t"
             << unRestingFBs << "\t"
             << m_unCollectedFood << "\t"
             << 0 << std::endl;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(WmsLoopFunctions, "wms_loop_functions")
