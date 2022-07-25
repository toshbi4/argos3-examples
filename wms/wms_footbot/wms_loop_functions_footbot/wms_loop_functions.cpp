#include "wms_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <wms/wms_footbot/wms_controller_footbot/wms_controller.h>

/****************************************/
/****************************************/

WmsLoopFunctions::WmsLoopFunctions() :
   m_cForagingArenaSideX(-48.0f, 48.0f),
   m_cForagingArenaSideY(-48.0f, 48.0f),
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
      m_unCollectedFood = robots_num;

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

    if ((GetSpace().GetSimulationClock() > 0) && (start == mcs(0))){
        start = micros();
    }

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

      }
      else {
          /* Check whether the foot-bot is on a food item */
          bool bDone = false;
          for(size_t i = 0; i < m_cGoalsPos.size() && !bDone; ++i) {
             if((cPos - m_cGoalsPos[i]).SquareLength() < m_fFoodSquareRadius) {
                /* The foot-bot is now carrying an item */
                sFoodData.HasFoodItem = true;
                sFoodData.FoodItemIdx = i;
                --m_unCollectedFood;
                std::cout << m_unCollectedFood << std::endl;

                if (m_unCollectedFood == 0){
                    std::chrono::microseconds elapsed = micros() - start;
                    std::cout << std::to_string(elapsed.count()) << std::endl;
                }

                /* We are done */
                bDone = true;
             }
         }
      }
   }
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(WmsLoopFunctions, "wms_loop_functions")
