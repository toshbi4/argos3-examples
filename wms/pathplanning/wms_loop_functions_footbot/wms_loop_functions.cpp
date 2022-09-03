#include "wms_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <wms/pathplanning/wms_controller_footbot/wms_controller.h>
#include <argos3/plugins/simulator/entities/box_entity.h>

/****************************************/
/****************************************/

WmsLoopFunctions::WmsLoopFunctions() :
   m_cForagingArenaSideX(4.0f, 8.5f),
   m_cForagingArenaSideY(-6.5f, 6.5f),
   m_pcFloor(NULL),
   m_unCollectedFood(0),
   pathPlanning{},
   pointsCount{2},
   borderIdNumber{0}
{
}

/****************************************/
/****************************************/

void WmsLoopFunctions::Init(TConfigurationNode& t_node) {

   uint16_t robots_num = 0;

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

      CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
      std::cout << m_cFootbots.size() << std::endl;
      robots_num = m_cFootbots.size();
      m_unCollectedFood = robots_num;

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

   //Create obstacles and set a free place
   createBorder(CVector2(-9.1f, -6.0f), CVector2(-9.0f, 6.0f));

   createBorder(CVector2(-9.0f, 6.0f), CVector2(4.0f, 6.1f));
   freeSpace.push_back(FreeRectangle{CVector2(-9.0f, 5.5f), CVector2(4.0f, 6.0f)});
   createBorder(CVector2(-9.0f, 4.5f), CVector2(4.0f, 5.5f));

   freeSpace.push_back(FreeRectangle{CVector2(-9.0f, 4.0f), CVector2(4.0f, 4.5f)});
   createBorder(CVector2(-9.0f, 3.0f), CVector2(4.0f, 4.0f));

   freeSpace.push_back(FreeRectangle{CVector2(-9.0f, 2.5f), CVector2(4.0f, 3.0f)});
   createBorder(CVector2(-9.0f, 1.5f), CVector2(4.0f, 2.5f));

   freeSpace.push_back(FreeRectangle{CVector2(-9.0f, 1.0f), CVector2(4.0f, 1.5f)});
   createBorder(CVector2(-9.0f, -1.0f), CVector2(4.0f, 1.0f));

   freeSpace.push_back(FreeRectangle{CVector2(-9.0f, -1.5f), CVector2(4.0f, -1.0f)});
   createBorder(CVector2(-9.0f, -2.5f), CVector2(4.0f, -1.5f));

   freeSpace.push_back(FreeRectangle{CVector2(-9.0f, -3.0f), CVector2(4.0f, -2.5f)});
   createBorder(CVector2(-9.0f, -4.0f), CVector2(4.0f, -3.0f));

   freeSpace.push_back(FreeRectangle{CVector2(-9.0f, -4.5f), CVector2(4.0f, -4.0f)});
   createBorder(CVector2(-9.0f, -5.5f), CVector2(4.0f, -4.5f));

   freeSpace.push_back(FreeRectangle{CVector2(-9.0f, -6.0f), CVector2(4.0f, -5.5f)});
   createBorder(CVector2(-9.0f, -6.1f), CVector2(4.0f, -6.0f));

   createBorder(CVector2(4.0f, 6.5f), CVector2(9.0f, 6.6f));
   createBorder(CVector2(3.9f, 6.0f), CVector2(4.0f, 6.5f));
   freeSpace.push_back(FreeRectangle{CVector2(4.0f, -6.5f), CVector2(9.0f, 6.5f), 1});
   createBorder(CVector2(3.9f, -6.5f), CVector2(4.0f, -6.0f));
   createBorder(CVector2(4.0f, -6.6f), CVector2(9.0f, -6.5f));
   createBorder(CVector2(9.0f, -6.5f), CVector2(9.1f, 6.5f));

   pathPlanning.init(robots_num, pointsCount, freeSpace);
}

void WmsLoopFunctions::createBorder(CVector2 firstCoordinate, CVector2 secondCoordinate){

   CVector2 boxCenter {CVector2((secondCoordinate - firstCoordinate) * 0.5 + firstCoordinate)};
   CVector3 boxSize {secondCoordinate.GetX() - firstCoordinate.GetX(),
                     secondCoordinate.GetY() - firstCoordinate.GetY(),
                     0.1f};

   // Custom desctribution
   try {
      CBoxEntity* pcBox;
      std::ostringstream cBoxId;

         /* Make the id */
         cBoxId.str("");
         cBoxId << "box" << borderIdNumber;
         /* Create the box in the origin and add it to ARGoS space */
         pcBox = new CBoxEntity(
            cBoxId.str(),                                    //str_id
            CVector3(boxCenter.GetX(), boxCenter.GetY(), 0), //center c_position
            CQuaternion(0.0f,0.0f,0.0f,0.0f),                //c_orientation
            false,                                           //bool b_movable
            boxSize);                                        //c_size

         AddEntity(*pcBox);

   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("While placing robots in a line", ex);
   }

   borderIdNumber++;

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
//   for(UInt32 i = 0; i < m_cGpathPlanning.getGoals().size(); ++i) {
//      pathPlanning.getGoals()[i] = goals[i];
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

      if((c_position_on_plane - pathPlanning.getGoals()[robot_id][cController.pathPointNumber]).SquareLength() < m_fFoodSquareRadius) {
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
   /* Get a pointer to the floor entity */
   m_pcFloor = &GetSpace().GetFloorEntity();

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

      /* Get food data */
      WmsController::SFoodData& sFoodData = cController.GetFoodData();

      if (cController.pathPointNumber <= pointsCount - 1) {
         sFoodData.HasFoodItem = false;
         cController.setCoordinates(cPos, cOrient, pathPlanning.getGoals()[robot_id][cController.pathPointNumber]);
      }

      /* The foot-bot has a food item */
      if(sFoodData.HasFoodItem) {

      }
      else {
         /* Check whether the foot-bot is on a food item */
         bool bDone = false;
         if((cPos - pathPlanning.getGoals()[robot_id][cController.pathPointNumber]).SquareLength() < m_fFoodSquareRadius) {
            /* The foot-bot is now carrying an item */
            sFoodData.HasFoodItem = true;
            //sFoodData.FoodItemIdx = i;
            //--m_unCollectedFood;
            std::cout << m_unCollectedFood << std::endl;

            if (m_unCollectedFood == 0){
                std::chrono::microseconds elapsed = micros() - start;
                std::cout << std::to_string(elapsed.count()) << std::endl;
            }

            /* We are done */
            bDone = true;

            /* The floor texture must be updated */
            m_pcFloor->SetChanged();

            pathPlanning.reachedPoint(robot_id);
            cController.pathPointNumber++;
         }
      }
   }
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(WmsLoopFunctions, "wms_loop_functions")
