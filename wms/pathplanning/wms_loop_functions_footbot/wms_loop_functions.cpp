#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>

#include <wms/pathplanning/wms_controller_footbot/wms_controller.h>
#include "wms_loop_functions.h"

WmsLoopFunctions::WmsLoopFunctions() :
   m_cForagingArenaSideX(4.0f, 8.5f),
   m_cForagingArenaSideY(-6.5f, 6.5f),
   m_pcFloor(NULL),
   borderIdNumber{0}
{
}

void WmsLoopFunctions::Init(TConfigurationNode& t_node) {

   try {

      TConfigurationNode& tWorkspace = GetNode(t_node, "workspace");
      GetNodeAttribute(tWorkspace, "radius", m_fFoodSquareRadius);
      m_fFoodSquareRadius *= m_fFoodSquareRadius;

      CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
      std::cout << m_cFootbots.size() << std::endl;

      m_pcFloor = &GetSpace().GetFloorEntity();

   }
   catch(CARGoSException& ex) {
     THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
   }

   createScene();
   Reset();
}

void WmsLoopFunctions::createScene(){
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

void WmsLoopFunctions::Reset() {


    CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
       it != m_cFootbots.end();
       ++it) {

       CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
       WmsController& cController = dynamic_cast<WmsController&>(cFootBot.GetControllableEntity().GetController());
       cController.pathPlanning.init(freeSpace);
       cController.reset();
    }

    m_pcFloor->SetChanged();

}

void WmsLoopFunctions::Destroy() {

}

CColor WmsLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {

   /* Check whether a robot is on a food item */
   CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

   uint16_t robot_id = 0;
   for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
      it != m_cFootbots.end();
      ++it) {

      /* Get handle to foot-bot entity and controller */
      CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
      WmsController& cController = dynamic_cast<WmsController&>(cFootBot.GetControllableEntity().GetController());

      if((c_position_on_plane - cController.pathPlanning.getGoals()[cController.pathPointNumber]).SquareLength() < m_fFoodSquareRadius) {
         return CColor::BLACK;
      }

      robot_id += 1;

   }

   return CColor::WHITE;
}

void WmsLoopFunctions::PreStep() {
   /* Logic to pick and drop food items */

   if ((GetSpace().GetSimulationClock() > 0) && (start == mcs(0))){
       start = micros();
   }

   /* Check whether a robot is on a food item */
   CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

   uint16_t robot_id = 0;
   for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
      it != m_cFootbots.end();
      ++it) {

      /* Get handle to foot-bot entity and controller */
      CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
      WmsController& cController = dynamic_cast<WmsController&>(cFootBot.GetControllableEntity().GetController());


      /* Get the position of the foot-bot on the ground as a CVector2 */
      CVector2 cPos;
      CQuaternion cOrient;

      cPos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
              cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
      cOrient = cFootBot.GetEmbodiedEntity().GetOriginAnchor().Orientation;

      /* Get cargo data */
      bool hasCargo = cController.getCargoData();

      if (cController.pathPointNumber <= cController.pathPlanning.getPointsCount() - 1) {
         cController.setCargoData(false);
         cController.setCoordinates(cPos, cOrient, cController.pathPlanning.getGoals()[cController.pathPointNumber]);
      }

      if((cPos - cController.pathPlanning.getGoals()[cController.pathPointNumber]).SquareLength() < m_fFoodSquareRadius) {
         cController.setCargoData(true);
         m_pcFloor->SetChanged();
         cController.pathPointNumber++;
      }

      robot_id += 1;

   }
}

REGISTER_LOOP_FUNCTIONS(WmsLoopFunctions, "wms_loop_functions")
