#include "wms_qt_user_functions.h"
#include "wms/pathplanning/wms_controller_footbot/wms_controller.h"
#include "wms/pathplanning/wms_loop_functions_footbot/wms_loop_functions.h"
#include <argos3/core/simulator/entity/controllable_entity.h>

using namespace argos;

/****************************************/
/****************************************/

WmsQTUserFunctions::WmsQTUserFunctions():
   m_cTrajLF(dynamic_cast<WmsLoopFunctions&>(CSimulator::GetInstance().GetLoopFunctions()))
{
   RegisterUserFunction<WmsQTUserFunctions,CFootBotEntity>(&WmsQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void WmsQTUserFunctions::Draw(CFootBotEntity& c_entity) {
   WmsController& cController = dynamic_cast<WmsController&>(c_entity.GetControllableEntity().GetController());
   bool hasCargo = cController.getCargoData();
   if(hasCargo) {
      DrawCylinder(
         CVector3(0.0f, 0.0f, 0.3f), 
         CQuaternion(),
         0.1f,
         0.05f,
         CColor::ORANGE);
   }
}
void WmsQTUserFunctions::DrawInWorld() {

   /* Go through all the robot waypoints and draw them */
   for(WmsLoopFunctions::TWaypointMap::const_iterator it = m_cTrajLF.GetWaypoints().begin();
       it != m_cTrajLF.GetWaypoints().end();
       ++it) {
      DrawWaypoints(it->second);
   }

//      DrawCircle(
//         CVector3(0.0f, 0.0f, 0.1f),
//         CQuaternion(),
//         0.1f,
//         CColor::RED,
//         true);
}

void WmsQTUserFunctions::DrawWaypoints(const std::vector<CVector3>& c_waypoints) {
   /* Start drawing segments when you have at least two points */
   if(c_waypoints.size() > 1) {
      size_t unStart = 0;
      size_t unEnd = 1;
//      while(unEnd < c_waypoints.size()) {
//         std::cout << c_waypoints[unStart].GetZ() << std::endl;
//         DrawRay(CRay3(c_waypoints[unEnd],
//                       c_waypoints[unStart]));
//         ++unStart;
//         ++unEnd;
//      }
   }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(WmsQTUserFunctions, "wms_qt_user_functions")
