#include "wms_qt_user_functions.h"
#include <wms/wms_footbot/wms_controller_footbot/wms_controller.h>
#include <argos3/core/simulator/entity/controllable_entity.h>

using namespace argos;

/****************************************/
/****************************************/

WmsQTUserFunctions::WmsQTUserFunctions() {
   RegisterUserFunction<WmsQTUserFunctions,CFootBotEntity>(&WmsQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void WmsQTUserFunctions::Draw(CFootBotEntity& c_entity) {
   WmsController& cController = dynamic_cast<WmsController&>(c_entity.GetControllableEntity().GetController());
   WmsController::SFoodData& sFoodData = cController.GetFoodData();
   if(sFoodData.HasFoodItem) {
      DrawCylinder(
         CVector3(0.0f, 0.0f, 0.3f), 
         CQuaternion(),
         0.1f,
         0.05f,
         CColor::BLACK);
   }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(WmsQTUserFunctions, "wms_qt_user_functions")
