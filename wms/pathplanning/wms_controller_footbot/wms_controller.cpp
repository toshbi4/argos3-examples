#include <argos3/core/utility/configuration/argos_configuration.h>
#include "wms_controller.h"

void WmsController::SWheelTurningParams::Init(TConfigurationNode& t_node) {
   try {
      TurningMechanism = NO_TURN;
      CDegrees cAngle;
      GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
      HardTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
      SoftTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
      NoTurnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "max_speed", MaxSpeed);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
   }
}

WmsController::WmsController() : 
   pathPointNumber(0),
   pathPlanning{},
   stop{false},
   isWaitingNewTask{true},
   changeFloor {false},
   stepInProcess {true},
   speedVector {CVector2(0.0f, 0.0f)},
   m_pcWheels(NULL),
   m_pcPosSens(NULL),
   m_pcRNG(NULL),
   hasCargo (false),
   pid {0.01, 100, -100, 1, 0.01, 0.5}
{}

void WmsController::Init(TConfigurationNode& t_node) {

   try {
      m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
      m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
      m_pcPosSens = GetSensor<CCI_PositioningSensor>("positioning");
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the foot-bot foraging controller for robot \""
                                  << GetId() << "\"", ex);
   }

   m_pcRNG = CRandom::CreateRNG("argos");
}

void WmsController::setCoordinates(CVector2& cGoalPos){

   CVector3 cPos = m_pcPosSens->GetReading().Position;
   CQuaternion cOrient = m_pcPosSens->GetReading().Orientation;

   // yaw (z-axis rotation)
   double siny_cosp = 2 * (cOrient.GetW() * cOrient.GetZ() + cOrient.GetX() * cOrient.GetY());
   double cosy_cosp = 1 - 2 * (cOrient.GetY() * cOrient.GetY() + cOrient.GetZ() * cOrient.GetZ());
   double yaw = std::atan2(siny_cosp, cosy_cosp);
   double cos_yaw = cos(yaw); //radians
   double sin_yaw = sin(yaw); //radians

   double sin_theta = - cos_yaw; // yaw = theta + 90
   double cos_theta = sin_yaw; // yaw = theta + 90

   //    Real rot_values[4]{cos(yaw), sin(yaw), -sin(yaw), cos(yaw)};
   //    CMatrix<2, 2> rot(rot_values);
   CVector2 goalPos(cGoalPos.GetX() - cPos.GetX(), cGoalPos.GetY() - cPos.GetY());

   //= (static_cast<CMatrix<1, 2>>(goalPos) * rot/* + static_cast<CMatrix<1, 2>>(cPos)*/).GetTransposed();
   CMatrix<2, 1> localVec;

   localVec(0) = goalPos.GetX()*cos_theta + goalPos.GetY()*sin_theta;
   localVec(1) = -goalPos.GetX()*sin_theta + goalPos.GetY()*cos_theta;

   // std::cout << "---" << std::endl;
   // std::cout << "yaw = " << yaw << std::endl;
   // std::cout << "cos(yaw) = " << cos_theta << std::endl;
   // std::cout << "sin(yaw) = " << sin_theta << std::endl;
   // std::cout << "localCoords: " << localVec(0) << " " << localVec(1) << std::endl;
   // std::cout << "GoalPos: " << cGoalPos.GetX() << " " << cGoalPos.GetY() << std::endl;

   speedVector.Set(localVec(0), localVec(1));

};

void WmsController::ControlStep() {

   CVector2 speed {0, 2.0};

   if (isWaitingNewTask){
      SetWheelSpeedsFromLocalVector(CVector2(0.0f, 0.0f));
      return;
   }

   if (stop){
      SetWheelSpeedsFromLocalVector(CVector2(0.0f, 0.0f));
      return;
   } else {
      SetWheelSpeedsFromLocalVector(m_sWheelTurningParams.MaxSpeed * speedVector);
   }

//      if ((GetSpace().GetSimulationClock() > 0) && (start == mcs(0))){
//          start = micros();
//      }

   CVector2 cPos = CVector2(m_pcPosSens->GetReading().Position.GetX(),
                            m_pcPosSens->GetReading().Position.GetY());
   CQuaternion cOrient = m_pcPosSens->GetReading().Orientation;


   if (pathPointNumber <= pathPlanning.pointsCount - 1) {
      setCoordinates(pathPlanning.m_cGoalsPos[pathPointNumber].coords);
   }

   if((cPos - pathPlanning.m_cGoalsPos[pathPointNumber].coords).SquareLength() < m_fFoodSquareRadius) {

      // Discrete movements
      stepInProcess = false;
      setStop(true);

      if (!changeFloor){
         changeFloor = true;
      }

      if (pathPlanning.m_cGoalsPos[pathPointNumber].type == 1){
         setCargoData(true);
      } else if (pathPlanning.m_cGoalsPos[pathPointNumber].type == 2){
         setCargoData(false);
      }

      if (pathPointNumber == pathPlanning.pointsCount - 1) {
         if (pathPlanning.routesCreated < 300) {
            if (!hasCargo){
               //setStop(false);
               isWaitingNewTask = true;
               pathPointNumber = -1;
            }
         } else {
            setStop(true);
            //++loadedRobots;
         }
      }

//            if (loadedRobots == m_cFootbots.size()){
//               std::chrono::microseconds elapsed = micros() - start;
//               std::cout << std::to_string(elapsed.count()) << std::endl;
//               std::cout << "Absolute time, mcs: " << start.count() << std::endl;
//               std::cout << "Absolute time, mcs: " << micros().count() << std::endl;
//               std::cout << "Absolute time, mcs: " << (micros().count() - start.count()) << std::endl;
//               std::cout << "Sim steps: " << GetSpace().GetSimulationClock() << std::endl;
//            }

      pathPointNumber++;
   }

}

void WmsController::reset() {

   pathPointNumber = 0;
   hasCargo = true;

}

void WmsController::SetWheelSpeedsFromLocalVector(const CVector2& c_heading) {
   pidControl(c_heading);
}

void WmsController::pidControl(const CVector2& c_heading){

   if (stop){
       // std::cout << "Set Vel: " << "0.0f" << " " << "0.0f" << std::endl;
       m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
       return;
   }

   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize() - CRadians::PI_OVER_TWO;

   double inc = pid.calculate(0, static_cast<double>(cHeadingAngle.GetValue()));

   Real fLeftWheelSpeed, fRightWheelSpeed;

   /* Apply the calculated speeds to the appropriate wheels */
   if(cHeadingAngle > CRadians::ZERO) {
      inc = -inc;
   }
   else {
      inc = inc;
   }

   if (cHeadingAngle.GetAbsoluteValue() < 0.1) {
      fLeftWheelSpeed  = m_sWheelTurningParams.MaxSpeed * 1;
      fRightWheelSpeed = m_sWheelTurningParams.MaxSpeed * 1 - inc;
   } else {
      fLeftWheelSpeed  = 0;
      fRightWheelSpeed = -inc;
   }

   // std::cout << "Set Vel: " << fLeftWheelSpeed << " " << fRightWheelSpeed << std::endl;
   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

REGISTER_CONTROLLER(WmsController, "wms_controller")
