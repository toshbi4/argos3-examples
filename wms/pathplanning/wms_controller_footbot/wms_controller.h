#ifndef WMS_CONTROLLER_H
#define WMS_CONTROLLER_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/quaternion.h>
#include <wms/pathplanning/wms_controller_footbot/pathplanning.h>
#include "wms/pathplanning/wms_controller_footbot/pid.h"

using namespace argos;

class WmsController : public CCI_Controller {

public:

   struct SWheelTurningParams {
      /*
       * The turning mechanism.
       * The robot can be in three different turning states.
       */
      enum ETurningMechanism
      {
         NO_TURN = 0, // go straight
         SOFT_TURN,   // both wheels are turning forwards, but at different speeds
         HARD_TURN    // wheels are turning with opposite speeds
      } TurningMechanism;
      /*
       * Angular thresholds to change turning state.
       */
      CRadians HardTurnOnAngleThreshold;
      CRadians SoftTurnOnAngleThreshold;
      CRadians NoTurnAngleThreshold;
      /* Maximum wheel speed */
      Real MaxSpeed;

      void Init(TConfigurationNode& t_tree);
   };

   WmsController();

   virtual ~WmsController() {}

   virtual void Init(TConfigurationNode& t_node);

   void setCoordinates(CVector2& cGoalPos);


   virtual void ControlStep();
   virtual void reset();

   virtual void Destroy() {}

   bool getCargoData() {
      return hasCargo;
   }
   void setCargoData(bool val){
      hasCargo = val;
   }
   void setStop(bool val){
      stop = val;
   }
   void setFreeSpace(std::vector<FreeRectangle> aFreeSpace){
      freeSpace = aFreeSpace;
   }
   void setParameters(Real aRadius, uint8_t aMotionType){
       m_fFoodSquareRadius = aRadius;
       motionType = aMotionType;
   }

   uint8_t pathPointNumber;
   PathPlanning pathPlanning;
   bool stop;
   bool isWaitingNewTask;
   bool changeFloor;
   bool stepInProcess;

private:

   void SetWheelSpeedsFromLocalVector(const CVector2& c_heading);
   void pidControl(const CVector2& c_heading);

   CVector2 speedVector;
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the positioning sensor */
   CCI_PositioningSensor* m_pcPosSens;

   CRandom::CRNG* m_pcRNG;
   SWheelTurningParams m_sWheelTurningParams;
   Real m_fFoodSquareRadius;
   uint8_t motionType; // 0 - perpendicular, 1 - diagonal

   std::vector<FreeRectangle> freeSpace;
   bool hasCargo;
   PID pid;
};

#endif
