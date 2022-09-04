#ifndef WMS_CONTROLLER_H
#define WMS_CONTROLLER_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/quaternion.h>

using namespace argos;

class WmsController : public CCI_Controller {

public:

   uint8_t pathPointNumber;

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

public:


   WmsController();

   virtual ~WmsController() {}


   virtual void Init(TConfigurationNode& t_node);

   void setCoordinates(CVector2& cPos, CQuaternion& cOrient, CVector2& cGoalPos);


   virtual void ControlStep();
   virtual void reset();

   virtual void Destroy() {}

   bool getCargoData() {
      return hasCargo;
   }
   void setCargoData(bool val){
       hasCargo = val;
   }

private:

   CVector2 speedVector;
   void SetWheelSpeedsFromLocalVector(const CVector2& c_heading);

private:

   CCI_DifferentialSteeringActuator* m_pcWheels;
   CRandom::CRNG* m_pcRNG;
   SWheelTurningParams m_sWheelTurningParams;
   bool hasCargo;

};

#endif
