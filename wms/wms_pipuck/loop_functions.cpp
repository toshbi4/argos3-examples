#include "loop_functions.h"
#include <argos3/plugins/robots/pi-puck/simulator/pipuck_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

static const std::string PP_CONTROLLER  = "test_controller";

    bool CTestLoopFunctions::IsExperimentFinished() {

        if ((GetSpace().GetSimulationClock() > 0) && (start == mcs(0))){
            start = micros();
        }

        CSpace::TMapPerType& m_cPiPuck = GetSpace().GetEntitiesByType("pipuck");

        uint8_t ik = 0;
        for(CSpace::TMapPerType::iterator it = m_cPiPuck.begin();
            it != m_cPiPuck.end();
            ++it) {
           /* Get handle to pipuck entity and controller */
           CPiPuckEntity& cPiPuck = *any_cast<CPiPuckEntity*>(it->second);
           const CVector3& cPiPuckPosition = cPiPuck.GetEmbodiedEntity().GetOriginAnchor().Position;
           std::cout << std::to_string(cPiPuckPosition[0]) << std::endl;
           std::cout << cPiPuck.GetId() << std::endl;

           std::cout << std::to_string(ik++) << std::endl;

           if (cPiPuckPosition[0] < 0){
               std::chrono::microseconds elapsed = micros() - start;
               std::cout << std::to_string(elapsed.count()) << std::endl;
               return true;
           } else {
               return false;
           }

        }
    }

    void CTestLoopFunctions::Init(TConfigurationNode& t_tree) {

        PlaceLine(CVector2(8.5, 6), 50, 0.2, 0);
    }

    void CTestLoopFunctions::PlaceLine(const CVector2& c_start,
                                             UInt32 un_robots,
                                             Real f_distance,
                                             UInt32 un_id_start) {
        try {
           CPiPuckEntity* pcPP;
           std::ostringstream cPPId;
           /* For each robot */
           for(size_t i = 0; i < un_robots; ++i) {
              /* Make the id */
              cPPId.str("");
              cPPId << "pp" << (i + un_id_start);
              /* Create the robot in the origin and add it to ARGoS space */
              pcPP = new CPiPuckEntity(
                 cPPId.str(),
                 PP_CONTROLLER,
                 CVector3(c_start.GetX(),-f_distance*i + c_start.GetY(), 0),
                 CQuaternion(0, 0, 0, 1));

              AddEntity(*pcPP);
           }
        }
        catch(CARGoSException& ex) {
           THROW_ARGOSEXCEPTION_NESTED("While placing robots in a line", ex);
        }
    }

   /****************************************/
   /****************************************/

   const CVector3 CTestLoopFunctions::TARGET_POSITION = CVector3(0.5, 0, 0);
   const Real CTestLoopFunctions::THRESHOLD = 0.01;

   /****************************************/
   /****************************************/

   REGISTER_LOOP_FUNCTIONS(CTestLoopFunctions, "test_loop_functions");

}
