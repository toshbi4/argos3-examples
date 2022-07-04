#include "loop_functions.h"
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

    bool CTestLoopFunctions::IsExperimentFinished() {

        CFootBotEntity* pcFootBot = nullptr;

        if ((GetSpace().GetSimulationClock() > 0) && (start == mcs(0))){
            start = micros();
        }

        try {
           CEntity& cEntity = GetSpace().GetEntity("fb");
           pcFootBot = dynamic_cast<CFootBotEntity*>(&cEntity);
        }
        catch(CARGoSException& ex) {
           THROW_ARGOSEXCEPTION_NESTED("Robot was not added to the simulator", ex);
        }
        if(pcFootBot == nullptr) {
           THROW_ARGOSEXCEPTION("Robot was not a Foot-Bot");
        }

        const CVector3& cFootBotPosition = pcFootBot->GetEmbodiedEntity().GetOriginAnchor().Position;

        std::cout << std::to_string(cFootBotPosition[0]) << std::endl;
//        LOG << "[INFO] The physics engine \""
//            << 'GetId()'
//            << "\" will perform "
//            << '2'
//            << " iterations per tick (dt = "
//            << '3' << " sec)"
//            << std::endl;
        if (cFootBotPosition[0] > 9){
            std::chrono::microseconds elapsed = micros() - start;
            std::cout << std::to_string(elapsed.count()) << std::endl;
            return true;
        } else {
            return false;
        }

//    if(Distance(cFootBotPosition, TARGET_POSITION) > THRESHOLD) {
//       THROW_ARGOSEXCEPTION("Robot did not drive forwards to the target position");
//    }
    }

   /****************************************/
   /****************************************/

   const CVector3 CTestLoopFunctions::TARGET_POSITION = CVector3(0.5, 0, 0);
   const Real CTestLoopFunctions::THRESHOLD = 0.01;

   /****************************************/
   /****************************************/

   REGISTER_LOOP_FUNCTIONS(CTestLoopFunctions, "test_loop_functions");

}
