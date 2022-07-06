#include "loop_functions.h"
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

static const std::string FB_CONTROLLER    = "test_controller";

    bool CTestLoopFunctions::IsExperimentFinished() {

        if ((GetSpace().GetSimulationClock() > 0) && (start == mcs(0))){
            start = micros();
        }

        CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

        for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
            it != m_cFootbots.end();
            ++it) {
           /* Get handle to foot-bot entity and controller */
           CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
           const CVector3& cFootBotPosition = cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position;
           std::cout << std::to_string(cFootBotPosition[0]) << std::endl;

           if (cFootBotPosition[0] > 9){
               std::chrono::microseconds elapsed = micros() - start;
               std::cout << std::to_string(elapsed.count()) << std::endl;
               return true;
           } else {
               return false;
           }

        }
    }

    void CTestLoopFunctions::Init(TConfigurationNode& t_tree) {

        PlaceLine(CVector2(-9, -9), 500, 0.2, 0);
    }

    void CTestLoopFunctions::PlaceLine(const CVector2& c_start,
                                             UInt32 un_robots,
                                             Real f_distance,
                                             UInt32 un_id_start) {
        try {
           CFootBotEntity* pcFB;
           std::ostringstream cFBId;
           /* For each robot */
           for(size_t i = 0; i < un_robots; ++i) {
              /* Make the id */
              cFBId.str("");
              cFBId << "fb" << (i + un_id_start);
              /* Create the robot in the origin and add it to ARGoS space */
              pcFB = new CFootBotEntity(
                 cFBId.str(),
                 FB_CONTROLLER,
                 CVector3(c_start.GetX(),f_distance*i + c_start.GetY(), 0));
              AddEntity(*pcFB);
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
