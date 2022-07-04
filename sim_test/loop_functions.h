#ifndef TEST_LOOP_FUNCTIONS_H
#define TEST_LOOP_FUNCTIONS_H

namespace argos {
   class CEmbodiedEntity;
}

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <chrono>

namespace argos {

    using mcs = std::chrono::microseconds;

   class CTestLoopFunctions : public CLoopFunctions {

   public:

      CTestLoopFunctions():
          start {mcs(0)}
      {}

      virtual ~CTestLoopFunctions() {}

      virtual bool IsExperimentFinished() override;

   private:

      // Get time stamp in microseconds.
      std::chrono::microseconds micros()
      {
          std::chrono::microseconds us = std::chrono::duration_cast<std::chrono::microseconds>(
                  std::chrono::high_resolution_clock::now().time_since_epoch());
          return us;
      }

      const static CVector3 TARGET_POSITION;
      const static Real THRESHOLD;
      std::chrono::microseconds start;

   };
}

#endif

