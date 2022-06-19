#ifndef CUSTOM_FUNCTIONS_H
#define CUSTOM_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <myepuck/simulator/myepuck_entity.h>

using namespace argos;

class CustomFunctions : public CLoopFunctions {

public:

   CustomFunctions();

   virtual ~CustomFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);

   virtual void Reset();

   virtual void PostStep();
   
};

#endif
