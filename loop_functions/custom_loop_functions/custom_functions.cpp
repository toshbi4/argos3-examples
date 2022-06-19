#include "custom_functions.h"

/****************************************/
/****************************************/

CustomFunctions::CustomFunctions() {
    std::cout << "1" << std::endl;
}

void CustomFunctions::Init(TConfigurationNode& t_tree) {
    std::cout << "2" << std::endl;
}

void CustomFunctions::Reset() {
    std::cout << "3" << std::endl;
}

void CustomFunctions::PostStep() {
    std::cout << "4" << std::endl;
}

REGISTER_LOOP_FUNCTIONS(CustomFunctions, "custom_functions");
