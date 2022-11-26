#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>

#include <wms/pathplanning/wms_controller_footbot/wms_controller.h>
#include "wms_loop_functions.h"

/*
 * To reduce the number of waypoints stored in memory,
 * consider two robot positions distinct if they are
 * at least MIN_DISTANCE away from each other
 * This constant is expressed in meters
 */
static const Real MIN_DISTANCE = 0.05f;
/* Convenience constant to avoid calculating the square root in PostStep() */
static const Real MIN_DISTANCE_SQUARED = MIN_DISTANCE * MIN_DISTANCE;

WmsLoopFunctions::WmsLoopFunctions() :
   m_cForagingArenaSideX(4.0f, 8.5f),
   m_cForagingArenaSideY(-6.5f, 6.5f),
   m_pcFloor(NULL),
   borderIdNumber{0},
   loadedRobots{0},
   goalsPredefined{false},
   taskNumber{0}
{
}

WmsLoopFunctions::~WmsLoopFunctions(){
   if (!goalsPredefined){
      std::ofstream out;
      out.open("./goals.txt");
      if (out.is_open())
      {
         for(int i=0; i < loadPoints.size(); i++){
            out << loadPoints[i].GetX() << " " << loadPoints[i].GetY();
            out << " " << unloadPoints[i].GetX() << " " << unloadPoints[i].GetY() << std::endl;
         }
      }
   }
}

void WmsLoopFunctions::Init(TConfigurationNode& t_node) {

   try {

      TConfigurationNode& tWorkspace = GetNode(t_node, "workspace");
      GetNodeAttribute(tWorkspace, "radius", m_fFoodSquareRadius);
      m_fFoodSquareRadius *= m_fFoodSquareRadius;

      GetNodeAttribute(tWorkspace, "motionType", motionType);

      CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
      // std::cout << m_cFootbots.size() << std::endl;

      m_pcFloor = &GetSpace().GetFloorEntity();

   }
   catch(CARGoSException& ex) {
     THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
   }

   /* Check file with predefined points */
   std::string line;
   std::ifstream in("./goals.txt"); // open file

   if (in.is_open())
   {
      std::cout << "Goals predefined" << std::endl;
      goalsPredefined = true;
      while (getline(in, line))
      {
          // std::cout << line << std::endl;

          std::istringstream in(line); // make a stream for the line itself

          float x_load, y_load, x_unload, y_unload;
          in >> x_load >> y_load >> x_unload >> y_unload; // now read the whitespace-separated floats

          loadPoints.push_back(CVector2(x_load, y_load));
          unloadPoints.push_back(CVector2(x_unload, y_unload));
          // std::cout << loadPoints[loadPoints.size()-1].GetX() << " " << loadPoints[loadPoints.size()-1].GetY() << std::endl;
      }
   } else {
      std::cout << "Goals are not predefined" << std::endl;
      goalsPredefined = false;
   }
   in.close();

   createScene();

   /* Check whether a robot is on a food item */
   CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
   for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
   it != m_cFootbots.end();
   ++it) {

      /* Get handle to foot-bot entity and controller */
      CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
      WmsController& cController = dynamic_cast<WmsController&>(cFootBot.GetControllableEntity().GetController());

      /* Create a waypoint vector */
      m_tWaypoints[&cFootBot] = std::vector<CVector3>();
      /* Add the initial position of the foot-bot */
      m_tWaypoints[&cFootBot].push_back(CVector3(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                                                 cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY(),
                                                 0.1f));

      cController.setFreeSpace(freeSpace);
      cController.setParameters(m_fFoodSquareRadius, motionType);
   }

   Reset();
}

void WmsLoopFunctions::createScene(){
    // Create obstacles and set a free place
    createBorder(CVector2(-9.1f, -6.0f), CVector2(-9.0f, 6.0f));

    createBorder(CVector2(-9.0f, 6.0f), CVector2(4.0f, 6.1f));
    // TODO: FIX HINT BY OBSTALCE AVOIDANCE
    freeSpace.push_back(FreeRectangle{CVector2(-9.0f, 5.5f + 0.1f), CVector2(4.0f, 6.0f - 0.1f)});
    createBorder(CVector2(-9.0f, 4.5f), CVector2(4.0f, 5.5f));

    freeSpace.push_back(FreeRectangle{CVector2(-9.0f, 4.0f + 0.1f), CVector2(4.0f, 4.5f - 0.1f)});
    createBorder(CVector2(-9.0f, 3.0f), CVector2(4.0f, 4.0f));

    freeSpace.push_back(FreeRectangle{CVector2(-9.0f, 2.5f + 0.1f), CVector2(4.0f, 3.0f - 0.1f)});
    createBorder(CVector2(-9.0f, 1.5f), CVector2(4.0f, 2.5f));

    freeSpace.push_back(FreeRectangle{CVector2(-9.0f, 1.0f + 0.1f), CVector2(4.0f, 1.5f - 0.1f)});
    createBorder(CVector2(-9.0f, -1.0f), CVector2(4.0f, 1.0f));

    freeSpace.push_back(FreeRectangle{CVector2(-9.0f, -1.5f + 0.1f), CVector2(4.0f, -1.0f - 0.1f)});
    createBorder(CVector2(-9.0f, -2.5f), CVector2(4.0f, -1.5f));

    freeSpace.push_back(FreeRectangle{CVector2(-9.0f, -3.0f + 0.1f), CVector2(4.0f, -2.5f - 0.1f)});
    createBorder(CVector2(-9.0f, -4.0f), CVector2(4.0f, -3.0f));

    freeSpace.push_back(FreeRectangle{CVector2(-9.0f, -4.5f + 0.1f), CVector2(4.0f, -4.0f - 0.2f)});
    createBorder(CVector2(-9.0f, -5.5f), CVector2(4.0f, -4.5f));

    freeSpace.push_back(FreeRectangle{CVector2(-9.0f, -6.0f + 0.1f), CVector2(4.0f, -5.5f - 0.1f)});
    createBorder(CVector2(-9.0f, -6.1f), CVector2(4.0f, -6.0f));

    createBorder(CVector2(4.0f, 6.5f), CVector2(9.0f, 6.6f));
    createBorder(CVector2(3.9f, 6.0f), CVector2(4.0f, 6.5f));
    freeSpace.push_back(FreeRectangle{CVector2(4.0f, -6.5f), CVector2(9.0f, 6.5f), 1});
    createBorder(CVector2(3.9f, -6.5f), CVector2(4.0f, -6.0f));
    createBorder(CVector2(4.0f, -6.6f), CVector2(9.0f, -6.5f));
    createBorder(CVector2(9.0f, -6.5f), CVector2(9.1f, 6.5f));
}

void WmsLoopFunctions::createBorder(CVector2 firstCoordinate, CVector2 secondCoordinate){

   CVector2 boxCenter {CVector2((secondCoordinate - firstCoordinate) * 0.5 + firstCoordinate)};
   CVector3 boxSize {secondCoordinate.GetX() - firstCoordinate.GetX(),
                     secondCoordinate.GetY() - firstCoordinate.GetY(),
                     0.1f};

   // Custom desctribution
   try {
      CBoxEntity* pcBox;
      std::ostringstream cBoxId;

         /* Make the id */
         cBoxId.str("");
         cBoxId << "box" << borderIdNumber;
         /* Create the box in the origin and add it to ARGoS space */
         pcBox = new CBoxEntity(
            cBoxId.str(),                                    //str_id
            CVector3(boxCenter.GetX(), boxCenter.GetY(), 0), //center c_position
            CQuaternion(0.0f,0.0f,0.0f,0.0f),                //c_orientation
            false,                                           //bool b_movable
            boxSize);                                        //c_size

         AddEntity(*pcBox);

   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("While placing robots in a line", ex);
   }

   borderIdNumber++;

}

void WmsLoopFunctions::Reset() {

    CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
       it != m_cFootbots.end();
       ++it) {

       CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
       WmsController& cController = dynamic_cast<WmsController&>(cFootBot.GetControllableEntity().GetController());
       if (goalsPredefined){
          cController.pathPlanning.init(freeSpace,
                                        cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position,
                                        cController.getCargoData(),
                                        motionType,
                                        &loadPoints[taskNumber],
                                        &unloadPoints[taskNumber]);
          ++taskNumber;
       } else {
          cController.pathPlanning.init(freeSpace,
                                        cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position,
                                        cController.getCargoData(),
                                        motionType);
       }
       cController.isWaitingNewTask = false;
    }

    // m_pcFloor->SetChanged();

}

void WmsLoopFunctions::Destroy() {

}

CColor WmsLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {

   /* Check whether a robot is on a food item */
   CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

   uint16_t robot_id = 0;
   for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
      it != m_cFootbots.end();
      ++it) {

      /* Get handle to foot-bot entity and controller */
      CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
      WmsController& cController = dynamic_cast<WmsController&>(cFootBot.GetControllableEntity().GetController());

      if((c_position_on_plane - cController.pathPlanning.getGoals()[cController.pathPointNumber].coords).SquareLength() < m_fFoodSquareRadius) {
         return CColor::BLACK;
      }

      robot_id += 1;

   }

//   if((c_position_on_plane - CVector2(-0.2f, 5.9f)).SquareLength() < m_fFoodSquareRadius) {
//      return CColor::BLACK;
//   }

//   if((c_position_on_plane - CVector2(5.8f, -0.1f)).SquareLength() < m_fFoodSquareRadius) {
//      return CColor::BLACK;
//   }

   return CColor::WHITE;
}

void WmsLoopFunctions::PreStep() {

   uint16_t finishedStep = 0;

   CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
   for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
      it != m_cFootbots.end();
      ++it) {

      CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
      WmsController& cController = dynamic_cast<WmsController&>(cFootBot.GetControllableEntity().GetController());
      if (cController.changeFloor){
         // m_pcFloor->SetChanged();
         cController.changeFloor = false;
      }

      if (!cController.stepInProcess) {
         ++finishedStep;
      }

      if (cController.isWaitingNewTask){
         if (goalsPredefined){
            cController.pathPlanning.init(freeSpace,
                                          cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position,
                                          cController.getCargoData(),
                                          motionType,
                                          &loadPoints[taskNumber],
                                          &unloadPoints[taskNumber]);
            ++taskNumber;
         } else {
            cController.pathPlanning.init(freeSpace,
                                          cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position,
                                          cController.getCargoData(),
                                          motionType);
         }
         cController.isWaitingNewTask = false;
      }

      /* Add the current position of the foot-bot if it's sufficiently far from the last */
//      if(SquareDistance(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position,
//                        m_tWaypoints[&cFootBot].back()) > MIN_DISTANCE_SQUARED) {
//         m_tWaypoints[&cFootBot].push_back(CVector3(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
//                                                    cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY(),
//                                                    0.1f));
//      }
   }

   // Discrete movements
   if (finishedStep == m_cFootbots.size()) {
      for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
         it != m_cFootbots.end();
         ++it) {
         CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
         WmsController& cController = dynamic_cast<WmsController&>(cFootBot.GetControllableEntity().GetController());
         cController.stepInProcess = true;
         cController.setStop(false);
      }
   }

}

REGISTER_LOOP_FUNCTIONS(WmsLoopFunctions, "wms_loop_functions")
