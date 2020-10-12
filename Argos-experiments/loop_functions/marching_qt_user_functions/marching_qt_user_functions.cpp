#include "marching_qt_user_functions.h"
#include "loop_functions/foraging_loop_functions/foraging_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <controllers/footbot_foraging/footbot_foraging.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/utility/math/ray3.h>

using namespace argos;

/****************************************/
/****************************************/

CMarchingQTUserFunctions::CMarchingQTUserFunctions() :
   m_cMarchLF(dynamic_cast<CForagingLoopFunctions&>(CSimulator::GetInstance().GetLoopFunctions())) {
   RegisterUserFunction<CMarchingQTUserFunctions,CFootBotEntity>(&CMarchingQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CMarchingQTUserFunctions::Draw(CFootBotEntity& c_entity) {
   CFootBotForaging& cController = dynamic_cast<CFootBotForaging&>(c_entity.GetControllableEntity().GetController());
    std:: string strID = c_entity.GetId().substr (2,5);
	int unID = std::stoi (strID,nullptr,10);
	/* This one is helpful for drawing the ID above the robot */
	/* Alternatively, one could also use a modified DrawInfo function below */
	//~ if(unID == 200){	
	   //~ DrawText(CVector3(0.0, 0.0, 0.3),   // position
				//~ c_entity.GetId().substr (2,5)); // text
	//~ }
	
	//~ std::vector<std::string> info;
	//~ info.push_back(std::to_string(cController.GetDegree()));
	
	//~ DrawInfo(c_entity, info);
}

/****************************************/
/****************************************/

void CMarchingQTUserFunctions::DrawInWorld() {
   /* Go through all the robot neighbors and draw links to them */
   for(CForagingLoopFunctions::TNeighborsMap::const_iterator it = m_cMarchLF.GetNeighborPositions().begin();
       it != m_cMarchLF.GetNeighborPositions().end();
       ++it) {
      DrawLinks(it->second);
   }
}

/****************************************/
/****************************************/

void CMarchingQTUserFunctions::DrawLinks(const std::vector<CVector3>& neighborPositions) {
   /* Start drawing if a footbot has at least one neighbor */
   if(neighborPositions.size() > 1) {
      size_t unStart = 0;
      size_t unEnd = 1;
      CVector3 pos0 = neighborPositions[unStart];
      pos0.SetZ(0.3f);
      while(unEnd < neighborPositions.size()) {
		  CVector3 pos1 = neighborPositions[unEnd];
		  pos1.SetZ(0.3f);
         DrawRay(CRay3(pos1,pos0), CColor::CYAN);
         ++unEnd;
      }
   }
}

/****************************************/
/****************************************/

void CMarchingQTUserFunctions::DrawInfo(CFootBotEntity& c_entity, std::vector<std::string>& info) {
   /* The position of the text is expressed wrt the reference point of the footbot
    * For a foot-bot, the reference point is the center of its base.
    * See also the description in
    * $ argos3 -q foot-bot
    */
   if(info.size() > 0) {
      size_t unEnd = 1;
      std::string infoToDraw = info[unEnd-1];
      while(unEnd < info.size()) {
		  infoToDraw = infoToDraw + " " + info[unEnd];
            ++unEnd;
      }
	   //~ DrawText(c_entity.GetEmbodiedEntity().GetOriginAnchor().Position,   // position
	   DrawText(CVector3(0.0, 0.0, 0.3),   // position
				infoToDraw); // text
   }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CMarchingQTUserFunctions, "marching_qt_user_functions")
