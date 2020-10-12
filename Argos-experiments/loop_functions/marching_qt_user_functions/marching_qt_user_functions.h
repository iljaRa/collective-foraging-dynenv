#ifndef MARCHING_QT_USER_FUNCTIONS_H
#define MARCHING_QT_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/loop_functions.h>

using namespace argos;

class CForagingLoopFunctions;

class CMarchingQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CMarchingQTUserFunctions();

   virtual ~CMarchingQTUserFunctions() {}

   virtual void DrawInWorld();

   void Draw(CFootBotEntity& c_entity);
   
private:

   void DrawLinks(const std::vector<CVector3>& neighborPositions);

   void DrawInfo(CFootBotEntity& c_entity, std::vector<std::string>& info);
   
private: 
	
   CForagingLoopFunctions& m_cMarchLF;
};

#endif
