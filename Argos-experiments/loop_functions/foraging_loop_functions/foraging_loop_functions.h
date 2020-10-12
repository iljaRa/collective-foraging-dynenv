#ifndef FORAGING_LOOP_FUNCTIONS_H
#define FORAGING_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/simulator/entities/rab_equipped_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <list>
#include <map>

using namespace argos;

class CForagingLoopFunctions : public CLoopFunctions {

public:

   CForagingLoopFunctions();
   virtual ~CForagingLoopFunctions() {}

   /* ---------------------- */
   /* ---------------------- */
   /* The list of functions. */
   /* ---------------------- */
   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();
   virtual CColor GetFloorColor(const CVector2& c_position_on_plane);
   virtual void PreStep();
   void CommViaSFN();
   void DF_ConComp(std::vector<int>& conCompVect, int i);
   void ChangeItemNumber();
   void OpenOutputFiles();
   void CloseOutputFiles();
   void CommViaProxNetw();
   void UpdateProxNetw();
   
   /* ------------ */
   /* ------------ */
   /* Output files */
   /* ------------ */
   std::ofstream os_dNdt;
   std::ofstream os_networks;
   std::ofstream os_degDist;

   std::string str_folderName;
   std::string s_randID;
   
   /* ------------------------------------------------------------- */
   /* ------------------------------------------------------------- */
   /* Variables used for the identification of connected components */
   /* ------------------------------------------------------------- */

   std::vector<std::vector<int>> MProx;
   std::vector<bool> conComp;
   std::vector<std::vector<int>> conCompGlob;
   std::vector<int> conCompNum;
   int numConComp;
   
   /* --------------------------------------------------------------- */
   /* --------------------------------------------------------------- */
   /* Variables used for the communication via the scale-free network */
   /* --------------------------------------------------------------- */

   std::vector<CCI_RangeAndBearingSensor::TReadings> tPacketsArray;
   std::vector<int> lastExplResults;
   std::vector<bool> isInNest;
   std::vector<bool> isExploring;
   
   /* ---------------------------------------------------------------------- */
   /* ---------------------------------------------------------------------- */
   /* This method is used to visualize the links. -------------------------- */
   /* It is used by the qt_user_functions. Removing it will return an error. */
   typedef std::map<CFootBotEntity*, std::vector<CVector3> > TNeighborsMap;
   TNeighborsMap m_tNeighbors;
   
   inline const TNeighborsMap& GetNeighborPositions() const {
      return m_tNeighbors;
   }

   /* ---------------------------------------- */
   /* ---------------------------------------- */
   /* Variables related to the arena geometry. */
   /* ---------------------------------------- */
   Real m_fFoodSquareRadius;
   CRange<Real> m_cForagingArenaSideX, m_cForagingArenaSideY;
   std::vector<CVector2> m_cFoodPos;
   CFloorEntity* m_pcFloor;

   double si_nestBoundMinX;
   double si_nestBoundMaxX;
   double si_nestBoundMinY;
   double si_nestBoundMaxY;
   
   /* ------------------------------------------------------------ */
   /* ------------------------------------------------------------ */
   /* Variables related to the change in the number of food items. */
   /* ------------------------------------------------------------ */
   int timeThreshold;
   int newTimeThreshold;
   int itemsNum;
   
   /* -------------- */
   /* -------------- */
   /* Miscellaneous. */
   /* -------------- */
   CRandom::CRNG* m_pcRNG;
   double d_totFB;
   
   UInt64 time;
   bool bNest;
   
   bool isScaleFreeCommunication;
   bool isSingleStimulus;
   
   bool isDegDist;
   int timeInstDegDist;

};

#endif
