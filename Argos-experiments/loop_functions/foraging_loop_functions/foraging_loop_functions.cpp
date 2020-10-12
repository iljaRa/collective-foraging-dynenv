#include "foraging_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <controllers/footbot_foraging/footbot_foraging.h>
#include <stdio.h>
#include <unistd.h>

/****************************************/
/****************************************/

CForagingLoopFunctions::CForagingLoopFunctions() :
    m_cForagingArenaSideX(-10.7f, 18.7f),
    m_cForagingArenaSideY(-3.7f, 3.7f),
    m_pcFloor(NULL),
    MProx(10,std::vector<int>(10)),
    conComp(10),
    conCompGlob(1,std::vector<int>(1)),
    m_pcRNG(NULL) {
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::Init(TConfigurationNode& t_node) {
    try {
		/* Get the destination directory for the output files */
        TConfigurationNode& tFolder = GetNode(t_node, "output");
        GetNodeAttribute(tFolder, "name", str_folderName);
        
		/* Get information on whether degree distribution should be written to a file */
        TConfigurationNode& tAnalysis = GetNode(t_node, "analysis");
        GetNodeAttribute(tAnalysis, "output_deg_dist", isDegDist);
        GetNodeAttribute(tAnalysis, "time_instance", timeInstDegDist);
        
		/* Get the foraging node from XML */
        TConfigurationNode& tForaging = GetNode(t_node, "foraging");
        
		/* Get from XML whether scale-free network should be created */
        GetNodeAttribute(tForaging, "is_sf", isScaleFreeCommunication);
        
		/* Get from XML whether only a single environmental change should be simulated */
        GetNodeAttribute(tForaging, "is_single_stim", isSingleStimulus);
        
        /* Get a pointer to the floor entity */
        m_pcFloor = &GetSpace().GetFloorEntity();
        
        /* Get the number of food items we want to be scattered from XML */
        UInt32 unFoodItems;
        GetNodeAttribute(tForaging, "items", unFoodItems);
        
        /* Get the radius of food items we want to be scattered from XML */
        GetNodeAttribute(tForaging, "radius", m_fFoodSquareRadius);
        m_fFoodSquareRadius *= m_fFoodSquareRadius;
        
        /* Create a new RNG */
        m_pcRNG = CRandom::CreateRNG("argos");
        
        /* Get the area in which we want the food items to be scattered from XML */
        TConfigurationNode& tFoodArea = GetNode(t_node, "food_area");
        
        double food_min, food_max;
        GetNodeAttribute(tFoodArea, "min_x", food_min);
        GetNodeAttribute(tFoodArea, "max_x", food_max);
        m_cForagingArenaSideX.Set(food_min, food_max);
        GetNodeAttribute(tFoodArea, "min_y", food_min);
        GetNodeAttribute(tFoodArea, "max_y", food_max);
        m_cForagingArenaSideY.Set(food_min, food_max);
        
        /* Distribute uniformly the items in the environment */
        for(UInt32 i = 0; i < unFoodItems; ++i) {
            m_cFoodPos.push_back(
                        CVector2(m_pcRNG->Uniform(m_cForagingArenaSideX),
                                 m_pcRNG->Uniform(m_cForagingArenaSideY)));
        }
        
        /* Get the bounds of the nest */
        TConfigurationNode& tNestBounds = GetNode(t_node, "nest_bounds");
        GetNodeAttribute(tNestBounds, "min_x", si_nestBoundMinX);
        GetNodeAttribute(tNestBounds, "max_x", si_nestBoundMaxX);
        GetNodeAttribute(tNestBounds, "min_y", si_nestBoundMinY);
        GetNodeAttribute(tNestBounds, "max_y", si_nestBoundMaxY);

		/* Open output files and initialize some variables. */
        OpenOutputFiles();
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
    }
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::Reset() {
    /* Distribute uniformly the items in the environment */
    for(UInt32 i = 0; i < m_cFoodPos.size(); ++i) {
        m_cFoodPos[i].Set(m_pcRNG->Uniform(m_cForagingArenaSideX),
                          m_pcRNG->Uniform(m_cForagingArenaSideY));
    }
	m_pcFloor->SetChanged();
    time = 0;
    CloseOutputFiles();
    OpenOutputFiles();
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::Destroy() {
    CloseOutputFiles();
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::CloseOutputFiles() {
	os_networks.close();
	os_dNdt.close();
	if(isDegDist){
		os_degDist.close();
	}
}

/****************************************/
/****************************************/

CColor CForagingLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
    if(c_position_on_plane.GetX() < si_nestBoundMaxX && c_position_on_plane.GetX() > si_nestBoundMinX &&
            c_position_on_plane.GetY() < si_nestBoundMaxY && c_position_on_plane.GetY() > si_nestBoundMinY) {
        return CColor::GRAY50;
    }
    for(UInt32 i = 0; i < m_cFoodPos.size(); ++i) {
        if((c_position_on_plane - m_cFoodPos[i]).SquareLength() < m_fFoodSquareRadius) {
            return CColor::BLACK;
        }
    }
    return CColor::WHITE;
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::DF_ConComp(std::vector<int>& conCompVect, int i) {
	/** --- **/
	/** --- Use the recursive depth-first search to identify connected components. --- **/
	/** --- **/
    conComp[i]=true;
    conCompVect.push_back(i);
    for (int j=0; j<MProx[i].size(); j++) {
		int newNode = MProx[i][j];
        if(!conComp[newNode]){ DF_ConComp(conCompVect, newNode); }
    }
}

/****************************************/
/****************************************/


void CForagingLoopFunctions::CommViaSFN(){
	/* ------------------------------------------------------------------------------------------------------------------- */
	/* -------------------------------------- START SCALE-FREE NETW GENERATION ------------------------------------------- */
	/* ------------------------------------------------------------------------------------------------------------------- */
	/* ------------------------------------------------------------------------------------------------------------------- */
	
	/** --- **/
	/** --- Declare and initialize a couple of things. --- **/
	/** --- **/
	
   std::vector<std::vector<int>> MSFN;
   std::vector<int> network;
   std::vector<int> localArea;
   std::vector<int> degrees;
   std::vector<Real> distances;
   
   /* This vector tells us whether the robot was added to the scale-free network */
   std::vector<bool> addedToNetwork;
   
   std::vector<CVector2> positions;
   std::vector<Real> rabRanges;
   
   /* When we iterate the robot map m_cFootbots below, the robot ids are sorted 
    * in the following order: {1, 10, 11, 12, ..., 949}
    * and not: {0, 1, 2, 3, ... , 949}
    * It is important to keep this in mind when handling vectors and other containers. 
    * Therefore, we first resize the vectors to the size of the swarm (=d_totFB) and then 
    * write data into the vectors by assignment, i.e. 'vector[robotID]=value'
    * instead of the more common 'vector.push_back(value)' approach. */
   degrees.resize(d_totFB);
   addedToNetwork.resize(d_totFB);
   positions.resize(d_totFB);
   rabRanges.resize(d_totFB);
    	
    std::string strID = "";
    int robotID = 0;
    int neighbor;
    conCompGlob.clear();

    CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    
    for(CSpace::TMapPerType::iterator it2 = m_cFootbots.begin();
        it2 != m_cFootbots.end();
        ++it2) {
		CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it2->second);
		CFootBotForaging& cController = dynamic_cast<CFootBotForaging&>(cFootBot.GetControllableEntity().GetController());
		CVector2 cPos;
		cPos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
				 cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
		strID = cController.GetId().substr (2,5);
		robotID = std::stoi (strID,nullptr,10);
		
		CCI_RangeAndBearingSensor* ci_rabSens = cController.GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");      // <========= rab_sensor
		const CCI_RangeAndBearingSensor::TReadings& tPackets = ci_rabSens->GetReadings();            // <========= readings
		/* Get the count of robot's neighbors */
		degrees[robotID] = tPackets.size();
		
		/* This array tells us whether the robot is part of a connected component */
		conComp[robotID]=false; 
	}
	
	/** --- **/
	/** --- Use depth-first-search to obtain a list of all connected components, based on the proximity network of only resting robots --- **/
	/** --- **/

	/* ------------------------------------------------------------------------------------------ */
	/* -------------------------------- Get connected components -------------------------------- */
	/* ------------------------------------------------------------------------------------------ */
	for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
	it != m_cFootbots.end();
	++it)  {
		CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
		CFootBotForaging& cController = dynamic_cast<CFootBotForaging&>(cFootBot.GetControllableEntity().GetController());
		strID = cController.GetId().substr (2,5);
		robotID = std::stoi (strID,nullptr,10);
		
		if(!conComp[robotID]){ 
			std::vector<int> conCompVect;
			DF_ConComp(conCompVect, robotID);  
			conCompGlob.push_back(conCompVect); 
		}
	}
	/* ------------------------------------------------------------------------------------------*/
	/* ----------------------------------------------------------------------------------------- */

	/* ------------------------------------------------------------------------------------------------------------------- */
	/* -------------------------------------- LOCAL AREA SCALE FREE NETWORK ALGORITHM ------------------------------------ */
	/* ------------------------------------------------------------------------------------------------------------------- */
	/* ------------------------------------------------------------------------------------------------------------------- */

	Real prefAttachProb = 1.0;
	Real sinkNetwRadius = 0.0;
	CRange<Real> probRange;
	probRange.Set(0.0f,1.0f);
	CRange<UInt32> probRangeUni;
	
	/** --- **/
	/** --- Get information about every robot, such as its last exploration result, its position, the range of its range-and-bearing, etc. **/
	/** --- **/

	for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
		it != m_cFootbots.end();
		++it) {
			CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
			CFootBotForaging& cController = dynamic_cast<CFootBotForaging&>(cFootBot.GetControllableEntity().GetController());

			CCI_RangeAndBearingSensor* ci_rabSens = cController.GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");      // <========= rab_sensor
			const CCI_RangeAndBearingSensor::TReadings& tPackets = ci_rabSens->GetReadings();            // <========= readings
			strID = cController.GetId().substr (2,5);
			robotID = std::stoi (strID,nullptr,10);
			
			/* Get the position of the robot */
			CVector2 cPos;
			cPos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
					 cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
			positions[robotID]=cPos;
			
			/* This vector tells us whether the robot was added to the scale-free network */
			addedToNetwork[robotID]=false;
			
			/* Get the range of the range-and-bearing sensor/actuator */
			CRABEquippedEntity& cFootBotRAB = cFootBot.GetRABEquippedEntity();
			rabRanges[robotID]=cFootBotRAB.GetRange()*1.0;
		
			/* These lines are used for visualizing the communication links */
			CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
			m_tNeighbors[pcFB].clear();
	}
	
	MSFN.resize(d_totFB);
	Real distance;
	double networkDegree = 0.0;
	int sink = 0;
	int newNode = 0, netNode = 0;
	int highestDegree = 0;
	
	/** --- **/
	/** --- Run the scale-free network construction for each connected component --- **/
	/** --- **/

	for(int cC = 0; cC < conCompGlob.size(); cC++){

		/* ------------------------------------------------------------------------------------------ */
		/* ----------------------------------------- STEP ONE --------------------------------------- */
		/* ------------------------------------- CHOOSE THE SINK ------------------------------------ */
		/* ------------------------------------------------------------------------------------------ */
		prefAttachProb = 1.0;
		network.clear();
		distances.clear();
		networkDegree = 0.0;
		sink=conCompGlob[cC][0];
		newNode = 0;
		netNode = 0;
		highestDegree = 0;

		/* --- Choose the node with the largest neighborhood as a sink --- */
		for(int i = 0; i < conCompGlob[cC].size(); i++){
			newNode = conCompGlob[cC][i];
			
			if(highestDegree < degrees[newNode]){ highestDegree = degrees[newNode]; sink = newNode; }
		}

		addedToNetwork[sink] = true;
		network.push_back(sink);
		distances.push_back(0);
		sinkNetwRadius = rabRanges[sink];
		
		/* --- Connect the sink to robots within the sink's range-and-bearing radius --- */
		for(int i = 0; i < conCompGlob[cC].size(); i++){
			newNode = conCompGlob[cC][i];
			distance = (positions[newNode]-positions[sink]).SquareLength();
			distance = sqrt(distance);
			if(distance <= sinkNetwRadius && !addedToNetwork[newNode]){

				network.push_back(newNode);
				networkDegree += 2.0;

				addedToNetwork[newNode] = true;
				
				/* --- Create a bi-diractional link between sink and newNode --- */
				MSFN[sink].push_back(newNode);
				MSFN[newNode].push_back(sink);

			}
		}

		/* Keep adding nodes to the network 
		 * - until the network has the size of the connected component 
		 * - or until the sink radius is longer than the nest diameter
		 * - unless there is only one node in the connected component
		 * */
		while(network.size() < conCompGlob[cC].size() && sinkNetwRadius < 70.7 && conCompGlob[cC].size() > 1){
			/* ------------------------------------------------------------------------------------------ */
			/* ------------------------------------------------------------------------------------------ */
			/* ----------------------------------------- STEP TWO --------------------------------------- */
			/* ------------------------------------- DEFINE LOCAL AREA ---------------------------------- */
			/* ------------------------------------------------------------------------------------------ */
			sinkNetwRadius += 0.2;
			localArea.clear();
			for(int i = 0; i < conCompGlob[cC].size(); i++){
				newNode = conCompGlob[cC][i];
				distance = (positions[newNode]-positions[sink]).SquareLength();
				distance = sqrt(distance);
				if(distance < sinkNetwRadius && !addedToNetwork[newNode]){
					localArea.push_back(newNode);
				}
			}
			
			/* ------------------------------------------------------------------------------------------ */
			/* ------------------------------------------------------------------------------------------ */
			/* ---------------------------------------- STEP THREE -------------------------------------- */
			/* ------------------------------------- CONNECT NEW NODES ---------------------------------- */
			/* ------------------------------------------------------------------------------------------ */
			for(int i = 0; i < localArea.size(); i++){
				newNode = localArea[i];
				probRangeUni.Set(0,network.size());
				prefAttachProb = 0;
				Real probAdd = m_pcRNG->Uniform(probRange);
				int count_considered = 0;
				while(count_considered < network.size()){
					netNode = network[m_pcRNG->Uniform(probRangeUni)];
					/* -------------- COMPUTE THE PROBABILITY -------------- */
					/* ------------ WITH PREFERENTIAL ATTACHMENT ----------- */
					prefAttachProb += (1.0*MSFN[netNode].size()) / (1.0*networkDegree);
					/* ----------------------------------------------------- */
					if(probAdd < prefAttachProb && netNode != newNode){
						if(!addedToNetwork[newNode]){
							network.push_back(newNode);
							addedToNetwork[newNode] = true;
						}
						networkDegree += 2.0;
						MSFN[newNode].push_back(netNode);
						MSFN[netNode].push_back(newNode);
						prefAttachProb = 0;
						probAdd = m_pcRNG->Uniform(probRange);
					}
					count_considered++;
				}
			}
			/* -------------------------------------------------------------------------------------------*/
			/* ------------------------------------------------------------------------------------------ */
		}
	}

	/** --- **/
	/** --- Free up some memory. --- **/
	/** --- **/
	
	network.clear();
	localArea.clear();
	degrees.clear();
	distances.clear();
	addedToNetwork.clear();
	positions.clear();
	rabRanges.clear();
    conCompGlob.clear();
	
	/** --- **/
	/** --- The following loop is used to send information through the above generated links of the scale-free network. --- **/
	/** --- In case of proximity networks, a standard implementation is used 
	 * where the information is shared via range-and-bearing sensor and actuator with robots that are within the comm. radius. ---**/
	/** --- However, as the scale-free network links are synthetically generated, we need to manuanly set what information the robot will receive. --- **/
	/** --- **/
	
	for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
		it != m_cFootbots.end();
		++it) {
		CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
		CFootBotForaging& cController = dynamic_cast<CFootBotForaging&>(cFootBot.GetControllableEntity().GetController());
		
		/* Only consider robots that are inside the nest are inside the nest and are not exploring */
		if(!cController.IsInNest() || cController.IsExploring()){ continue; }
		
		strID = cController.GetId().substr (2,5);
		robotID = std::stoi (strID,nullptr,10);	

		/* These lines are used for visualizing the communication links */
		CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
		m_tNeighbors[pcFB].push_back(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position);
		/* ------------------------------------------------------------ */
			
		/* Write the degree distribution to a file */
		if(isDegDist && time == timeInstDegDist){
			os_degDist << MSFN[robotID].size() << std::endl;
		}

		/* ------------------------------------------------------------------ */
		/* ---------------------- SEND THE INFORMATION ---------------------- */
		/* ------------------------------------------------------------------ */
		
		std::vector<UInt8> newNeighbInfo;
		
		for(size_t j = 0; j < MSFN[robotID].size(); j++){
			/* --- Get the ID of the node --- */
			newNode = MSFN[robotID][j];
			
			/* --- This eventually outputs an edge list per predefined time step --- */
			if(time == 5000 || time == 8750 || time == 8750+2500 || time == 8750+5000 ||
			   time == 8750+7500 || time == 8750+10000 || time == 8750+12500 || time == 8750+15000 || 
			   time == 8750+17500){
				   os_networks << time << "\t" << robotID << "\t" << newNode << std::endl;
			}

			/* --- Save the node ID and lastExplResult to the newSPacket --- */	
			int infoOfNewNode = lastExplResults[newNode];		
			newNeighbInfo.push_back(infoOfNewNode);
			
			/* These lines are used for visualizing the communication links */
			std::string fbID = "fb" + std::to_string(newNode); 
			CFootBotEntity& cFootBot2 = *any_cast<CFootBotEntity*>(m_cFootbots[fbID]);
			m_tNeighbors[pcFB].push_back(cFootBot2.GetEmbodiedEntity().GetOriginAnchor().Position);
		}
		/* Send the new information stored in newNeighbInfo to the robot controller */
		cController.ListenToNeighbs(newNeighbInfo);
		/* -------------------------------------------------------------- */
		/* -------------------------------------------------------------- */
	}
	/* ------------------------------------------------------------------------------------------------------------------- */
	/* ------------------------------------------------------------------------------------------------------------------- */
	
	/* ------------------------------------------------------------------------------------------------------------------- */
	/* -------------------------------------- END SCALE-FREE NETW GENERATION --------------------------------------------- */
	/* ------------------------------------------------------------------------------------------------------------------- */
	/* ------------------------------------------------------------------------------------------------------------------- */
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::CommViaProxNetw() {
    std::string strID = "";
    int robotID = 0;
    int newNode;
    CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    
	for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
        it != m_cFootbots.end();
        ++it) {
        CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
        CFootBotForaging& cController = dynamic_cast<CFootBotForaging&>(cFootBot.GetControllableEntity().GetController());
        strID = cController.GetId().substr (2,5);
        robotID = std::stoi (strID,nullptr,10);
		
        MProx[robotID].clear();
		
		/* Only consider robots that are inside the nest and are not exploring */
		if(!cController.IsInNest() || cController.IsExploring()){ continue; }
		
        CCI_RangeAndBearingSensor* ci_rabSens = cController.GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");      // <========= rab_sensor
        const CCI_RangeAndBearingSensor::TReadings& tPackets = ci_rabSens->GetReadings();
        
		/* These lines are used for visualizing the communication links */
		CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
		m_tNeighbors[pcFB].push_back(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position);
		/* ------------------------------------------------------------ */

		/* ------------------------------------------------------------------ */
		/* ---------------------- SEND THE INFORMATION ---------------------- */
		/* ------------------------------------------------------------------ */
		
		std::vector<UInt8> newNeighbInfo;
		
		for(size_t i = 0; i < tPackets.size(); i++){
			
			newNode = (int) tPackets[i].Data[1]*255 + tPackets[i].Data[2];
				
			if(isInNest[newNode] && !isExploring[newNode]){
			/* Only consider neighbors that are inside the nest */
			/* This prevents communication across the nest border */
				MProx[robotID].push_back(newNode);
				
				/* --- This outputs an edge list per predefined time step --- */
				if(time == 5000 || time == 8750 || time == 8750+2500 || time == 8750+5000 ||
				   time == 8750+7500 || time == 8750+10000 || time == 8750+12500 || time == 8750+15000 || 
				   time == 8750+17500){
					   os_networks << time << "\t" << robotID << "\t" << newNode << std::endl;
				}

				/* --- Save the node ID and lastExplResult to the newSPacket --- */
				newNeighbInfo.push_back(lastExplResults[newNode]);
				
				/* These lines are used for visualizing the communication links */
				std::string fbID = "fb" + std::to_string(newNode); 
				CFootBotEntity& cFootBot2 = *any_cast<CFootBotEntity*>(m_cFootbots[fbID]);
				m_tNeighbors[pcFB].push_back(cFootBot2.GetEmbodiedEntity().GetOriginAnchor().Position);
			}
		}
		/* Send the new information stored in newNeighbInfo to the robot controller */
		cController.ListenToNeighbs(newNeighbInfo);
		/* -------------------------------------------------------------- */
		/* -------------------------------------------------------------- */
			
		/* Write the degree distribution to a file */
		if(isDegDist && time == timeInstDegDist){
			os_degDist << MProx[robotID].size() << std::endl;
		}
	}
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::UpdateProxNetw() {
    std::string strID = "";
    int robotID = 0;
    int newNode;
    CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    
	for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
        it != m_cFootbots.end();
        ++it) {
        CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
        CFootBotForaging& cController = dynamic_cast<CFootBotForaging&>(cFootBot.GetControllableEntity().GetController());
        strID = cController.GetId().substr (2,5);
        robotID = std::stoi (strID,nullptr,10);
		
        MProx[robotID].clear();
		
		/* Only consider robots that are inside the nest are inside the nest and are not exploring */
		if(!cController.IsInNest() || cController.IsExploring()){ continue; }
		
        CCI_RangeAndBearingSensor* ci_rabSens = cController.GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");      // <========= rab_sensor
        const CCI_RangeAndBearingSensor::TReadings& tPackets = ci_rabSens->GetReadings();
        
		for(size_t i = 0; i < tPackets.size(); i++){
			
			newNode = (int) tPackets[i].Data[1]*255 + tPackets[i].Data[2];
			
			if(isInNest[newNode] && !isExploring[newNode]){
			/* Only consider neighbors that are inside the nest */
			/* This prevents communication across the nest border */
				MProx[robotID].push_back(newNode);
			}
		}
	}
}


/****************************************/
/****************************************/


void CForagingLoopFunctions::PreStep() {
	std::string strID = "";
    int robotID = 0;
    int neighbor;
    CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    
    isInNest.clear();
    isExploring.clear();
    lastExplResults.clear();
    
    /* When we iterate the robot map m_cFootbots below, the robot ids are sorted 
    * in the following order: {1, 10, 11, 12, ..., 949}
    * and not: {0, 1, 2, 3, ... , 949}
    * It is important to keep this in mind when handling vectors and other containers. 
    * Therefore, we first resize the vectors to the size of the swarm (=d_totFB) and then 
    * write data into the vectors by assignment, i.e. 'vector[robotID]=value'
    * instead of the more common 'vector.push_back(value)' approach. */
	isInNest.resize(d_totFB);
	isExploring.resize(d_totFB);
	lastExplResults.resize(d_totFB);
    
	for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
	it != m_cFootbots.end();
	++it) {
		CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
		CFootBotForaging& cController = dynamic_cast<CFootBotForaging&>(cFootBot.GetControllableEntity().GetController());
        strID = cController.GetId().substr (2,5);
        robotID = std::stoi (strID,nullptr,10);
	
		/* Check whether the robot is inside the nest */
		isInNest[robotID]=cController.IsInNest();
	
		/* Check whether the robot is in the exploring state */
		isExploring[robotID]=cController.IsExploring();
		
		/* Get the result of the robot's last exploration attempt */
		lastExplResults[robotID]=cController.GetLastExplResults();
		
		/* These lines are used for visualizing the communication links */
		CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
		m_tNeighbors[pcFB].clear();
	}
	
	
	/** --- **/
	/** --- Handle the communication based on whether 
	 *  --- scale-free networks should be created or not. --- */
	/** --- **/
    if(isScaleFreeCommunication){
		/** --- **/
		/** --- First, the matrix of the proximity network is updated. --- */
		/** --- **/
		UpdateProxNetw();
		/** --- **/
		/** --- Based on this proximity network, 
		 *  --- a scale-free network of spatially distributed agents is created. --- */
		/** --- Furthermore, the communication via 
		 *  --- the created scale-free network is handled. --- */
		/** --- **/
		CommViaSFN();
	}
	else{
		/** --- **/
		/** --- Here, the communication via the proximity network is handled. --- */
		/** --- **/
		CommViaProxNetw();
	}

	/** --- **/
    /** --- This function updates the number of items in the foraging area. --- */
	/** --- **/
    if(time > 5000){ ChangeItemNumber(); }
    
    /** --- **/
    /** --- This for-loop handles the item discovery and retrieval. --- */
	/** --- **/
    int resting = 0, exploring = 0, returning = 0, numCollected = 0;

    for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
        it != m_cFootbots.end();
        ++it) {
        /* Get handle to foot-bot entity and controller */
        CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
        CFootBotForaging& cController = dynamic_cast<CFootBotForaging&>(cFootBot.GetControllableEntity().GetController());
        /* Get the position of the foot-bot on the ground as a CVector2 */
        CCI_RangeAndBearingSensor* ci_rabSens = cController.GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");      // <========= rab_sensor
        const CCI_RangeAndBearingSensor::TReadings& tPackets = ci_rabSens->GetReadings();            // <========= readings
        CVector2 cPos;
        cPos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                 cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
        /* Get food data */
        CFootBotForaging::SFoodData& sFoodData = cController.GetFoodData();

        strID = cController.GetId().substr (2,5);
        robotID = std::stoi (strID,nullptr,10);
        MProx[robotID].clear();
        
        /* Collect information on robot behavior. */
        if(cController.IsResting()){
			 resting++; 
		}    
        else if(cController.IsExploring()){
			 exploring++; 
		}     
        else if(cController.IsReturningToNest()){
			 returning++; 
		}         

        /* The foot-bot has a food item */
        if(sFoodData.HasFoodItem) {
            /* Check whether the foot-bot is in the nest*/
            if(cPos.GetX() < si_nestBoundMaxX && cPos.GetX() > si_nestBoundMinX &&
                  cPos.GetY() < si_nestBoundMaxY && cPos.GetY() > si_nestBoundMinY) { 
                /* If the foot-bot is in the nest, the food item is dropped */
                /* A new item appears at a random location in the foraging area */
                /* If the id of the food item is lower than itemsNum */ 
                /* this allows to control the maximum number of items */
                if(sFoodData.FoodItemIdx < itemsNum){
					
                    //~ m_cFoodPos[sFoodData.FoodItemIdx].Set(m_pcRNG->Uniform(m_cForagingArenaSideX),
                                                          //~ m_pcRNG->Uniform(m_cForagingArenaSideY));
                }
                
                sFoodData.HasFoodItem = false;
                sFoodData.FoodItemIdx = 0;
                ++sFoodData.TotalFoodItems;
                numCollected++;
                
                /* The floor texture must be updated */
                //~ m_pcFloor->SetChanged();
            }
        }
        else {
            /* The foot-bot has no food item */
            /* Check whether the foot-bot is out of the nest*/
            if(cPos.GetX() > si_nestBoundMaxX || cPos.GetX() < si_nestBoundMinX ||
                    cPos.GetY() > si_nestBoundMaxY || cPos.GetY() < si_nestBoundMinY) {
                /* Check whether the foot-bot is on a food item */
                bool bDone = false;
                for(size_t i = 0; i < m_cFoodPos.size() && !bDone; ++i) {
                    if((cPos - m_cFoodPos[i]).SquareLength() < m_fFoodSquareRadius) {
                        /* If so, we move that item out of sight */
                        //~ m_cFoodPos[i].Set(100.0f, 100.f);
                        /* The foot-bot is now carrying an item */
                        sFoodData.HasFoodItem = true;
                        sFoodData.FoodItemIdx = i;
                        /* The floor texture must be updated */
                        //~ m_pcFloor->SetChanged();
                        /* We are done */
                        bDone = true;
                    }
                }
            }
        }        
    }
    
	/** --- **/
    /** --- Finally, save the measured data to a file. --- */
	/** --- **/
	//~ LOG << time << std::endl;
	os_dNdt << time << "\t" << resting << "\t" << exploring << "\t" << returning << "\t" << numCollected << std::endl;

	/* Update time */
    time++;
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::ChangeItemNumber() {
	
	/* ---------------- GET THE NEXT TIME STEP AT WHICH itemsNum CHANGES ---------------- */
	if(isSingleStimulus){
		if(time==timeThreshold){
			itemsNum=300;
		}
	}
	else{
		if(time >= timeThreshold && time < newTimeThreshold && itemsNum == 30){itemsNum = 300; timeThreshold+=2500; newTimeThreshold=timeThreshold+2500;}
		else if(time >= timeThreshold && time < newTimeThreshold && itemsNum == 300){itemsNum = 30; timeThreshold+=2500; newTimeThreshold=timeThreshold+2500;}
	}
	/* ---------------------------------------------------------------------------------- */
		
	/* ---------------------------------- ADD ITEMS ------------------------------------- */
	if(itemsNum == 300 && m_cFoodPos.size() < itemsNum){
		for(UInt32 i = 0; i < itemsNum; ++i) {
			m_cFoodPos.push_back(
						CVector2(m_pcRNG->Uniform(m_cForagingArenaSideX),
								 m_pcRNG->Uniform(m_cForagingArenaSideY)));
			m_pcFloor->SetChanged();
		}
	}
	/* ---------------------------------------------------------------------------------- */

	/* ---------------------------------- REMOVE ITEMS ---------------------------------- */
	if(m_cFoodPos.size() > itemsNum){
		int itemsToMove = m_cFoodPos.size() - itemsNum;
		for(size_t i = 0; i < itemsToMove; ++i) {
				// Remove the item 
				m_cFoodPos.erase(m_cFoodPos.begin() + i);
				// The floor texture must be updated 
				m_pcFloor->SetChanged();
		}

	}
	/* ---------------------------------------------------------------------------------- */
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::OpenOutputFiles() {
    /* This method opens the output files of the foot-bots,
    * one file for each foot bot.
   */

    /* Open the file for the ID logging, erasing its contents */
    int robotID;
    std::string txt = ".txt";
    std::string filename = "";
    std::string strID = "";
    std::string dirID = "";
    dirID.append(str_folderName).append("/");

    filename.append(dirID).append("Nrest").append(txt);
    os_dNdt.open(filename, std::ios_base::trunc | std::ios_base::out);
    filename = "";
    
    os_dNdt << "Time\tN_resting\tN_exploring\tN_returning\tCollected_items" << std::endl;

    /* Resize the output-files vector to the number of footbots */
    CSpace::TMapPerType& m_cFootbotsID = GetSpace().GetEntitiesByType("foot-bot");
    
    d_totFB = m_cFootbotsID.size();
    conComp.resize(d_totFB);
    MProx.resize(d_totFB);
    for(int i = 0; i < d_totFB; i++) { MProx[i].resize(d_totFB); }
    
    filename.append(dirID).append("networks").append(txt);
    os_networks.open(filename, std::ios_base::trunc | std::ios_base::out);
    filename = "";

	/* Initialize a couple of variables */
    time = 0;
	itemsNum = 30;
	timeThreshold = 7500;
	newTimeThreshold = 10000;
	timeInstDegDist=10;
	
	/* If the degree distribution at the particular time step timeInstDegDist should be written to a file */
	if(isDegDist){
		filename.append(dirID).append("degree_distribution").append(txt);
		os_degDist.open(filename, std::ios_base::trunc | std::ios_base::out);
	}

	for(CSpace::TMapPerType::iterator it = m_cFootbotsID.begin();
	it != m_cFootbotsID.end();
	++it) {
		CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
		CFootBotForaging& cController = dynamic_cast<CFootBotForaging&>(cFootBot.GetControllableEntity().GetController());

		strID = cController.GetId().substr (2,5);
		robotID = std::stoi (strID,nullptr,10);

		conComp[robotID]=false;

		cController.SetScaleFreeCommunication(isScaleFreeCommunication);
	}
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CForagingLoopFunctions, "foraging_loop_functions")
