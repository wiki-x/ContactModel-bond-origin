#pragma once
#ifndef CFINDING_H
#define CFINDING_H

#include "IPluginContactModelV2_5_0.h"
#include "IParticleManagerApi_1_2.h"
#include "CBond.h"
#include "CBondList.h"
#include "CBondParameters.h"
#include "CBondParametersList.h"
#include <string>
#include <fstream>



class Cfinding :public NApiCm::IPluginContactModelV2_5_0
{
public:
	/**
 * Name of the preferences file to load bond information
 * from
 */
	static const std::string PREFS_FILE;

	/**
	 * Name of the temp file to hold broken bond information
	 */
	static const std::string BROKEN_BONDS_FILENAME;


	Cfinding();
	~Cfinding();
	virtual void getPreferenceFileName(char prefFileName[NApi::FILE_PATH_MAX_LENGTH]);
	virtual bool isThreadSafe();
	virtual bool usesCustomProperties();
	virtual bool usesContactFactor();
	virtual void setFilePath(const char simFile[]);
	virtual bool setup(NApiCore::IApiManager_1_0& apiManager,
		const char                 prefFile[],
		char                       customMsg[NApi::ERROR_MSG_MAX_LENGTH]);
	virtual bool starting(NApiCore::IApiManager_1_0& apiManager, int numThreads);
	virtual void stopping(NApiCore::IApiManager_1_0& apiManager);
	virtual NApi::ECalculateResult calculateForce(
		int          threadId,
		double       time,
		double       timestep,
		int          elem1Id,
		const char   elem1Type[],
		double       elem1Mass,
		double       elem1Density,
		double       elem1Volume,
		unsigned int elem1Surfaces,
		double       elem1MoIX,
		double       elem1MoIY,
		double       elem1MoIZ,
		double       elem1ShearMod,
		double       elem1Poisson,
		double       elem1ContactRadius,
		double       elem1PhysicalRadius,
		double       elem1PosX,
		double       elem1PosY,
		double       elem1PosZ,
		double       elem1ComX,
		double       elem1ComY,
		double       elem1ComZ,
		double       elem1ContactPointVelX,
		double       elem1ContactPointVelY,
		double       elem1ContactPointVelZ,
		double       elem1VelX,
		double       elem1VelY,
		double       elem1VelZ,
		double       elem1AngVelX,
		double       elem1AngVelY,
		double       elem1AngVelZ,
		const double elem1Orientation[9],
		NApiCore::ICustomPropertyDataApi_1_0* elem1PropData,
		bool         elem2IsSurf,
		int          elem2Id,
		const char   elem2Type[],
		double       elem2Mass,
		double       elem2Density,
		double       elem2Volume,
		unsigned int elem2Surfaces,
		double       elem2MoIX,
		double       elem2MoIY,
		double       elem2MoIZ,
		double       elem2Area,
		double       elem2ShearMod,
		double       elem2Poisson,
		double       elem2ContactRadius,
		double       elem2PhysicalRadius,
		double       elem2PosX,
		double       elem2PosY,
		double       elem2PosZ,
		double       elem2ComX,
		double       elem2ComY,
		double       elem2ComZ,
		double       elem2ContactPointVelX,
		double       elem2ContactPointVelY,
		double       elem2ContactPointVelZ,
		double       elem2VelX,
		double       elem2VelY,
		double       elem2VelZ,
		double       elem2AngVelX,
		double       elem2AngVelY,
		double       elem2AngVelZ,
		const double elem2Orientation[9],
		NApiCore::ICustomPropertyDataApi_1_0* elem2PropData,
		NApiCore::ICustomPropertyDataApi_1_0* contactPropData,
		NApiCore::ICustomPropertyDataApi_1_0* simulationPropData,
		double       coeffRest,
		double       staticFriction,
		double       rollingFriction,
		double       contactPointX,
		double       contactPointY,
		double       contactPointZ,
		double       normalContactOverlap,
		double       normalPhysicalOverlap,
		double       contactFactor,
		double&      tangentialPhysicalOverlapX,
		double&      tangentialPhysicalOverlapY,
		double&      tangentialPhysicalOverlapZ,
		double&      calculatedNormalForceX,
		double&      calculatedNormalForceY,
		double&      calculatedNormalForceZ,
		double&      calculatedUnsymNormalForceX,
		double&      calculatedUnsymNormalForceY,
		double&      calculatedUnsymNormalForceZ,
		double&      calculatedTangentialForceX,
		double&      calculatedTangentialForceY,
		double&      calculatedTangentialForceZ,
		double&      calculatedUnsymTangentialForceX,
		double&      calculatedUnsymTangentialForceY,
		double&      calculatedUnsymTangentialForceZ,
		double&      calculatedElem1AdditionalTorqueX,
		double&      calculatedElem1AdditionalTorqueY,
		double&      calculatedElem1AdditionalTorqueZ,
		double&      calculatedElem1UnsymAdditionalTorqueX,
		double&      calculatedElem1UnsymAdditionalTorqueY,
		double&      calculatedElem1UnsymAdditionalTorqueZ,
		double&      calculatedElem2AdditionalTorqueX,
		double&      calculatedElem2AdditionalTorqueY,
		double&      calculatedElem2AdditionalTorqueZ,
		double&      calculatedElem2UnsymAdditionalTorqueX,
		double&      calculatedElem2UnsymAdditionalTorqueY,
		double&      calculatedElem2UnsymAdditionalTorqueZ);
	virtual void configForTimeStep(NApiCore::ICustomPropertyDataApi_1_0* simPropData,
		NApiCore::IParticleManagerApi_1_0* particleManager,
		NApiCore::IGeometryManagerApi_1_0* geometryManager,
		double time);
	virtual unsigned int getNumberOfRequiredProperties(const NApi::EPluginPropertyCategory category);

	virtual bool getDetailsForProperty(
		unsigned int                    propertyIndex,
		NApi::EPluginPropertyCategory   category,
		char                            name[NApi::CUSTOM_PROP_MAX_NAME_LENGTH],
		NApi::EPluginPropertyDataTypes& dataType,
		unsigned int&                   numberOfElements,
		NApi::EPluginPropertyUnitTypes& unitType,
		char                            initValBuff[NApi::BUFF_SIZE]);
private:
	//static const std::string BONDING;//尝试显示bond键
	NApiCore::IParticleManagerApi_1_2* m_PartManger;//管理器
	std::ofstream       m_brokenBondsFile;//写的文件，和破坏bond键有关？12.08
	std::string         m_brokenBondsFileName;//写的文件名字，和破坏bond键有关？12.08
	bool                m_bondMakingHappening;//bond键是否生成12.08
	double              m_requestedBondTime;//requestbond键生成时间？12.08
	double              m_bondingTime; // bond键生成时间？12.08
	double              m_bondingTimestep;//时间步？12.08
	CBondList           m_bonds;//列表bond？
	CBondParametersList m_bondParameters;//bond参数
};

#endif