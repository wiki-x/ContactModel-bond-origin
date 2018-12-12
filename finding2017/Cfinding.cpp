#include "Cfinding.h"

#include "Helpers.h"

using namespace std;
using namespace NApi;
using namespace NApiCm;
using namespace NApiCore;

const string Cfinding::PREFS_FILE            = "bonded_particle_prefs.txt";
const string Cfinding::BROKEN_BONDS_FILENAME = "broken_bonds.txt";


Cfinding::Cfinding():
	m_brokenBondsFileName(""),
	m_brokenBondsFile(),
	m_bondMakingHappening(false),
	m_requestedBondTime(0.0),
	m_bondingTime(0.0),
	m_bondingTimestep(0.0)
{
	;
}


Cfinding::~Cfinding()
{
	    if (true == m_brokenBondsFile.is_open())
    {
        m_brokenBondsFile.close();
    }
}

void Cfinding::getPreferenceFileName(char prefFileName[NApi::FILE_PATH_MAX_LENGTH])
{
	// Copy pref file name
    strcpy(prefFileName, PREFS_FILE.c_str());
	//原型声明：extern char *strcpy(char *dest,char *src);
    //头文件：string.h
    //功能：把src所指由NULL结束的字符串复制到dest所指的数组中。
    //说明：src和dest所指内存区域不可以重叠且dest必须有足够的空间来容纳src的字符串。
    //返回指向dest的指针。
}

bool Cfinding::isThreadSafe() 
{
	return true;
}

bool Cfinding::usesCustomProperties()
{
	return true;
}
bool Cfinding::usesContactFactor()
{
	return false;
}

void Cfinding::setFilePath(const char simFile[])
{

}

bool Cfinding::setup(NApiCore::IApiManager_1_0& apiManager,
                     const char                 prefFile[],
                     char                       customMsg[NApi::ERROR_MSG_MAX_LENGTH])
{
	ifstream prefsFile(prefFile);

    if(!prefsFile)
    {
        return false;
    }
    else
    {
        prefsFile >> m_requestedBondTime;
		//m_requestedBondTime = 1.2;
        double nNormalStiffness,
               nTangStiffness,
               nNormalStrength,
               nShearStrength,
               nBondDiskRadius;
        string str;

        while (prefsFile)
        {
            prefsFile >> str
                      >> nNormalStiffness
                     >> nTangStiffness
                      >> nNormalStrength
                      >> nShearStrength
                     >> nBondDiskRadius;
			//str = "qiu";
			//nNormalStiffness = nTangStiffness = 100000000;
			//nNormalStrength = nShearStrength = 2000000;
			//nBondDiskRadius = 0.0015;
            string::size_type i (str.find (':'));
            string surfA = str.substr (0, i);
            str.erase (0, i + 1);
            string surfB = str;

            m_bondParameters.addBondParameters(surfA,
                                               surfB,
				CBondParameters(nNormalStiffness,
                                                               nTangStiffness,
                                                               nNormalStrength,
                                                               nShearStrength,
                                                               nBondDiskRadius));
        }

        // we want to write the broken bonds file in the same location
        // as the preference file, so extract the path from the pref file location
        string tmpPrefFile = prefFile;
        string::size_type idx = tmpPrefFile.find(PREFS_FILE);
        if(idx == string::npos)
        {
            return false;
        }

        m_brokenBondsFileName = tmpPrefFile.substr(0, idx);
        m_brokenBondsFileName += BROKEN_BONDS_FILENAME;

        // Open the broken bonds file

        m_brokenBondsFile.open(m_brokenBondsFileName.c_str(),
                               ios::out|ios::trunc );

        if(!m_brokenBondsFile)
        {
            return false;
        }
    }

    return true;
}

bool Cfinding::starting(NApiCore::IApiManager_1_0& apiManager, int numThreads)
{
	m_PartManger=static_cast<IParticleManagerApi_1_2*>(apiManager.getApi(eParticleManager,1,2));
	if(0==m_PartManger)
	{
		return false;
	}
	return true;
}

void Cfinding::stopping(NApiCore::IApiManager_1_0& apiManager)
{

}

NApi::ECalculateResult Cfinding::calculateForce(
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
                                           double&      calculatedElem2UnsymAdditionalTorqueZ) 
										   {
 // Some temporary variables
    CSimple3DVector F_bond_n,
                    F_bond_t,
                    F_HM_n,
                    F_HM_t,
                    F_damping_n,
                    F_damping_t,
                    F_total_n,
                    F_total_t,
                    T_bond_1,
                    T_bond_2,
                    T_damping_1,
                    T_damping_2,
                    T_total_1,
                    T_total_2;

    /************************************************************************/
    /* Making / obtaining bonds from records                                */
    /************************************************************************/
    CBond elasticBond;

    bool bBondExists = false;
	
	

    //原来：if( elem1IsSurf && elem2IsSurf) //在V2.5接口中，取消elem1IsSurf，elem1始终 是Surf
	if (elem2IsSurf)
    {
        //initial assignment
        bool bFormBond = false;
		//尝试速度位移清除
		if (!m_bondMakingHappening && time == m_requestedBondTime)
		{
			calculatedNormalForceX = 0.0;
			calculatedNormalForceY = 0.0;
			calculatedNormalForceZ = 0.0;

			calculatedUnsymNormalForceX = 0.0;
			calculatedUnsymNormalForceY = 0.0;
			calculatedUnsymNormalForceZ = 0.0;

			calculatedTangentialForceX = 0.0;
			calculatedTangentialForceY = 0.0;
			calculatedTangentialForceZ = 0.0;

			calculatedUnsymTangentialForceX = 0.0;
			calculatedUnsymTangentialForceY = 0.0;
			calculatedUnsymTangentialForceZ = 0.0;

			tangentialPhysicalOverlapX = 0.0;
			tangentialPhysicalOverlapY = 0.0;
			tangentialPhysicalOverlapZ = 0.0;

			calculatedElem1AdditionalTorqueX = 0.0;
			calculatedElem1AdditionalTorqueY = 0.0;
			calculatedElem1AdditionalTorqueZ = 0.0;

			calculatedElem1UnsymAdditionalTorqueX = 0.0;
			calculatedElem1UnsymAdditionalTorqueY = 0.0;
			calculatedElem1UnsymAdditionalTorqueZ = 0.0;

			calculatedElem2AdditionalTorqueX = 0.0;
			calculatedElem2AdditionalTorqueY = 0.0;
			calculatedElem2AdditionalTorqueZ = 0.0;

			calculatedElem2UnsymAdditionalTorqueX = 0.0;
			calculatedElem2UnsymAdditionalTorqueY = 0.0;
			calculatedElem2UnsymAdditionalTorqueZ = 0.0;
		}

        if( !m_bondMakingHappening && time > m_requestedBondTime)//模拟时间大于需要bond的时间，同时满足bond还没激活（m_bondmakninghappening默认为ture？）
			                                                                                                  //开始激活bond，进行下述操作1210
        {
            m_bondMakingHappening = true;//开始进行bond黏结
            m_bondingTime = time;//黏结时间为现在
            m_bondingTimestep = timestep;//bond的时间步
            bFormBond = true;//
        }
        else if(m_bondMakingHappening && time < ( m_bondingTime + m_bondingTimestep / 2.0) )
        {
            bFormBond = true;
        }

        if(bFormBond)
        {
            CBond thisBond(elem1Id, elem2Id);
            m_bonds.addObject(elem1Id,elem2Id, thisBond);
            //tangentialOverlapX = tangentialOverlapY = tangentialOverlapZ = 0.0;老版本
			//新版本
			tangentialPhysicalOverlapX = tangentialPhysicalOverlapY = tangentialPhysicalOverlapZ = 0.0;
            bBondExists = true;
        }
        else
        {
            if(m_bonds.contains(elem1Id, elem2Id))
            {
                bBondExists = true;
                elasticBond = m_bonds.getObject(elem1Id, elem2Id);
            }
        }
    }

    /************************************************************************/
    /* Predefining useful information on contact                            */
    /************************************************************************/

    /* The unit vector from element 1 to the contact point */
    CSimple3DPoint contactPoint     = CSimple3DPoint(contactPointX, contactPointY, contactPointZ);
    CSimple3DVector unitCPVect      = contactPoint - CSimple3DPoint(elem1PosX, elem1PosY, elem1PosZ) ;
    unitCPVect.normalise();//向量归一化，unitcpvect变成单位向量

    /* Calculate the relative velocities of the elements*/
    CSimple3DVector vel1(elem1VelX, elem1VelY, elem1VelZ);//elem1的速度
    CSimple3DVector vel2(elem2VelX, elem2VelY, elem2VelZ);//elem2的速度
    CSimple3DVector relVel        =  vel1 - vel2;//速度差值
    CSimple3DVector relVel_n      = unitCPVect * unitCPVect.dot( relVel);//速度在接触方向上的投影，既法向相对速度
    CSimple3DVector relVel_t      = relVel - relVel_n;//切向相对速度
    CSimple3DVector angVel1(elem1AngVelX, elem1AngVelY, elem1AngVelZ);//elem1的角速度
    CSimple3DVector angVel2(elem2AngVelX, elem2AngVelY, elem2AngVelZ);//elem2的角速度

    /*Equivalent radii & mass */
    //double nEquivRadius     =  elem1PhysicalCurvature * elem2PhysicalCurvature / (elem1PhysicalCurvature + elem2PhysicalCurvature );老版本,求等效半径
	double nEquivRadius = elem1PhysicalRadius*elem2PhysicalRadius / (elem1PhysicalRadius + elem2PhysicalRadius);
    double nEquivMass       = elem1Mass * elem2Mass / (elem1Mass + elem2Mass);//等效质量

    /*Effective Young's Modulus*/
    double nYoungsMod1      = 2 * (1.0 + elem1Poisson) * elem1ShearMod;
    double nYoungsMod2      = 2 * (1.0 + elem2Poisson) * elem2ShearMod;

    /*Equivalent Young's & shear modulus*/
    double nEquivYoungsMod  =  ( 1 - pow(elem1Poisson, 2) ) / nYoungsMod1
                            + ( 1 - pow(elem2Poisson, 2) ) / nYoungsMod2;
    nEquivYoungsMod         = 1.0 / nEquivYoungsMod;

    double nEquivShearMod   =   ( 2.0 - elem1Poisson ) /  elem1ShearMod
                            + ( 2.0 - elem2Poisson ) /  elem2ShearMod;

    nEquivShearMod          = 1.0 / nEquivShearMod;

    /* Tangential Overlap */
    CSimple3DVector nOverlap_t(tangentialPhysicalOverlapX, tangentialPhysicalOverlapY, tangentialPhysicalOverlapZ);

    /************************************************************************/
    /* Bonded Particle Calculation                                          */
    /************************************************************************/
    if(bBondExists && !elasticBond.getBroken())//bond存在，弹性bond还没断裂
    {
        CBondParameters bondParams = m_bondParameters.getBondParameters(elem1Type, elem2Type);

        double nArea = PI * bondParams.m_nBondDiskRadius * bondParams.m_nBondDiskRadius;
        CSimple3DVector ndOverlap_n = relVel_n * timestep;
        CSimple3DVector ndOverlap_t = relVel_t * timestep;;

        CSimple3DVector dF_n = -ndOverlap_n * bondParams.m_nNormalStiffness * nArea;
        CSimple3DVector dF_t = -ndOverlap_t * bondParams.m_nTangStiffness * nArea;

        double J = 0.5 * PI * pow(bondParams.m_nBondDiskRadius,4);
        double I = J / 2.0;

        CSimple3DVector angle  = (angVel1 -angVel2) * timestep;
        CSimple3DVector normalAngle = unitCPVect * unitCPVect.dot(angle);
        CSimple3DVector tangAngle = angle - normalAngle;

        CSimple3DVector dT_n = - normalAngle * J * bondParams.m_nTangStiffness;
        CSimple3DVector dT_t = - tangAngle * I * bondParams.m_nNormalStiffness;

        F_bond_n = elasticBond.getForceNormal() + dF_n;
        F_bond_t = elasticBond.getForceTangential() + dF_t;

        CSimple3DVector T_n = elasticBond.getTorqueNormal() + dT_n;
        CSimple3DVector T_t = elasticBond.getTorqueTangential() + dT_t;

        /* check for fracture */
        double maxStress = -F_bond_n.length() / nArea + T_t.length() / I * bondParams.m_nBondDiskRadius;
        double maxTorque = F_bond_t.length() / nArea + T_n.length() / J * bondParams.m_nBondDiskRadius;
        m_bonds.removeObject(elem1Id, elem2Id);

        if(fabs(maxStress) <= bondParams.m_nNormalStrength && fabs(maxTorque) <= bondParams.m_nShearStrength)
        {
            T_bond_1 = T_n + T_t;
            T_bond_2 = -T_bond_1;

            elasticBond.getForceNormal() = F_bond_n;
            elasticBond.getForceTangential() = F_bond_t;
            elasticBond.getTorqueNormal() = T_n;
            elasticBond.getTorqueTangential() = T_t;
			m_brokenBondsFile << "id:" << elem1Id << " id2" << elem2Id << " bond not broke " << " time " << time << endl;
        }
        else
        {
            F_bond_n = CSimple3DVector();
            F_bond_t = CSimple3DVector();
            elasticBond.setBroken(true);

            m_brokenBondsFile << "id1: " << elem1Id
                              << " pos: x=" << elem1PosX << ", y=" << elem1PosY << ", z=" << elem1PosZ
                              << " id2: " << elem2Id
                              << " pos: x=" << elem2PosX << ", y=" << elem2PosY << ", z=" << elem2PosZ
                              << " time: " << time << endl;
        }

        m_bonds.addObject(elem1Id, elem2Id, elasticBond);
    }

    /************************************************************************/
    /* Hertz-Mindlin Calculation                                            */
    /************************************************************************/

    //Although the bond might exist, we may not have actual physical overlap.
    //double nPhysicalOverlap = normalOverlap;
	double nPhysicalOverlap = normalPhysicalOverlap;
	//if(elem1IsSurf)
        //nPhysicalOverlap += elem1PhysicalCurvature - elem1ContactCurvature;
    if(elem2IsSurf){
        //nPhysicalOverlap += elem2PhysicalCurvature - elem2ContactCurvature;
		nPhysicalOverlap += elem2PhysicalRadius - elem2ContactRadius;
	}
	double JKR = 10000.00;//test jkr

    if( nPhysicalOverlap > 0)
    {
		
        /*********************************************/
        /* Correcting for physical vs contact radius */
        /*********************************************/

        //Tangential overlap.may also be wrong
        nOverlap_t -= relVel_t * timestep;

        //the normal velocity is unaffected, but the tangential one might be different
        //this approximation assumes the particles are spheres.
        //if(elem1IsSurf)
            //vel1 += angVel1.cross(unitCPVect*(elem1PhysicalCurvature - elem1ContactCurvature));
        if(elem2IsSurf)
            vel2 += angVel2.cross(-unitCPVect * (elem2PhysicalRadius - elem2ContactRadius));

        relVel =  vel1 - vel2;
        relVel_n  = unitCPVect * unitCPVect.dot( relVel);
        relVel_t = relVel - relVel_n;

        nOverlap_t += relVel_t * timestep;

        /*****************************/
        /* Normal Calculation        */
        /*****************************/

        double K_n   = 4.0 / 3.0 * nEquivYoungsMod * sqrt(nEquivRadius);//法向刚度
        F_HM_n  = - unitCPVect * K_n * pow(nPhysicalOverlap, 1.5);//法向力

        /*****************************/
        /* Tangential Calculation    */
        /*****************************/

        double S_t =  8.0 * nEquivShearMod * sqrt(nEquivRadius * nPhysicalOverlap);//切向刚度
        F_HM_t = -nOverlap_t * S_t;//切向力

        /*****************************/
        /* Damping/Friction Calculation*/
        /*****************************/

        double B = 0.0;
        if(coeffRest > 0.0)//恢复系数
        {
            double myLog = log(coeffRest);
            B = - myLog/ sqrt(myLog * myLog + PI * PI);//edem帮助手册中关于hertz模型的β变量
        }

        double S_n = 2.0 * nEquivYoungsMod * sqrt(nEquivRadius * nPhysicalOverlap);
        F_damping_n = unitCPVect * 2 * sqrt(5.0 / 6.0) * B * sqrt(S_n * nEquivMass) *  relVel_n.length();//“there is a damping force````"

        /*Are we in a loading situation?*/
        if(relVel_n.dot( unitCPVect) > 0.0)
        {
            F_damping_n = -F_damping_n;
        }

        if(F_HM_t.length() > F_HM_n.length() * staticFriction)//切向力大于静摩擦力
        {
            F_HM_t = F_HM_t * F_HM_n.length() * staticFriction / F_HM_t.length();
            /* slippage has occurred so the tangential overlap is reduced a bit */
            nOverlap_t = -F_HM_t / S_t;

            /* at this point we get energy loss from the sliding */
            F_damping_t = F_HM_t;
            /* little trick so the formulae works below */
            F_HM_t = CSimple3DVector();
        }
        else
        {
            /* at this point we get energy loss from the damping */
            F_damping_t = - relVel_t * 2 * sqrt(5.0 / 6.0) * B * sqrt(S_t * nEquivMass);
        }

        /*****************************/
        /* Rolling Friction          */
        /*****************************/

        /*Only relevant if actually rolling*/
        if(!isZero(angVel1.lengthSquared()))
        {
            CSimple3DVector torque1 = angVel1;
            torque1.normalise();
            //torque1 *= -F_HM_n.length() * elem1PhysicalCurvature * rollingFriction;老版本
			torque1 *= -F_HM_n.length() * elem1PhysicalRadius * rollingFriction;//为什么和法向力有关？
            T_damping_1 = torque1;
        }

        if(!isZero(angVel2.lengthSquared()))
        {
            CSimple3DVector torque2 = angVel2;
            torque2.normalise();
            //torque2 *= -F_HM_n.length() * elem2PhysicalCurvature * rollingFriction;;老版本
			torque2 *= -F_HM_n.length() * elem2PhysicalRadius * rollingFriction;//为什么和法向力有关？
            T_damping_2 = torque2;
        }
    }

    /****************************************/
    /* Fill in parameters we were passed in */
    /****************************************/
    F_total_n = F_bond_n + F_HM_n + F_damping_n;
    F_total_t = F_bond_t + F_HM_t + F_damping_t;
    T_total_1 = T_damping_1;
    T_total_2 = T_damping_2;

    calculatedNormalForceX = F_total_n.dx();
    calculatedNormalForceY = F_total_n.dy();
    calculatedNormalForceZ = F_total_n.dz();

    calculatedUnsymNormalForceX = F_damping_n.dx();
    calculatedUnsymNormalForceY = F_damping_n.dy();
    calculatedUnsymNormalForceZ = F_damping_n.dz();

    calculatedTangentialForceX = F_total_t.dx();
    calculatedTangentialForceY = F_total_t.dy();
    calculatedTangentialForceZ = F_total_t.dz();

    calculatedUnsymTangentialForceX = F_damping_t.dx();
    calculatedUnsymTangentialForceY = F_damping_t.dy();
    calculatedUnsymTangentialForceZ = F_damping_t.dz();

	tangentialPhysicalOverlapX = nOverlap_t.dx();
	tangentialPhysicalOverlapY = nOverlap_t.dy();
	tangentialPhysicalOverlapZ = nOverlap_t.dz();

    calculatedElem1AdditionalTorqueX = T_total_1.dx();
    calculatedElem1AdditionalTorqueY = T_total_1.dy();
    calculatedElem1AdditionalTorqueZ = T_total_1.dz();

    calculatedElem1UnsymAdditionalTorqueX = T_damping_1.dx();
    calculatedElem1UnsymAdditionalTorqueY = T_damping_1.dy();
    calculatedElem1UnsymAdditionalTorqueZ = T_damping_1.dz();

    calculatedElem2AdditionalTorqueX = T_total_2.dx();
    calculatedElem2AdditionalTorqueY = T_total_2.dy();
    calculatedElem2AdditionalTorqueZ = T_total_2.dz();

    calculatedElem2UnsymAdditionalTorqueX = T_damping_2.dx();
    calculatedElem2UnsymAdditionalTorqueY = T_damping_2.dy();
    calculatedElem2UnsymAdditionalTorqueZ = T_damping_2.dz();

    return eSuccess;
										   }


 void Cfinding::configForTimeStep(NApiCore::ICustomPropertyDataApi_1_0* simPropData,
											   NApiCore::IParticleManagerApi_1_0* particleManager,
											   NApiCore::IGeometryManagerApi_1_0* geometryManager,
											   double time)
										   {

}

 unsigned int Cfinding::getNumberOfRequiredProperties(const NApi::EPluginPropertyCategory category)
 {
	 return 0;
}

 bool Cfinding::getDetailsForProperty(
	 unsigned int                    propertyIndex,
	 NApi::EPluginPropertyCategory   category,
	 char                            name[NApi::CUSTOM_PROP_MAX_NAME_LENGTH],
	 NApi::EPluginPropertyDataTypes& dataType,
	 unsigned int&                   numberOfElements,
	 NApi::EPluginPropertyUnitTypes& unitType,
	 char                            initValBuff[NApi::BUFF_SIZE])
 {
	 return false;
 }


