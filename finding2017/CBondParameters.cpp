#include "CBondParameters.h"


CBondParameters::CBondParameters() :
                 m_nNormalStiffness(),
                 m_nTangStiffness(),
                 m_nNormalStrength(),
                 m_nShearStrength(),
                 m_nBondDiskRadius()
{
    ;
}

CBondParameters::CBondParameters(double nNormalStiffness,
                                 double nTangStiffness,
                                 double nNormalStrength,
                                 double nShearStrength,
                                 double nBondDiskRadius) :
                 m_nNormalStiffness(nNormalStiffness),
                 m_nTangStiffness(nTangStiffness),
                 m_nNormalStrength(nNormalStrength),
                 m_nShearStrength(nShearStrength),
                 m_nBondDiskRadius(nBondDiskRadius)
{
    ;
}
