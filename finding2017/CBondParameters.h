#if !defined(cbondparameters_h)
#define cbondparameters_h

class CBondParameters
{
    public:
        CBondParameters();

        CBondParameters(double nNormalStiffness,
                        double nTangStiffness,
                        double nNormalStrength,
                        double nShearStrength,
                        double nBondDiskRadius);

        double      m_nNormalStiffness;
        double      m_nTangStiffness;
        double      m_nNormalStrength;
        double      m_nShearStrength;
        double      m_nBondDiskRadius;
};

#endif