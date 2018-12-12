#if !defined(cbond_h)
#define cbond_h

#include "Helpers.h"

class CBond
{
    public:
        CBond(int nId1, int nId2);
        CBond();
        virtual ~CBond();

        // getters
        bool             getBroken() const       { return m_bBroken; }
        CSimple3DVector& getForceNormal()        { return m_vFnormal; }
        CSimple3DVector& getForceTangential()    { return m_vFtang; }
        CSimple3DVector& getTorqueNormal()       { return m_vTnormal; }
        CSimple3DVector& getTorqueTangential()   { return m_vTtang; }

        // setters
        void setBroken(const bool bBroken) { m_bBroken = bBroken; }

        bool operator==(const CBond &compBond);

    private:
        int             m_nId1, m_nId2;
        CSimple3DVector m_vFnormal, m_vFtang;
        CSimple3DVector m_vTnormal, m_vTtang;
        bool            m_bBroken;
};

#endif
