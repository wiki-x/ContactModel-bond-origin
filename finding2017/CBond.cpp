#include "CBond.h"

CBond::CBond(int nId1, int nId2) :
       m_nId1(nId1),
       m_nId2(nId2),
       m_bBroken(false)
{
    ;
}

CBond::CBond() :
       m_nId1(0),
       m_nId2(0),
       m_bBroken(false)

{
    ;
}

CBond::~CBond()
{
    ;
}

bool CBond::operator==(const CBond &compBond)
{
    return ( ( (compBond.m_nId1 == m_nId1) && (compBond.m_nId2 == m_nId2) ) ||
           ( (compBond.m_nId1 == m_nId2) && (compBond.m_nId2 == m_nId1) ) );
}