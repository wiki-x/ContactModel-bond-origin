#include "CBondList.h"

using namespace std;

CBondList::CBondList()
{
    ;
}

CBondList::CBondList(const CBondList &object)
{
    if(&object != this)
        *this = object;
}

CBondList::~CBondList()
{
    ;
}

void CBondList::addObject(const int nId1, const int nId2, const CBond &object)
{
    tKey key = hashAlgorithm(nId1, nId2);
    m_bondMap.insert( make_pair(key, object) );
}

bool CBondList::contains(const int nId1, const int nId2) const
{
    tKey key = hashAlgorithm(nId1, nId2);
    return ( m_bondMap.find(key) != m_bondMap.end() );
}

CBond CBondList::getObject(const int nId1, const int nId2)
{
    tKey key = hashAlgorithm(nId1, nId2);
    tBondMap::const_iterator it = m_bondMap.find(key);
    if( it != m_bondMap.end() )
    {
        return it->second;
    }
    else
    {
        CBond object;
        m_bondMap.insert( make_pair(key, object) );
        return object;
    }
}

void CBondList::removeObject(const int nId1, const int nId2)
{
    m_bondMap.erase( hashAlgorithm(nId1, nId2) );
}

CBondList::tKey CBondList::hashAlgorithm(const int nId1, const int nId2) const
{
    if(nId1 > nId2)
    {
        return make_pair(nId1, nId2);
    }
    else
    {
        return make_pair(nId2, nId1);
    }
}