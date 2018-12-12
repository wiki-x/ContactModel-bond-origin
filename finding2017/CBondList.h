#if !defined(cbondlist_h)
#define cbondlist_h

#include <utility>
#include <map>

#include "CBond.h"

class CBondList
{
    public:
        CBondList();
        CBondList(const CBondList &object);
        virtual ~CBondList();

        // Object manipulation
        void    addObject(const int nId1, const int nId2, const CBond &object);
        CBond   getObject(const int nId1, const int nId2);
        bool    contains(const int nId1, const int nId2) const;
        void    removeObject(const int nId1, const int nId2);

    private:
        typedef std::pair<int, int> tKey;
        typedef std::map<tKey, CBond> tBondMap;

        tKey hashAlgorithm(const int nId1, const int nId2) const;

        tBondMap m_bondMap; /**< map of current bonds */
};

#endif

