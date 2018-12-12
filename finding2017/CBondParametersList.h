#if !defined(cbondparameterslist_h)
#define cbondparameterslist_h

#include <map>
#include <string>

#include "CBondParameters.h"

class CBondParametersList: public std::map<std::string, CBondParameters>
{
    public:
        static const std::string JOIN_STRING;

        void addBondParameters(const std::string & key1,
                               const std::string &key2,
                               CBondParameters bondParameters);

        CBondParameters getBondParameters(const std::string &key1,
                                          const std::string &key2);

};

#endif