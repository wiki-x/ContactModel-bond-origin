#include "CBondParametersList.h"

#include <vector>
#include <algorithm>

using namespace std;

const string CBondParametersList::JOIN_STRING = "~_~_~";

void CBondParametersList::addBondParameters(const string & key1,
                                            const string &key2,
                                            CBondParameters bondParameters)
{
    vector<string> keys;
    keys.push_back(key1);
    keys.push_back(key2);

    //Use STL sort to put them in alphabetical order.
    sort(keys.begin(), keys.end());

    string nKey = keys[0] + JOIN_STRING + keys[1];
    insert( pair<string, CBondParameters>(nKey, bondParameters) );
}

CBondParameters CBondParametersList::getBondParameters(const string & key1,
                                                       const string &key2)
{
    vector<string> keys;
    keys.push_back(key1);
    keys.push_back(key2);

    //Use STL sort to put them in alphabetical order.
    sort(keys.begin(), keys.end());

    string nKey = keys[0] + JOIN_STRING + keys[1];

    map<string, CBondParameters>::iterator it;
    if((it = find(nKey)) != end() )
        return it->second;
    else
        return CBondParameters();
}