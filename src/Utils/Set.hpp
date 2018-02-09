#ifndef ABETARE__SET_HPP_
#define ABETARE__SET_HPP_

#include "Utils/HashFn.hpp"
#include <unordered_set>

#define IntSet std::unordered_set<int, HashStruct<int> >	
#define UseSet(Key)  std::unordered_set<Key, HashStruct<Key> >

#endif
    
    
    
    







