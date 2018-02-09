#ifndef ABETARE__HASH_FN_HPP_
#define ABETARE__HASH_FN_HPP_

#include "Utils/Definitions.hpp"
#include <cstring>

namespace Abetare
{    
    size_t StringHash(const char *s, const int n);
    
    template <class Key_t> struct HashStruct { };

    template<> struct HashStruct<char*>
    {size_t operator()(const char* s) const { return s ? StringHash(s, strlen(s)) : 0;}};

    template<> struct HashStruct<const char*>
    {size_t operator()(const char* s) const { return s ? StringHash(s, strlen(s)) : 0;}};
    
    template<> struct HashStruct<char>{size_t operator()(char x) const { return StringHash(&x, sizeof(char));}};

    template<> struct HashStruct<unsigned char> 
    {size_t operator()(unsigned char x) const { return StringHash((const char *)(&x), sizeof(x));}};

    template<> struct HashStruct<signed char>
    {size_t operator()(unsigned char x) const { return StringHash((const char *)(&x), sizeof(x));}};

    template<> struct HashStruct<short> 
    {size_t operator()(short x) const { return StringHash((const char *)(&x), sizeof(x));}};

    template<> struct HashStruct<unsigned short> 
    {size_t operator()(unsigned short x) const { return StringHash((const char *)(&x), sizeof(x));}};

    template<> struct HashStruct<int> 
    {size_t operator()(int x) const { return StringHash((const char *)(&x), sizeof(x));}};
    
    template<> struct HashStruct<unsigned int> 
    {size_t operator()(unsigned int x) const { return StringHash((const char *)(&x), sizeof(x));}};

    template<> struct HashStruct<long> 
    {size_t operator()(long x) const { return StringHash((const char *)(&x), sizeof(x));}};
    
    template<> struct HashStruct<unsigned long> 
    {size_t operator()(unsigned long x) const { return StringHash((const char *)(&x), sizeof(x));}};

    template<> struct HashStruct<MyKey> 
    {size_t operator()(MyKey x) const { return StringHash((const char *)(&x), sizeof(x));}};

}

#endif
    
    
    
    







