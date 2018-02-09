#ifndef ABETARE__LOGIC_DFA_IO_HPP_
#define ABETARE__LOGIC_DFA_IO_HPP_

#include "Logic/DFA.hpp"
#include <cstdio>

namespace Abetare
{
    class DFAIO
    {
    public:
	static DFA* ReadLBT(FILE * const in);
	static void PrintDotty(const DFA * const dfa, FILE * const out);
	
    };
    
}
    
#endif



