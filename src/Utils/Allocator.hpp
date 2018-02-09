#ifndef ABETARE__ALLOCATOR_HPP_
#define ABETARE__ALLOCATOR_HPP_

#include "Utils/Misc.hpp"

namespace Abetare
{
    template <typename Type>
    class Allocator
    {
    public:
	Allocator(void) : m_dim(0)
	{
	}
	
	virtual ~Allocator(void)
	{
	}
	
	virtual void SetDim(const int dim)
	{
	    m_dim = dim;
	}

	virtual int GetDim(void) const
	{
	    return m_dim;
	}
			
	virtual Type* New(void) const
	{
	    return new Type[GetDim()];
	}
	
	virtual void Delete(Type * const s) const
	{
	    if(s)
		delete[] s;
	}
	
	virtual void Copy(Type * const dest, const Type * const src) const
	{
	    CopyArray<Type>(dest, GetDim(), src);
	}
	
	virtual Type* Copy(const Type * const src) const
	{
	    Type *dest = New();
	    Copy(dest, src);
	    return dest;
	}

    protected:
	int m_dim;
	
    };    
}

#endif

