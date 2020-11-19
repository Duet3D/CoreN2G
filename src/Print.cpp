/*
 Print.cpp - members of Print class
 */

#include "Print.h"

// This default implementation may be overridden in derived classes for optimisation purposes
size_t Print::write(const uint8_t *buffer, size_t size) noexcept
{
	size_t n = 0;
	while (size--)
	{
		n += write(*buffer++);
	}
	return n;
}

// End
