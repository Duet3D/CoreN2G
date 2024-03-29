/*
  Stream.h
*/

#ifndef HARDWARE_SAME5X_STREAM_H
#define HARDWARE_SAME5X_STREAM_H

#include "Print.h"

class Stream : public Print
{
public:
	Stream() noexcept {}

	virtual int available() noexcept = 0;
	virtual int read() noexcept = 0;
	virtual void flush() noexcept = 0;
	virtual size_t canWrite() noexcept = 0;
	virtual size_t readBytes(char *_ecv_array buffer, size_t length) noexcept;	// this one has a default implementation, but can be overridden

	size_t readBytes(uint8_t *_ecv_array buffer, size_t length) noexcept { return readBytes(reinterpret_cast<char *_ecv_array>(buffer), length); }
};

#endif
