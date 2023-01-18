/**
 * @file syscalls.h
 * This file defines replacement system calls functions for the standard library
 * The client application project must #include it in exactly one .cpp file
 *
 * It cannot be a .cpp file in this project because if we do that, functions from the standard library get used instead of these functions.
 *
 * The including file may declare "#define SystemStackSize xxxx" before including this, otherwise a default will be used
 */

#include <Core.h>
#include <sys/stat.h>
#include <cerrno>
#include <new>

extern char _end;										// defined by the linker script
extern uint32_t _estack;

[[noreturn]] void OutOfMemoryHandler() noexcept;		// this must be provided by the client application

#if RP2040

// The RP2040 SDK allocates a 2Kb stack for Core 0 at the top of the 4kb Scratch Y memory, and a 2kb stack for Core 1 at the top of the 4kb Scratch X memory.
// It defines __StackLimit as the very top of the 256kb of normal RAM, and __StackBottom as the bottom of Core 0 stack.
// It includes the _sbrk function in file runtime.c which also includes other functions that are needed, so we can't replace _sbrk until we modify the SDK.

extern uint32_t __StackLimit;							// defined by the linker script (poorly named, see linker script)
extern uint32_t __StackBottom;

const char *sysStackLimit = (const char*)&__StackBottom;
const char *heapLimit = (const char*)&__StackLimit;

#else

#ifndef SystemStackSize
# define SystemStackSize	(1024)
#endif

const char *sysStackLimit = (const char*)&_estack - SystemStackSize;
const char *heapLimit = (const char*)&_estack - SystemStackSize;

#endif

char *heapTop = &_end;

/**
 * \brief Replacement of C library of _sbrk
 */
extern "C" void * _sbrk(ptrdiff_t incr) noexcept
{
	char *newHeap = heapTop + incr;
	if (newHeap <= heapLimit)
	{
		void * const prev_heap = heapTop;
		heapTop = newHeap;
		return prev_heap;
	}

	OutOfMemoryHandler();

	// The out of memory handle usually terminates, but in case it doesn't, try to return failure. Unfortunately, this doesn't seem to work with newlib.
	errno = ENOMEM;
	return reinterpret_cast<void*>(-1);
}

/**
 * \brief Allocate memory permanently. In multi-threaded environments, take the malloc mutex before calling this.
 */
void *CoreAllocPermanent(size_t sz, std::align_val_t align) noexcept
{
	char * const newHeapLimit = reinterpret_cast<char *>(reinterpret_cast<uint32_t>(heapLimit - sz) & ~((uint32_t)align - 1));
	if (newHeapLimit < heapTop)
	{
		OutOfMemoryHandler();
	}
	heapLimit = newHeapLimit;
	return newHeapLimit;
}

/**
 * \brief Replacement of C library of link
 */
extern "C" int link(char *old, char *_new) noexcept
{
	(void)old, (void)_new;
	return -1;
}

/**
 * \brief Replacement of C library of _close
 */
extern "C" int _close(int file) noexcept
{
	(void)file;
	return -1;
}

/**
 * \brief Replacement of C library of _fstat
 */
extern "C" int _fstat(int file, struct stat *st) noexcept
{
	(void)file;
	st->st_mode = S_IFCHR;

	return 0;
}

/**
 * \brief Replacement of C library of _isatty
 */
extern "C" int _isatty(int file) noexcept
{
	(void)file;
	return 1;
}

/**
 * \brief Replacement of C library of _lseek
 */
extern "C" int _lseek(int file, int ptr, int dir) noexcept
{
	(void)file, (void)ptr, (void)dir;
	return 0;
}

/**
 * \brief Replacement of C library of _exit and related functions
 */
extern "C" void _exit(int status) noexcept
{
	for (;;) { }
}

extern "C" void exit(int code) noexcept
{
	_exit(code);
}

extern "C" void abort() noexcept
{
	_exit(1);
}

extern "C" int __register_exitproc (int type, void (*fn) (void), void *arg, void *d) noexcept
{
	// We don't support exit() so we don't need to register any calls
	return 0;
}

/**
 * \brief Replacement of C library of _kill
 */
extern "C" void _kill(int pid, int sig) noexcept
{
	return;
}

/**
 * \brief Replacement of C library of _getpid
 */
extern "C" int _getpid() noexcept
{
	return -1;
}

extern "C" int _read(int file, char *ptr, int len) noexcept
{
	(void)file, (void)ptr, (void)len;
    return 0;
}

extern "C" int _write(int file, char *ptr, int len) noexcept
{
	(void)file, (void)ptr;
	return len;
}

#if RP2040

// Replacements for RP2040 SDK functions that try to print stuff

extern "C" [[noreturn]] void vAssertCalled(uint32_t line, const char *file) noexcept;

extern "C" void __attribute__((noreturn)) panic_unsupported() noexcept
{
    panic("not supported");
}

extern "C" void __attribute__((noreturn)) __printflike(1, 0) panic(const char *fmt, ...) noexcept
{
	vAssertCalled(__LINE__, __FILE__);
}

extern "C" void __assert_func (const char *file, int line, const char *func, const char *failedexpr) noexcept
{
	vAssertCalled(__LINE__, __FILE__);
}

#include <cstdarg>

extern "C" void debugVprintf(const char *fmt, va_list vargs) noexcept;

extern "C" int printf(const char *fmt, ...) noexcept
{
	va_list vargs;
	va_start(vargs, fmt);
	debugVprintf(fmt, vargs);
	va_end(vargs);
	return 0;
}

#endif

// End
