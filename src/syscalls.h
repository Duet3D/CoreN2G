/**
 * @file syscalls.h
 * This file defines replacement system calls functions for the standard library
 * The client application project must #include it in exactly one .cpp file
 *
 * It cannot be a .cpp file in this project because if we do that, functions from the standard library get used instead of these functions.
 *
 */

#include <sys/stat.h>

/*----------------------------------------------------------------------------
 *        Exported variables
 *----------------------------------------------------------------------------*/

#undef errno
int errno = 0;

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

extern int _end;

/**
 * \brief Replacement of C library of _sbrk
 */
extern "C" char * _sbrk(int incr) noexcept
{
	static char *heap = nullptr;

	if (heap == nullptr)
	{
		heap = (char *)&_end;
	}
	char *prev_heap = heap;

	heap += incr;

	return prev_heap;
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
 * \brief Replacement of C library of _exit
 */
extern "C" void _exit(int status) noexcept
{
	for (;;) { }
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

// End
