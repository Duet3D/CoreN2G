/*
 * McuType.h
 *
 *  Created on: 29 May 2026
 *      Author: David
 */

#ifndef SRC_MCUTYPE_H_
#define SRC_MCUTYPE_H_

#if defined(__SAME54P20A__) || defined(__SAME51P20A__)
# define SAMC21				0
# define SAM3XA				0
# define SAM4E				0
# define SAM4S				0
# define SAME5x				1
# define SAME70				0
# define RP2040				0
# define RP2350				0
# define STM32H5			0
# define STM32H7			0
#elif defined(__SAME51N19A__) || defined(__SAME51G19A__) || defined(__SAME51J19A__)
# define SAMC21				0
# define SAM3XA				0
# define SAM4E				0
# define SAM4S				0
# define SAME5x				1
# define SAME70				0
# define RP2040				0
# define STM32H5			0
# define STM32H7			0
#elif defined(__SAMD51N19A__)
# define SAMC21				0
# define SAM3XA				0
# define SAM4E				0
# define SAM4S				0
# define SAME5x				1
# define SAME70				0
# define RP2040				0
# define RP2350				0
# define STM32H5			0
# define STM32H7			0
#elif defined(__SAMC21G18A__)
# define SAMC21				1
# define SAM3XA				0
# define SAM4E				0
# define SAM4S				0
# define SAME5x				0
# define SAME70				0
# define RP2040				0
# define RP2350				0
# define STM32H5			0
# define STM32H7			0
# define SUPPORT_SDHC		0			// SAMC21 doesn't support SDHC
# define SUPPORT_USB		0			// SAMC21 doesn't support USB
#elif defined(__SAM4E8E__)
# define SAME5x				0
# define RP2040				0
# define RP2350				0
# define STM32H5			0
# define STM32H7			0
# define SUPPORT_CAN		0			// SAM4E doesn't support CAN-FD
#elif defined(__SAM4S8C__)
# define SAME5x				0
# define RP2040				0
# define RP2350				0
# define STM32H5			0
# define STM32H7			0
# define SUPPORT_CAN		0			// SAM4S doesn't support CAN-FD
#elif defined(__SAME70Q20B__)
# define SAME5x				0
# define RP2040				0
# define RP2350				0
# define STM32H5			0
# define STM32H7			0
#elif defined __RP2040__
# define RP2040				1
# define RP2350				0
# define SAMC21				0
# define SAM3XA				0
# define SAM4E				0
# define SAM4S				0
# define SAME5x				0
# define SAME70				0
# define STM32H5			0
# define STM32H7			0
# define SUPPORT_SDHC		0			// we don't support SD cards on any RP2040-based boards
#elif defined __RP2350__
# define RP2040				0
# define RP2350				1
# define SAMC21				0
# define SAM3XA				0
# define SAM4E				0
# define SAM4S				0
# define SAME5x				0
# define SAME70				0
# define STM32				0
# define SUPPORT_SDHC		0			// we don't support SD cards on any RP2350-based boards
#elif defined(STM32H523xx)
# define RP2040				0
# define SAMC21				0
# define SAM3XA				0
# define SAM4E				0
# define SAM4S				0
# define SAME5x				0
# define SAME70				0
# define STM32H5			1
# define STM32H7			0
#elif defined(STM32H743xx)
# define RP2040				0
# define SAMC21				0
# define SAM3XA				0
# define SAM4E				0
# define SAM4S				0
# define SAME5x				0
# define SAME70				0
# define STM32H5			0
# define STM32H7			1
#else
# error unsupported processor
#endif

#define STM32		(STM32H5 || STM32H7)
#define RPXXXX		(RP2040 || RP2350)

#endif /* SRC_MCUTYPE_H_ */
