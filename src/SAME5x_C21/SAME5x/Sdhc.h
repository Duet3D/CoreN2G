/**
 * \file
 *
 * \brief SAM MCI HPL
 *
 * Copyright (c) 2016-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

#ifndef _HPL_MCI_H_INCLUDED
#define _HPL_MCI_H_INCLUDED

#include <Core.h>

#if SUPPORT_SDHC

#ifdef __cplusplus
extern "C" {
#endif

//! \name Flags used to define MCI parser for SD/MMC command
//! @{
//! Have response
#define MCI_RESP_PRESENT (1lu << 8)
//! 136 bit response
#define MCI_RESP_136 (1lu << 11)
//! Expect valid crc
#define MCI_RESP_CRC (1lu << 12)
//! Card may send busy
#define MCI_RESP_BUSY (1lu << 13)
//! Open drain for a braodcast command
#define MCI_CMD_OPENDRAIN (1lu << 14)
//! To signal a data write operation
#define MCI_CMD_WRITE (1lu << 15)
//! To signal a SDIO tranfer in multi byte mode
#define MCI_CMD_SDIO_BYTE (1lu << 16)
//! To signal a SDIO tranfer in block mode
#define MCI_CMD_SDIO_BLOCK (1lu << 17)
//! To signal a data transfer in stream mode
#define MCI_CMD_STREAM (1lu << 18)
//! To signal a data transfer in single block mode
#define MCI_CMD_SINGLE_BLOCK (1lu << 19)
//! To signal a data transfer in multi block mode
#define MCI_CMD_MULTI_BLOCK (1lu << 20)
//! @}

/**
 *  \brief Initialize MCI low level driver.
 *
 *  \return Operation status.
 */
void hsmci_init(Sdhc *p_hw, enum IRQn irqn) noexcept;

/**
 *  \brief Select a device and initialize it
 *
 *  \param[in] slot    Selected slot
 *  \param[in] clock   Maximum clock to use (Hz)
 *  \param[in] bus_width  Bus width to use (1, 4 or 8)
 *  \param[in] high_speed true, to enable high speed mode
 *  \return true if success
 */
bool hsmci_select_device(uint8_t slot, uint32_t clock, uint8_t bus_width, bool high_speed) noexcept;

/**
 *  \brief Deselect a device by an assigned slot
 *
 *  \param[in] slot    Selected slot
 */
void hsmci_deselect_device(uint8_t slot) noexcept;

/**
 *  \brief Get the maximum bus width of a device
 *         by a selected slot
 *
 *  \param[in] slot    Selected slot
 *  \return bus width.
 */
uint8_t hsmci_get_bus_width(uint8_t slot) noexcept;

/**
 *  \brief Get the high speed capability of the device.
 *
 *  \return true, if the high speed is supported.
 */
bool hsmci_is_high_speed_capable() noexcept;

/**
 *  \brief Send 74 clock cycles on the line.
 *   Note: It is required after card plug and before card install.
 */
void hsmci_send_clock() noexcept;

/**
 *  \brief Send a command on the selected slot
 *
 *  \param[in] cmd        Command definition
 *  \param[in] arg        Argument of the command
 *  \return true if success, otherwise false
 */
bool hsmci_send_cmd(uint32_t cmd, uint32_t arg) noexcept;

/**
 *  \brief Get 32 bits response of the last command.
 *
 *  \return 32 bits response.
 */
uint32_t hsmci_get_response() noexcept;

/**
 *  \brief Get 128 bits response of the last command.
 *
 *  \param[in] response   Pointer on the array to fill
 *                        with the 128 bits response.
 */
void hsmci_get_response_128(uint8_t *response) noexcept;

/**
 *  \brief Send an ADTC command on the selected slot
 *         An ADTC (Addressed Data Transfer Commands)
 *         command is used for read/write access..
 *
 *  \param[in] cmd          Command definition.
 *  \param[in] arg          Argument of the command.
 *  \param[in] block_size   Block size used for the transfer.
 *  \param[in] nb_block     Total number of block for this transfer
 *  \param[in] access_block if true, the x_read_blocks() and x_write_blocks()
 *                          functions must be used after this function.
 *                          If false, the mci_read_word() and mci_write_word()
 *                          functions must be used after this function.
 *
 * \return true if success, otherwise false
 */
bool hsmci_adtc_start(uint32_t cmd, uint32_t arg, uint16_t block_size, uint16_t nb_block, const void *dmaAddr) noexcept;

/**
 *  \brief Send a command to stop an ADTC command on the selected slot.
 *
 * \param[in] cmd        Command definition
 * \param[in] arg        Argument of the command
 *
 * \return true if success, otherwise false
 */
bool hsmci_adtc_stop(uint32_t cmd, uint32_t arg) noexcept;

/**
 *  \brief Read a word on the line.
 *
 *  \param[in] value  Pointer on a word to fill.
 *
 *  \return true if success, otherwise false
 */
bool hsmci_read_word(uint32_t *value) noexcept;

/**
 *  \brief Write a word on the line
 *
 *  \param[in] value  Word to send
 *
 *  \return true if success, otherwise false
 */
bool hsmci_write_word(uint32_t value) noexcept;

/**
 *  \brief Start a read blocks transfer on the line
 *  Note: The driver will use the DMA available to speed up the transfer.
 *
 *  \param[in] dst        Pointer on the buffer to fill
 *  \param[in] nb_block   Number of block to transfer
 *
 *  \return true if started, otherwise false
 */
bool hsmci_start_read_blocks(void *dst, uint16_t nb_block) noexcept;

/**
 *  \brief Start a write blocks transfer on the line
 *  Note: The driver will use the DMA available to speed up the transfer.
 *
 *  \param[in] src        Pointer on the buffer to send
 *  \param[in] nb_block   Number of block to transfer
 *
 *  \return true if started, otherwise false
 */
bool hsmci_start_write_blocks(const void *src, uint16_t nb_block) noexcept;

/**
 *  \brief Wait the end of transfer initiated by mci_start_read_blocks()
 *
 *  \return true if success, otherwise false
 */
bool hsmci_wait_end_of_read_blocks() noexcept;

/**
 *  \brief Wait the end of transfer initiated by mci_start_write_blocks()
 *
 *  \return true if success, otherwise false
 */
bool hsmci_wait_end_of_write_blocks() noexcept;

uint32_t hsmci_get_speed() noexcept;

#ifdef __cplusplus
}
#endif

#endif	// SUPPORT_SDHC

#endif /* _HPL_MCI_H_INCLUDED */
