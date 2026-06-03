/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#if SUPPORT_USB && !CORE_USES_TINYUSB

#include "SerialCDC.h"
#include "usbinterface.h"
#include "CoreImp.h"

extern __IO  uint32_t lineState;

void serialEventUSB() __attribute__((weak));

void SerialCDC::begin(void) noexcept
{
  usb_init();
}

void SerialCDC::begin(uint32_t /* baud_count */) noexcept
{
  // uart config is ignored in USB-CDC
  begin();
}

void SerialCDC::begin(uint32_t /* baud_count */, uint8_t /* config */) noexcept
{
  // uart config is ignored in USB-CDC
  begin();
}

void SerialCDC::Start(Pin VBusPin) noexcept
{
  begin();
}

void SerialCDC::end() noexcept
{
  usb_deinit();
}

int SerialCDC::availableForWrite() noexcept
{
  // Just transmit queue size, available for write
  return (int)usb_available_for_write();
}

size_t SerialCDC::write(uint8_t ch) noexcept
{
  // Just write single-byte buffer.
  return write(&ch, 1);
}

size_t SerialCDC::write(const uint8_t *buffer, size_t size) noexcept
{
  size_t rest = size;
  while (rest > 0 && usb_is_connected()) {
    // Determine buffer size available for write
    auto portion = (size_t)usb_available_for_write();
    // Truncate it to content size (if rest is greater)
    if (rest < portion) {
      portion = rest;
    }
    if (portion > 0) {
      // Only if some space in the buffer exists.
      // TS: Only main thread calls write and writeSize methods,
      // it's thread-safe since IRQ does not affects
      // TransmitQueue write position
      usb_write(buffer, portion);
      rest -= portion;
      buffer += portion;
    }
  }
  return size - rest;
}

int SerialCDC::available(void) noexcept
{
  // Just ReceiveQueue size, available for reading
  return static_cast<int>(usb_available_for_read());
}

int SerialCDC::read(void) noexcept
{
  // Dequeue only one char from queue
  // TS: it safe, because only main thread affects ReceiveQueue->read pos
  uint8_t ch;
  uint32_t ret = usb_read(&ch, 1);
  if (ret == 0) return -1;
  return (int)ch;
}

size_t SerialCDC::readBytes(char *buffer, size_t length) noexcept
{
  return usb_read((uint8_t *)buffer, length);
#if 0
  _startMillis = millis();
  do {
    read = CDC_ReceiveQueue_Read(&ReceiveQueue, reinterpret_cast<uint8_t *>(buffer), rest);
    CDC_resume_receive();
    rest -= read;
    buffer += read;
    if (rest == 0) {
      return length;
    }
  } while (millis() - _startMillis < _timeout);
  return length - rest;
#endif
}

int SerialCDC::peek(void) noexcept
{
  // Peek one symbol, it can't change receive avaiablity
  return -1;
}

void SerialCDC::flush(void) noexcept
{
  // Wait for TransmitQueue read size becomes zero
  // TS: safe, because it not be stopped while receive 0
  while (!usb_is_write_empty()) {}
}

uint32_t SerialCDC::baud() noexcept
{
  return 115200;
}

uint8_t SerialCDC::stopbits() noexcept
{
  return ONE_STOP_BIT;
}

uint8_t SerialCDC::paritytype() noexcept
{
  return NO_PARITY;
}

uint8_t SerialCDC::numbits() noexcept
{
  return 8;
}

bool SerialCDC::dtr(void) noexcept
{
  return false;
}

bool SerialCDC::rts(void) noexcept
{
  return false;
}

SerialCDC::operator bool() noexcept
{
  bool result = false;
  if (lineState == 1) {
    result = true;
  }
  delay(10);
  return result;
}

bool SerialCDC::IsConnected() noexcept
{
  return usb_is_connected();
}

#endif // USBCON && USBD_USE_CDC