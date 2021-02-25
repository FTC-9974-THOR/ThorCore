//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//


#include <stdio.h>
#include "pixydefs.h"
#include "usblink.h"
#include "debuglog.h"
#include "util.h"

USBLink::USBLink()
{
    m_handle = 0;
    m_context = 0;
    m_blockSize = 64;
    m_flags = LINK_FLAG_ERROR_CORRECTED;
}

USBLink::~USBLink()
{
    close();
}

// modified for Android
int USBLink::open(uint32_t fd)
{
    close();

    // prevent libusb from scanning usb devices on libusb_init(). we don't actually "own" the USB
    // bus, the Android OS does.
    libusb_set_option(NULL, LIBUSB_OPTION_WEAK_AUTHORITY);
    libusb_set_option(m_context, LIBUSB_OPTION_WEAK_AUTHORITY, NULL);

    libusb_init(&m_context);

    return openDevice(fd);
}

void USBLink::close()
{
    if (m_handle)
    {
        libusb_close(m_handle);
        m_handle = 0;
    }
    if (m_context)
    {
        libusb_exit(m_context);
        m_context = 0;
    }
}

/*
int USBLink::openDevice()
{
    libusb_device **list = NULL;
    int i, count = 0;
    libusb_device *device;
    libusb_device_descriptor desc;

    count = libusb_get_device_list(m_context, &list);

    for (i=0; i<count; i++)
    {
        device = list[i];
        libusb_get_device_descriptor(device, &desc);

        if (desc.idVendor==PIXY_VID && desc.idProduct==PIXY_PID)
        {
            if (libusb_open(device, &m_handle)==0)
            {
            #ifdef __MACOS__
                libusb_reset_device(m_handle);
            #endif
                if (libusb_set_configuration(m_handle, 1)<0)
                {
                    libusb_close(m_handle);
                    m_handle = 0;
                    continue;
                }
                if (libusb_claim_interface(m_handle, 1)<0)
                {
                    libusb_close(m_handle);
                    m_handle = 0;
                    continue;
                }
#ifdef __LINUX__
                libusb_reset_device(m_handle);
#endif
                break;
            }
        }
    }
    libusb_free_device_list(list, 1);
    if (i==count) // no devices found
        return -1;
    return 0;
}
*/

/**
 * Opens a Pixy on an Android-owned USB file descriptor.
 *
 * @param fd file descriptor of the USB port the Pixy is on
 * @returns 0 on success
 * @returns LIBUSB_ERROR_NO_MEM on memory allocation failure
 * @returns LIBUSB_ERROR_ACCESS on insufficient permissions
 * @returns LIBUSB_ERROR_BUSY if something else is using the Pixy
 * @returns LIBUSB_ERROR_NO_DEVICE if the Pixy becomes disconnected
 *
 */
// modified for Android
int USBLink::openDevice(uint32_t fd) {
    // The Android system owns the USB devices, and we can only ask the OS to do USB operations.
    // to do so, Android gives us a file descriptor we can do I/O operations on to perform USB read/writes.
    // opening and closing of this device must be done by the Java layer. we can call libusb_close(),
    // and in fact we should once we're done with the native handle, but we're not allowed to call
    // libusb_open(). instead, we use libusb_wrap_sys_device() to open the file descriptor that Java
    // gave us.
    int retCode = libusb_wrap_sys_device(m_context, fd, &m_handle);
    if (retCode == 0) {
        retCode = libusb_set_configuration(m_handle, 1);
        if (retCode < 0) {
            libusb_close(m_handle);
            m_handle = nullptr;
            return retCode;
        }
        retCode = libusb_claim_interface(m_handle, 1);
        if (retCode < 0) {
            libusb_close(m_handle);
            m_handle = nullptr;
            return retCode;
        }
        return libusb_reset_device(m_handle);
    }
    return retCode;
}

int USBLink::send(const uint8_t *data, uint32_t len, uint16_t timeoutMs)
{
    int res, transferred;

    if (timeoutMs==0) // 0 equals infinity
        timeoutMs = 10;

    if ((res=libusb_bulk_transfer(m_handle, 0x02, (unsigned char *)data, len, &transferred, timeoutMs))<0)
    {
#ifdef __MACOS__
        libusb_clear_halt(m_handle, 0x02);
#endif
        log("libusb_bulk_write %d", res);
        return res;
    }
    return transferred;
}

int USBLink::receive(uint8_t *data, uint32_t len, uint16_t timeoutMs)
{
    int res, transferred;

    if (timeoutMs==0) // 0 equals infinity
        timeoutMs = 100;

    // Note: if this call is taking more time than than expected, check to see if we're connected as USB 2.0.  Bad USB cables can
    // cause us to revert to a 1.0 connection.
    if ((res=libusb_bulk_transfer(m_handle, 0x82, (unsigned char *)data, len, &transferred, timeoutMs))<0)
    {
#ifdef __MACOS__
        libusb_clear_halt(m_handle, 0x82);
#endif
        //log("libusb_bulk_read %d", res);
        return res;
    }
    return transferred;
}

void USBLink::setTimer()
{
  m_timer = millis();
}

uint32_t USBLink::getTimer()
{
  uint32_t time = clock() - m_timer;

  return time;
}




