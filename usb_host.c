
/********************************** (C) COPYRIGHT *******************************
* File Name          : USBHOST.C
* Author             : WCH
* Version            : V1.1
* Date               : 2018/02/28
* Description        : CH554 USB Host interface functions
*******************************************************************************/

#include <stdio.h>
#include <string.h>
#include "ch554.h"
#include "debug.h"
#include "usb_host.h"

extern __xdata __at (0x0000) uint8_t RxBuffer[MAX_PACKET_SIZE];
extern __xdata __at (0x0040) uint8_t TxBuffer[MAX_PACKET_SIZE];

#include "ch554_usb.h"


#define DE_PRINTF 1

__bit HubLowSpeed;

// Define a user temporary buffer, which is used to process descriptors during enumeration.
// It can also be used as a normal temporary buffer after enumeration.
__xdata uint8_t Com_Buffer[COM_BUF_SIZE];


/*******************************************************************************
* Function Name  : DisableRootHubPort( )
* Description    : Close HUB port
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DisableRootHubPort()
{
#ifdef    FOR_ROOT_UDISK_ONLY
    CH554DiskStatus = DISK_DISCONNECT;
#endif
#ifndef    DISK_BASE_BUF_LEN
    ThisUsbDev.DeviceStatus = ROOT_DEV_DISCONNECT;
    ThisUsbDev.DeviceAddress = 0x00;
#endif
}


/*******************************************************************************
* Function Name: AnalyzeRootHub(void)
* Description: Analyze the ROOT-HUB status and handle the device plug-in and unplug events of the ROOT-HUB port.
               If the device is unplugged, the DisableRootHubPort() function is called in the function to close the port,
               insert the event, and set the status bit of the corresponding port.
* Input: None
* Output: None
* Return: Return ERR_SUCCESS if there is no situation,
*         return ERR_USB_CONNECT if a new connection is detected,
*         return ERR_USB_DISCON if a disconnection is detected
*******************************************************************************/
uint8_t AnalyzeRootHub(void)
{
    if (USB_MIS_ST & bUMS_DEV_ATTACH) {                                        // Device exists
#ifdef DISK_BASE_BUF_LEN
        if ((CH554DiskStatus == DISK_DISCONNECT) || ((UHOST_CTRL & bUH_PORT_EN) == 0))
#else
        if ((ThisUsbDev.DeviceStatus == ROOT_DEV_DISCONNECT) || ((UHOST_CTRL & bUH_PORT_EN) == 0)) // Device detected plugged in
#endif
        {
            // It is detected that a device is plugged in, but it has not been allowed yet, indicating that it has just been plugged in.
            DisableRootHubPort();                                                   // close port
#ifdef DISK_BASE_BUF_LEN
            CH554DiskStatus = DISK_CONNECT;
#else
            ThisUsbDev.DeviceStatus = ROOT_DEV_CONNECTED;                            // Set connection flag
#endif
#if DE_PRINTF
            printf("USB dev in\n");
#endif
            return ERR_USB_CONNECT;
        }
    }
#ifdef DISK_BASE_BUF_LEN
    else if (CH554DiskStatus >= DISK_CONNECT)
#else
    else if (ThisUsbDev.DeviceStatus >= ROOT_DEV_CONNECTED)                 // Device unplug detected
#endif
    {
        DisableRootHubPort();                                                     // close port
#if DE_PRINTF
        printf("USB dev out\n");
#endif
        return ERR_USB_DISCON;

    }
    return ERR_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetHostUsbAddr
* Description    : Set the USB device address currently operated by the USB host
* Input          : uint8_t addr
* Output         : None
* Return         : None
*******************************************************************************/
void SetHostUsbAddr(uint8_t addr)
{
    USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | addr & 0x7F;
}

#ifndef    FOR_ROOT_UDISK_ONLY


/*******************************************************************************
* Function Name  : SetUsbSpeed
* Description    : Set current USB speed
* Input          : uint8_t FullSpeed
* Output         : None
* Return         : None
*******************************************************************************/
void SetUsbSpeed(uint8_t FullSpeed)
{
    if (FullSpeed) {                                                           // full speed
        USB_CTRL &= ~ bUC_LOW_SPEED;                                           // full speed
        UH_SETUP &= ~ bUH_PRE_PID_EN;                                          // Disable PRE PID
    } else {
        USB_CTRL |= bUC_LOW_SPEED;                                             // low speed
    }
}
#endif


/*******************************************************************************
* Function Name  : ResetRootHubPort( )
* Description    : After a device is detected, the bus is reset to prepare for device enumeration and is set to default to full speed.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ResetRootHubPort()
{
    UsbDevEndp0Size = DEFAULT_ENDP0_SIZE;                      // Maximum packet size for USB device endpoint 0
#ifndef DISK_BASE_BUF_LEN
    memset(&ThisUsbDev, 0, sizeof(ThisUsbDev));                // Clear the structure
#endif
    SetHostUsbAddr(0x00);
    UHOST_CTRL &= ~bUH_PORT_EN;                                // close port
    SetUsbSpeed(1);                                            // Default is full speed
    UHOST_CTRL = UHOST_CTRL & ~ bUH_LOW_SPEED | bUH_BUS_RESET; // Default is full speed, start reset
    mDelaymS(20);                                              // Reset time 10mS to 20mS
    UHOST_CTRL = UHOST_CTRL & ~ bUH_BUS_RESET;                 // End reset
    mDelayuS(250);
    UIF_DETECT = 0;                                            // Clear interrupt flag
}


/*******************************************************************************
* Function Name  : EnableRootHubPort( )
* Description    : Enable the ROOT-HUB port and set the corresponding bUH_PORT_EN to 1 to enable the port.
                   Disconnection of the device may result in return failure.
* Input          : None
* Output         : None
* Return         : Return ERR_SUCCESS if a new connection is detected, and return ERR_USB_DISCON if there is no connection.
*******************************************************************************/
uint8_t EnableRootHubPort()
{
#ifdef DISK_BASE_BUF_LEN
    if (CH554DiskStatus < DISK_CONNECT) {
        CH554DiskStatus = DISK_CONNECT;
    }
#else
    if (ThisUsbDev.DeviceStatus < ROOT_DEV_CONNECTED) {
        ThisUsbDev.DeviceStatus = ROOT_DEV_CONNECTED;
    }
#endif
    if (USB_MIS_ST & bUMS_DEV_ATTACH) {                                      // Have equipment
#ifndef DISK_BASE_BUF_LEN
        if ((UHOST_CTRL & bUH_PORT_EN) == 0x00) {                            // Not yet enabled
            ThisUsbDev.DeviceSpeed = USB_MIS_ST & bUMS_DM_LEVEL ? 0 : 1;
            if (ThisUsbDev.DeviceSpeed == 0) {
                UHOST_CTRL |= bUH_LOW_SPEED;      // low speed
            }
        }
#endif
        USB_CTRL |= bUC_DMA_EN;     // Start the USB host and DMA, and automatically pause before the interrupt flag is cleared.
        UH_SETUP = bUH_SOF_EN;
        UHOST_CTRL |= bUH_PORT_EN;  // Enable HUB port
        return (ERR_SUCCESS);
    }
    return (ERR_USB_DISCON);
}
#ifndef DISK_BASE_BUF_LEN


/*******************************************************************************
* Function Name  : SelectHubPort( uint8_t HubPortIndex )
* Description    : Select the HUB port to be operated
* Input          : uint8_t HubPortIndex Select the specified port of the external HUB to operate the specified ROOT-HUB port.
* Output         : None
* Return         : None
*******************************************************************************/
void SelectHubPort(uint8_t HubPortIndex)
{
    if (HubPortIndex) { // Select the specified port of the external HUB of the ROOT-HUB port specified by the operation
        SetHostUsbAddr(DevOnHubPort[HubPortIndex-1].DeviceAddress); // Set the USB device address currently operated by the USB host
        SetUsbSpeed(DevOnHubPort[HubPortIndex-1].DeviceSpeed); // Set the current USB speed
        if (DevOnHubPort[HubPortIndex-1].DeviceSpeed == 0) { // Communication with low-speed USB devices through an external HUB requires a prefix ID
            UH_SETUP |= bUH_PRE_PID_EN; // Enable PRE PID
            HubLowSpeed = 1;
            mDelayuS(100);
        }
    } else {
        HubLowSpeed = 0;
        SetHostUsbAddr(ThisUsbDev.DeviceAddress); // Set the USB device address currently operated by the USB host
        SetUsbSpeed(ThisUsbDev.DeviceSpeed); // Set the speed of the USB device
    }
}
#endif


/*******************************************************************************
* Function Name  : WaitUSB_Interrupt
* Description    : Wait for USB interrupt
* Input          : None
* Output         : None
* Return         : ERR_SUCCESS data received or sent successfully
                   ERR_USB_UNKNOWN Data reception or transmission failed
*******************************************************************************/
uint8_t WaitUSB_Interrupt(void)
{
    uint16_t  i;
    for (i = WAIT_USB_TOUT_200US; (i != 0) && (UIF_TRANSFER == 0); i --){;}
    return (UIF_TRANSFER ? ERR_SUCCESS : ERR_USB_UNKNOWN);
}


/*******************************************************************************
* Function Name  : USBHostTransact
* Description    : CH554 transmission transaction, input destination endpoint address/PID token, synchronization flag,
*                  total NAK retry time in 20uS units (0 means no retry, 0xFFFF infinite retry), return 0 for success, timeout/error retry
                   This subroutine focuses on being easy to understand.
                   In practical applications, in order to provide running speed, the code of this subroutine should be optimized.
* Input          : uint8_t endp_pid token and address endp_pid: the high 4 bits are the token_pid token, the low 4 bits are the endpoint address
                   uint8_t tog synchronization flag
                   uint16_t timeout timeout time
* Output         : None
* Return         : ERR_USB_UNKNOWN timed out, possibly hardware abnormality
                   ERR_USB_DISCON device disconnected
                   ERR_USB_CONNECT device connection
                   ERR_SUCCESS Transfer completed
*******************************************************************************/
uint8_t USBHostTransact(uint8_t endp_pid, uint8_t tog, uint16_t timeout)
{
#define TransRetry UEP0_T_LEN // Save memory
    uint8_t s, r;
    uint16_t i;
    UH_RX_CTRL = UH_TX_CTRL = tog;
    TransRetry = 0;

    do {
        UH_EP_PID = endp_pid; // Specify token PID and destination endpoint number
        UIF_TRANSFER = 0; // Allow transmission
        for (i = WAIT_USB_TOUT_200US; i != 0 && UIF_TRANSFER == 0; i --);
        UH_EP_PID = 0x00; // Stop USB transfer
        if (UIF_TRANSFER == 0) {
            return (ERR_USB_UNKNOWN);
        }
        if (UIF_DETECT) { // USB device plug and unplug event
            UIF_DETECT = 0; // Clear interrupt flag
            s = AnalyzeRootHub(); // Analyze ROOT-HUB status
            if (s == ERR_USB_CONNECT) {
                FoundNewDev = 1;
            }
#ifdef DISK_BASE_BUF_LEN
            if (CH554DiskStatus == DISK_DISCONNECT) {
                return (ERR_USB_DISCON); // USB device disconnect event
            }
            if (CH554DiskStatus == DISK_CONNECT) {
                return (ERR_USB_CONNECT); // USB device connection event
            }
#else
            if (ThisUsbDev.DeviceStatus == ROOT_DEV_DISCONNECT) {
                return (ERR_USB_DISCON); // USB device disconnect event
            }
            if (ThisUsbDev.DeviceStatus == ROOT_DEV_CONNECTED) {
                return (ERR_USB_CONNECT); // USB device connection event
            }
#endif
            mDelayuS(200); // Wait for transfer to complete
        }
        if (UIF_TRANSFER) { // Transmission completed
            if (U_TOG_OK) {
                return (ERR_SUCCESS);
            }
            r = USB_INT_ST & MASK_UIS_H_RES; // USB device response status
            if (r == USB_PID_STALL) {
                return (r | ERR_USB_TRANSFER);
            }
            if (r == USB_PID_NAK) {
                if (timeout == 0) {
                    return (r | ERR_USB_TRANSFER);
                }
                if (timeout < 0xFFFF) {
                    timeout --;
                }
                TransRetry--;
            } else {
                switch (endp_pid >> 4) {
                    case USB_PID_SETUP:
                    case USB_PID_OUT:
                        if (r) {
                            return (r | ERR_USB_TRANSFER); // Not a timeout/error, unexpected response
                        }
                        break; // Retry after timeout
                    case USB_PID_IN:
                        if ((r == USB_PID_DATA0) && (r == USB_PID_DATA1)) { // If not synchronized, discard and try again
                            // Retry without synchronization
                        } else if (r) {
                            return (r | ERR_USB_TRANSFER); // Not a timeout/error, unexpected response
                        }
                        break; // Retry after timeout
                    default:
                       return (ERR_USB_UNKNOWN); // Impossible situation
                       break;
                }
            }
        } else { // Other interrupts, situations that should not occur
            USB_INT_FG = 0xFF; /* Clear interrupt flag */
        }
        mDelayuS(15);
    } while (++TransRetry < 3);
    return (ERR_USB_TRANSFER); // Response timeout
}


/*******************************************************************************
* Function Name  : HostCtrlTransfer
* Description    : Execute control transmission, the 8-byte request code is in pSetupReq, and DataBuf is an optional transceiver buffer.
* Input          : P__xdata uint8_t DataBuf If you need to receive and send data, then DataBuf needs to point to a valid buffer to store subsequent data.
                   Puint8_t RetLen  The total length of actual successful transmission and reception is stored in the byte variable pointed to by RetLen
* Output         : None
* Return         : ERR_USB_BUF_OVER IN status phase error
                   ERR_SUCCESS Data exchange successful
                   Other error status
*******************************************************************************/
uint8_t HostCtrlTransfer(__xdata uint8_t *DataBuf, uint8_t *RetLen)
{
    uint16_t RemLen = 0;
    uint8_t s, RxLen, RxCnt, TxCnt;
    __xdata uint8_t *pBuf;
    uint8_t *pLen;
    pBuf = DataBuf;
    pLen = RetLen;
    mDelayuS(200);
    if (pLen) {
        *pLen = 0; // The total length of actual successful transmission and reception
    }
    UH_TX_LEN = sizeof(USB_SETUP_REQ);
    s = USBHostTransact((uint8_t)(USB_PID_SETUP << 4 | 0x00), 0x00, 10000); // SETUP phase, 200mS timeout
    if (s != ERR_SUCCESS) {
        return(s);
    }
    UH_RX_CTRL = UH_TX_CTRL = bUH_R_TOG | bUH_R_AUTO_TOG | bUH_T_TOG | bUH_T_AUTO_TOG; // Default DATA1
    UH_TX_LEN = 0x01; // There is no data by default, so the status stage is IN
    RemLen = (pSetupReq -> wLengthH << 8) | (pSetupReq -> wLengthL);
    if (RemLen && pBuf) { // Need to send and receive data
        if (pSetupReq -> bRequestType & USB_REQ_TYP_IN) { // Receive
            while (RemLen) {
                mDelayuS(200);
                s = USBHostTransact((uint8_t)(USB_PID_IN << 4 | 0x00), UH_RX_CTRL, (200000 / 20)); // IN data
                if (s != ERR_SUCCESS) {
                    return(s);
                }
                RxLen = USB_RX_LEN < RemLen ? USB_RX_LEN : RemLen;
                RemLen -= RxLen;
                if (pLen) {
                    *pLen += RxLen; // The total length of actual successful transmission and reception
                }
                for (RxCnt = 0; RxCnt != RxLen; RxCnt ++) {
                    *pBuf = RxBuffer[RxCnt];
                    pBuf++;
                }
                if ((USB_RX_LEN == 0) || (USB_RX_LEN & (UsbDevEndp0Size - 1))) {
                    break; // short packet
                }
            }
            UH_TX_LEN = 0x00; // The status stage is OUT
        } else { // Send
            while (RemLen) {
                mDelayuS(200);
                UH_TX_LEN = (RemLen >= UsbDevEndp0Size) ? UsbDevEndp0Size : RemLen;
#ifndef DISK_BASE_BUF_LEN
                if (pBuf[1] == 0x09) { // HID class command processing
                    Set_Port = Set_Port^1;
                    *pBuf = Set_Port;
#if DE_PRINTF
                    printf ("SET_PORT %02X %02X ", (uint16_t)(*pBuf), (uint16_t)(Set_Port));
#endif
                }
#endif
                for (TxCnt = 0; TxCnt != UH_TX_LEN; TxCnt++) {
                    TxBuffer[TxCnt] = *pBuf;
                    pBuf++;
                }
                s = USBHostTransact(USB_PID_OUT << 4 | 0x00, UH_TX_CTRL, (200000 / 20)); // OUT data
                if (s != ERR_SUCCESS) {
                    return(s);
                }
                RemLen -= UH_TX_LEN;
                if (pLen) {
                    *pLen += UH_TX_LEN; // The total length of actual successful transmission and reception
                }
            }
        }
    }
    mDelayuS(200);
    s = USBHostTransact((UH_TX_LEN ? (USB_PID_IN << 4 | 0x00): (USB_PID_OUT << 4 | 0x00)), (bUH_R_TOG | bUH_T_TOG), (200000 / 20)); // STATUS stage
    if (s != ERR_SUCCESS) {
        return(s);
    }
    if (UH_TX_LEN == 0) {
        return (ERR_SUCCESS); // status OUT
    }
    if (USB_RX_LEN == 0) {
        return (ERR_SUCCESS); // Status IN, check the IN status and return data length
    }
    return (ERR_USB_BUF_OVER); // IN status phase error
}


/*******************************************************************************
* Function Name  : CopySetupReqPkg
* Description    : Copy control transfer request packet
* Input          : P__code uint8_t pReqPkt Control request packet address
* Output         : None
* Return         : None
*******************************************************************************/
void CopySetupReqPkg(__code uint8_t *pReqPkt)                // Copy control transfer request packet
{
    uint8_t   i;
    if (HubLowSpeed) {                                       // Low-speed equipment under HUB
       ((__xdata uint8_t *)pSetupReq)[0] = *pReqPkt;
       for (i = 1; i != sizeof(USB_SETUP_REQ) + 1; i ++) {
           ((__xdata uint8_t *)pSetupReq)[i] = *pReqPkt;
           pReqPkt++;
       }
    }
    if (HubLowSpeed == 0) {
       for (i = 0; i != sizeof(USB_SETUP_REQ); i++) {
           ((__xdata uint8_t *)pSetupReq)[i] = *pReqPkt;
           pReqPkt++;
       }
    }
}


/*******************************************************************************
* Function Name  : CtrlGetDeviceDescr
* Description    : Get the device descriptor and return it in TxBuffer
* Input          : None
* Output         : None
* Return         : ERR_USB_BUF_OVER descriptor length error
                   ERR_SUCCESS Success
                   other
*******************************************************************************/
uint8_t CtrlGetDeviceDescr(void) {
    uint8_t  s;
    uint8_t  len;
    UsbDevEndp0Size = DEFAULT_ENDP0_SIZE;
    CopySetupReqPkg(SetupGetDevDescr);
    s = HostCtrlTransfer(Com_Buffer, (uint8_t *)&len); // Execute control transfer
    if (s != ERR_SUCCESS) {
        return (s);
    }
    // The maximum packet length of endpoint 0. This is a simplified process.
    // Normally, you should first obtain the first 8 bytes and then update UsbDevEndp0Size immediately before continuing.
    UsbDevEndp0Size = ((PXUSB_DEV_DESCR)Com_Buffer) -> bMaxPacketSize0;
    if (len < ((PUSB_SETUP_REQ)SetupGetDevDescr) -> wLengthL) {
        return (ERR_USB_BUF_OVER); // Descriptor length error
    }
    return (ERR_SUCCESS);
}


/*******************************************************************************
* Function Name  : CtrlGetConfigDescr
* Description    : Get the configuration descriptor and return it in TxBuffer
* Input          : None
* Output         : None
* Return         : ERR_USB_BUF_OVER Descriptor length error
                   ERR_SUCCESS Success
                   other
*******************************************************************************/
uint8_t CtrlGetConfigDescr(void)
{
    uint8_t s, len;
    CopySetupReqPkg(SetupGetCfgDescr);
    s = HostCtrlTransfer(Com_Buffer, (uint8_t *)&len); // Execute control transfer
    if (s != ERR_SUCCESS) {
        return(s);
    }

    len = ((PXUSB_CFG_DESCR)Com_Buffer) -> wTotalLengthL;
    CopySetupReqPkg(SetupGetCfgDescr);
    if (HubLowSpeed) { // Low speed device under HUB
        pSetupReq -> wLengthH = len; // The total length of the complete configuration descriptor
    }
    if (HubLowSpeed == 0) {
        pSetupReq -> wLengthL = len; // The total length of the complete configuration descriptor
    }
    s = HostCtrlTransfer(Com_Buffer, (uint8_t *)&len); // Execute control transfer
    if (s != ERR_SUCCESS) {
       return(s);
    }
#ifdef DISK_BASE_BUF_LEN
    if (len > 64) {
        len = 64;
    }
    for (s = 0; s != len; s++) { // When operating on a USB flash drive, it needs to be copied to TxBuffer
        TxBuffer[s] = Com_Buffer[s];
    }
#endif
    return (ERR_SUCCESS);
}


/*******************************************************************************
* Function Name  : CtrlSetUsbAddress
* Description    : Set USB device address
* Input          : uint8_t addr Device addressDevice address
* Output         : None
* Return         : ERR_SUCCESS Success
                   other
*******************************************************************************/
uint8_t CtrlSetUsbAddress(uint8_t addr)
{
    uint8_t s;
    CopySetupReqPkg(SetupSetUsbAddr);
    if (HubLowSpeed) { // Low speed device under HUB
        pSetupReq -> wValueH = addr; // USB device address
    }
    if (HubLowSpeed == 0) {
        pSetupReq -> wValueL = addr; // USB device address
    }
    s = HostCtrlTransfer(NULL, NULL); // Perform control transfer
    if (s != ERR_SUCCESS) {
        return(s);
    }
    SetHostUsbAddr (addr); // Set the USB device address currently operated by the USB host
    mDelaymS(10); // Wait for the USB device to complete the operation
    return (ERR_SUCCESS);
}


/*******************************************************************************
* Function Name  : CtrlSetUsbConfig
* Description    : Set USB device configuration
* Input          : uint8_t cfg      configuration value
* Output         : None
* Return         : ERR_SUCCESS Success
                   other
*******************************************************************************/
uint8_t CtrlSetUsbConfig(uint8_t cfg)
{
    CopySetupReqPkg(SetupSetUsbConfig);
    if (HubLowSpeed) { // Low speed device under HUB
        pSetupReq -> wValueH = cfg; // USB device configuration
    }
    if (HubLowSpeed == 0) {
        pSetupReq -> wValueL = cfg; // USB device configuration
    }
    return (HostCtrlTransfer(NULL, NULL)); // Perform control transfer
}


/*******************************************************************************
* Function Name  : CtrlClearEndpStall
* Description    : Clear endpoint STALL
* Input          : uint8_t endp       endpoint address
* Output         : None
* Return         : ERR_SUCCESS Success
                   other
*******************************************************************************/
uint8_t CtrlClearEndpStall(uint8_t endp)
{
    CopySetupReqPkg(SetupClrEndpStall); // Clear endpoint errors
    if (HubLowSpeed) { // Low speed device under HUB
        pSetupReq -> wIndexH = endp; // Endpoint address
    }
    if (HubLowSpeed == 0) {
        pSetupReq -> wIndexL = endp; // Endpoint address
    }
    return (HostCtrlTransfer(NULL, NULL)); // Perform control transfer
}

#ifndef DISK_BASE_BUF_LEN


/*******************************************************************************
* Function Name  : CtrlSetUsbInterface
* Description    : Set USB device interface
* Input          : uint8_t cfg       configuration value
* Output         : None
* Return         : ERR_SUCCESS Success
                   other
*******************************************************************************/
uint8_t CtrlSetUsbInterface(uint8_t cfg)
{
    CopySetupReqPkg(SetupSetUsbInterface);
    if (HubLowSpeed) { // Low speed device under HUB
        pSetupReq -> wValueH = cfg; // USB device configuration
    }
    if (HubLowSpeed == 0) {
        pSetupReq -> wValueL = cfg; // USB device configuration
    }
    return (HostCtrlTransfer(NULL, NULL)); // Perform control transfer
}


/*******************************************************************************
* Function Name  : CtrlGetHIDDeviceReport
* Description    : Get the HID device report descriptor and return it in TxBuffer
* Input          : None
* Output         : None
* Return         : ERR_SUCCESS Success
                   Other errors
*******************************************************************************/
uint8_t CtrlGetHIDDeviceReport(uint8_t infc)
{
    uint8_t s;
    uint8_t len;

    CopySetupReqPkg(SetupSetHIDIdle);
    if (HubLowSpeed) { // Low speed device under HUB
        TxBuffer[5] = infc;
    } else {
        TxBuffer[4] = infc;
    }
    s = HostCtrlTransfer(Com_Buffer, (uint8_t *)&len); // Execute control transfer
    if (s != ERR_SUCCESS) {
        return(s);
    }

    CopySetupReqPkg(SetupGetHIDDevReport);
    if (HubLowSpeed) { // Low speed device under HUB
        TxBuffer[5] = infc;
    } else {
        TxBuffer[4] = infc;
    }
    s = HostCtrlTransfer(Com_Buffer, (uint8_t *)&len); // Execute control transfer
    if (s != ERR_SUCCESS) {
        return (s);
    }

    return (ERR_SUCCESS);
}


/*******************************************************************************
* Function Name  : CtrlGetHubDescr
* Description    : Get the HUB descriptor and return it in TxBuffer
* Input          : None
* Output         : None
* Return         : ERR_SUCCESS Success
                   ERR_USB_BUF_OVER length error
*******************************************************************************/
uint8_t CtrlGetHubDescr(void)
{
    uint8_t s;
    uint8_t len;
    CopySetupReqPkg(SetupGetHubDescr);
    s = HostCtrlTransfer(Com_Buffer, (uint8_t *)&len);                     // Execute control transfer
    if (s != ERR_SUCCESS) {
        return (s);
    }
    if (len < ((PUSB_SETUP_REQ)SetupGetHubDescr) -> wLengthL) {
        return (ERR_USB_BUF_OVER);                                            // Descriptor length error
    }
    return (ERR_SUCCESS);
}


/*******************************************************************************
* Function Name  : HubGetPortStatus
* Description    : Query the HUB port status and return it in TxBuffer
* Input          : uint8_t HubPortIndex
* Output         : None
* Return         : ERR_SUCCESS Success
                   ERR_USB_BUF_OVER length error
*******************************************************************************/
uint8_t HubGetPortStatus(uint8_t HubPortIndex)
{
    uint8_t   s;
    uint8_t  len;
    pSetupReq -> bRequestType = HUB_GET_PORT_STATUS;
    pSetupReq -> bRequest = HUB_GET_STATUS;
    pSetupReq -> wValueL = 0x00;
    pSetupReq -> wValueH = 0x00;
    pSetupReq -> wIndexL = HubPortIndex;
    pSetupReq -> wIndexH = 0x00;
    pSetupReq -> wLengthL = 0x04;
    pSetupReq -> wLengthH = 0x00;
    s = HostCtrlTransfer(Com_Buffer, (uint8_t *)&len);                         // Execute control transfer
    if (s != ERR_SUCCESS) {
        return (s);
    }
    if (len < 4) {
        return (ERR_USB_BUF_OVER);                                            // Descriptor length error
    }
    return (ERR_SUCCESS);
}


/*******************************************************************************
* Function Name  : HubSetPortFeature
* Description    : Set HUB port characteristics
* Input          : uint8_t HubPortIndex //HUB port
                   uint8_t FeatureSelt //HUB port characteristics
* Output         : None
* Return         : ERR_SUCCESS successful
                   Other errors
*******************************************************************************/
uint8_t HubSetPortFeature(uint8_t HubPortIndex, uint8_t FeatureSelt)
{
    pSetupReq -> bRequestType = HUB_SET_PORT_FEATURE;
    pSetupReq -> bRequest = HUB_SET_FEATURE;
    pSetupReq -> wValueL = FeatureSelt;
    pSetupReq -> wValueH = 0x00;
    pSetupReq -> wIndexL = HubPortIndex;
    pSetupReq -> wIndexH = 0x00;
    pSetupReq -> wLengthL = 0x00;
    pSetupReq -> wLengthH = 0x00;
    return (HostCtrlTransfer(NULL, NULL));                                 // Execute control transfer
}


/*******************************************************************************
* Function Name  : HubClearPortFeature
* Description    : Clear HUB port characteristics
* Input          : uint8_t HubPortIndex // HUB port
                   uint8_t FeatureSelt // HUB port characteristics
* Output         : None
* Return         : ERR_SUCCESS successful
                   Other errors
*******************************************************************************/
uint8_t HubClearPortFeature(uint8_t HubPortIndex, uint8_t FeatureSelt)
{
    pSetupReq -> bRequestType = HUB_CLEAR_PORT_FEATURE;
    pSetupReq -> bRequest = HUB_CLEAR_FEATURE;
    pSetupReq -> wValueL = FeatureSelt;
    pSetupReq -> wValueH = 0x00;
    pSetupReq -> wIndexL = HubPortIndex;
    pSetupReq -> wIndexH = 0x00;
    pSetupReq -> wLengthL = 0x00;
    pSetupReq -> wLengthH = 0x00;
    return (HostCtrlTransfer(NULL, NULL));                                // Execute control transfer
}


/*******************************************************************************
* Function Name  : CtrlGetXPrinterReport1
* Description    : Printer class commands
* Input          : None
* Output         : None
* Return         : ERR_USB_BUF_OVER Descriptor length error
                   ERR_SUCCESS Success
                   other
*******************************************************************************/
/*uint8_t CtrlGetXPrinterReport1(void)
{
    uint8_t  s;
    uint16_t len;
    CopySetupReqPkg(XPrinterReport);
    s = HostCtrlTransfer(Com_Buffer, (uint8_t *)&len);                         // Execute control transfer
    if (s != ERR_SUCCESS) {
        return (s);
    }
    return (ERR_SUCCESS);
}
*/

/*******************************************************************************
* Function Name  : AnalyzeHidIntEndp
* Description    : Analyze the address of the HID interrupt endpoint from the descriptor.
*                  If HubPortIndex is 0, save it to ROOTHUB.
*                  If it is a non-zero value, save it to the structure under HUB.
* Input          : P__xdata uint8_t buf: The address of the data buffer to be analyzed
*                  HubPortIndex: 0 represents the root HUB, non-0 represents the port number under the external HUB
* Output         : None
* Return         : Number of endpoints
*******************************************************************************/
uint8_t AnalyzeHidIntEndp(__xdata uint8_t *buf, uint8_t HubPortIndex)
{
    uint8_t i, s, l;
    s = 0;

    if (HubPortIndex) {
        memset(DevOnHubPort[HubPortIndex-1].GpVar, 0, sizeof(DevOnHubPort[HubPortIndex-1].GpVar)); // Clear the array
    } else {
        memset(ThisUsbDev.GpVar, 0, sizeof(ThisUsbDev.GpVar)); // Clear the array
    }

    // Search for interrupt endpoint descriptor, skip configuration descriptor and interface descriptor
    for (i = 0; i < ((PXUSB_CFG_DESCR)buf) -> wTotalLengthL; i += l) {
        if ((((PXUSB_ENDP_DESCR)(buf + i)) -> bDescriptorType == USB_DESCR_TYP_ENDP) && // is the endpoint descriptor
            ((((PXUSB_ENDP_DESCR)(buf + i)) -> bmAttributes & USB_ENDP_TYPE_MASK) == USB_ENDP_TYPE_INTER) && // is the interrupt endpoint
            ((((PXUSB_ENDP_DESCR)(buf + i)) -> bEndpointAddress & USB_ENDP_DIR_MASK))) { // is the IN endpoint
            // Save the address of the interrupt endpoint, bit 7 is used for synchronization flag bit, clear to 0
            if (HubPortIndex) {
                DevOnHubPort[HubPortIndex - 1].GpVar[s] = ((PXUSB_ENDP_DESCR)(buf + i)) -> bEndpointAddress & USB_ENDP_ADDR_MASK;
            } else {
                // The address of the interrupt endpoint, you can save wMaxPacketSize and bInterval as needed
                ThisUsbDev.GpVar[s] = ((PXUSB_ENDP_DESCR)(buf + i)) -> bEndpointAddress & USB_ENDP_ADDR_MASK;
            }
#if DE_PRINTF
            printf("%02x ", (uint16_t)ThisUsbDev.GpVar[s]);
#endif
            s++;
            if (s >= 4) {
                break; // Only analyze 4 endpoints
            }

        }
        l = ((PXUSB_ENDP_DESCR)(buf + i)) -> bLength; // Current descriptor length, skip
        if (l > 16) {
            break;
        }
    }
#if DE_PRINTF
    printf("\n");
#endif
    return (s);
}


/*******************************************************************************
* Function Name  : AnalyzeBulkEndp
* Description    : The batch endpoints are analyzed, and GpVar[0] and GpVar[1] store the upload endpoints.
*                  GpVar[2], GpVar[3] store download endpoints
* Input          : buf: the address of the data buffer to be analyzed
*                  HubPortIndex: 0 represents the root HUB, non-0 represents the port number under the external HUB
* Output         : None
* Return         : 0
*******************************************************************************/
uint8_t AnalyzeBulkEndp(__xdata uint8_t *buf, uint8_t HubPortIndex)
{
    uint8_t  i, s1, s2, l;
    s1 = 0;
    s2 = 2;

    if (HubPortIndex) {
        memset(DevOnHubPort[HubPortIndex-1].GpVar, 0, sizeof(DevOnHubPort[HubPortIndex-1].GpVar)); // Clear the array
    } else {
        memset(ThisUsbDev.GpVar, 0, sizeof(ThisUsbDev.GpVar)); // Clear the array
    }

    for (i = 0; i < ((PXUSB_CFG_DESCR)buf) -> wTotalLengthL; i += l) {
        // Search for interrupt endpoint descriptor, skip configuration descriptor and interface descriptor
        if ((((PXUSB_ENDP_DESCR)(buf + i)) -> bDescriptorType == USB_DESCR_TYP_ENDP) && // is the endpoint descriptor
            (((((PXUSB_ENDP_DESCR)(buf + i) ) -> bmAttributes & USB_ENDP_TYPE_MASK) == USB_ENDP_TYPE_BULK))) { // Is the interrupt endpoint
            if (HubPortIndex) {
                if (((PXUSB_ENDP_DESCR)(buf + i)) -> bEndpointAddress & USB_ENDP_DIR_MASK) {
                    DevOnHubPort[HubPortIndex - 1].GpVar[s1++] = ((PXUSB_ENDP_DESCR)(buf + i)) -> bEndpointAddress & USB_ENDP_ADDR_MASK;
                } else {
                    DevOnHubPort[HubPortIndex - 1].GpVar[s2++] = ((PXUSB_ENDP_DESCR)(buf + i)) -> bEndpointAddress & USB_ENDP_ADDR_MASK;
                }
            } else {
                if (((PXUSB_ENDP_DESCR)(buf + i)) -> bEndpointAddress & USB_ENDP_DIR_MASK) {
                    ThisUsbDev.GpVar[s1++] = ((PXUSB_ENDP_DESCR)(buf + i)) -> bEndpointAddress & USB_ENDP_ADDR_MASK;
                } else {
                    ThisUsbDev.GpVar[s2++] = ((PXUSB_ENDP_DESCR)(buf + i)) -> bEndpointAddress & USB_ENDP_ADDR_MASK;
                }
            }

            if (s1 == 2) {
                s1 = 1;
            }
            if (s2 == 4) {
                s2 = 3;
            }
        }
        l = ((PXUSB_ENDP_DESCR)(buf + i)) -> bLength; // Current descriptor length, skip
        if (l > 16) {
            break;
        }
    }
    return (0);
}


// Try to start AOA mode
/*uint8_t TouchStartAOA(void)
{
    uint8_t len,s,i,Num;
    uint16_t cp_len;
    // Get the protocol version number
    CopySetupReqPkg(GetProtocol);
    s = HostCtrlTransfer(Com_Buffer, &len); // Execute control transfer
    if (s != ERR_SUCCESS) {
        return(s);
    }
    if (Com_Buffer[0] < 2) {
        return ERR_AOA_PROTOCOL;
    }

    // output string
    for (i = 0; i < 6; i++) {
        Num = Sendlen[i];
        CopySetupReqPkg(&SetStringID[8 * i]);
        cp_len = (pSetupReq -> wLengthH << 8) | (pSetupReq -> wLengthL);
        memcpy(Com_Buffer, &StringID[Num], cp_len);
        s = HostCtrlTransfer(Com_Buffer, &len); // Execute control transfer
        if (s != ERR_SUCCESS) {
            return(s);
        }
     }

    CopySetupReqPkg(TouchAOAMode);
    s = HostCtrlTransfer(Com_Buffer, &len); // Execute control transfer
    if (s != ERR_SUCCESS) {
        return (s);
    }
    return ERR_SUCCESS;
}
*/

/*******************************************************************************
* Function Name  : InitRootDevice
* Description    : Initialize the USB device of the specified ROOT-HUB port
* Input          : uint8_t RootHubIndex specifies the port, built-in HUB port number 0/1
* Output         : None
* Return         :
*******************************************************************************/
uint8_t InitRootDevice(void)
{
    uint8_t t, i, s, cfg, dv_cls, if_cls, ifc;
    uint8_t touchaoatm = 0;
    t = 0;
    s = 0; // correct initial value ???
#if DE_PRINTF
    printf("Reset USB Port\n");
#endif
USBDevEnum:
    for (i = 0; i < t; i++) {
        mDelaymS(100);
        if (t > 10) {
            return (s);
        }
    }
    ResetRootHubPort(); // After detecting the device, reset the USB bus of the corresponding port
    for (i = 0, s = 0; i < 100; i ++) { // Wait for the USB device to reset and then reconnect, 100mS timeout
        mDelaymS(1);
        if (EnableRootHubPort() == ERR_SUCCESS) { // Enable ROOT-HUB port
            i = 0;
            s++; // Wait for the USB device to stabilize after being connected.
            if (s > (20 + t)) {
                break; // The connection has been stable for 15mS
            }
        }
    }
    if (i) { // The device is not connected after reset
        DisableRootHubPort();
#if DE_PRINTF
        printf("Disable usb port because of disconnect\n");
#endif
    }
    SelectHubPort(0);
#if DE_PRINTF
    printf("GetDevDescr: ");
#endif
    s = CtrlGetDeviceDescr(); // Get device descriptor
    if (s == ERR_SUCCESS) {
#if DE_PRINTF
        for (i = 0; i < ((PUSB_SETUP_REQ)SetupGetDevDescr) -> wLengthL; i ++) {
            printf("x%02X ", (uint16_t)(Com_Buffer[i]));
        }
        printf("\n"); // Display descriptor
#endif
        // Save VID PID information
        ThisUsbDev.DeviceVID = (((uint16_t)((PXUSB_DEV_DESCR)Com_Buffer)->idVendorH) << 8) + ((PXUSB_DEV_DESCR)Com_Buffer)->idVendorL;
        ThisUsbDev.DevicePID = (((uint16_t)((PXUSB_DEV_DESCR)Com_Buffer)->idProductH) << 8) + ((PXUSB_DEV_DESCR)Com_Buffer)->idProductL;
        dv_cls = ((PXUSB_DEV_DESCR)Com_Buffer) -> bDeviceClass; // Device class code
        // Set the USB device address, and add RootHubIndex to ensure that the two HUB ports are assigned different addresses
        s = CtrlSetUsbAddress(((PUSB_SETUP_REQ)SetupSetUsbAddr) -> wValueL);
        if (s == ERR_SUCCESS) {
            ThisUsbDev.DeviceAddress = ((PUSB_SETUP_REQ)SetupSetUsbAddr) -> wValueL; // Save USB address
#if DE_PRINTF
            printf("GetCfgDescr: ");
#endif
            s = CtrlGetConfigDescr(); // Get configuration descriptor
            if (s == ERR_SUCCESS) {
                cfg = ((PXUSB_CFG_DESCR)Com_Buffer) -> bConfigurationValue;
                ifc = ((PXUSB_CFG_DESCR)Com_Buffer) -> bNumInterfaces;
#if DE_PRINTF
                for (i = 0; i < ((PXUSB_CFG_DESCR)Com_Buffer) -> wTotalLengthL; i++) {
                    printf("x%02X ", (uint16_t)(Com_Buffer[i]));
                }
                printf("\n");
#endif
                // Analyze the configuration descriptor, obtain endpoint data/address of each endpoint/size of each endpoint, etc.,
                // update variables endp_addr and endp_size, etc.
                if_cls = ((PXUSB_CFG_DESCR_LONG)Com_Buffer) -> itf_descr.bInterfaceClass; // Interface class code
                if ((dv_cls == 0x00) && (if_cls == USB_DEV_CLASS_STORAGE)) { // It is a USB storage device, basically confirmed to be a U disk
                    AnalyzeBulkEndp(Com_Buffer, 0);
#if DE_PRINTF
                    for (i = 0; i != 4; i++) {
                        printf("%02x ", (uint16_t)ThisUsbDev.GpVar[i]);
                    }
                    printf("\n");
#endif
                    s = CtrlSetUsbConfig(cfg); // Set USB device configuration
                    if (s == ERR_SUCCESS) {
                        ThisUsbDev.DeviceStatus = ROOT_DEV_SUCCESS;
                        ThisUsbDev.DeviceType = USB_DEV_CLASS_STORAGE;
#if DE_PRINTF
                        printf("USB-Disk Ready\n");
#endif
                        SetUsbSpeed(1); // Default is full speed
                        return (ERR_SUCCESS);
                    }
                } /*
                else if ((dv_cls == 0x00) && (if_cls == USB_DEV_CLASS_PRINTER) &&
                           (((PXUSB_CFG_DESCR_LONG)Com_Buffer) -> itf_descr.bInterfaceSubClass == 0x01)) { // It is a printer device
#if DE_PRINTF
                    printf("USB-Print OK\n");
#endif
                    if ((Com_Buffer[19] == 5) && (Com_Buffer[20] && 0x80)) {
                        ThisUsbDev.GpVar[0] = Com_Buffer[20]; // IN endpoint
                    } else if ((Com_Buffer[19] == 5) && ((Com_Buffer[20] && 0x80) == 0)) {
                        ThisUsbDev.GpVar[1] = Com_Buffer[20]; // OUT endpoint
                    }
                    if ((Com_Buffer[26] == 5) && (Com_Buffer[20] && 0x80)) {
                        ThisUsbDev.GpVar[0] = Com_Buffer[27]; // IN endpoint
                    } else if ((Com_Buffer[26] == 5) && ((Com_Buffer[20] && 0x80) == 0)) {
                        ThisUsbDev.GpVar[1] = Com_Buffer[27]; // OUT endpoint
                    }
                    s = CtrlSetUsbConfig(cfg); // Set USB device configuration
                    if (s == ERR_SUCCESS) {
                        s = CtrlSetUsbInterface(cfg);
                        s = CtrlGetXPrinterReport1(); // Printer class command
                        if (s == ERR_SUCCESS) {
                            ThisUsbDev.DeviceStatus = ROOT_DEV_SUCCESS;
                            ThisUsbDev.DeviceType = USB_DEV_CLASS_PRINTER;
#if DE_PRINTF
                            printf("USB-Print Ready\n");
#endif
                            SetUsbSpeed(1); // Default is full speed
                            return (ERR_SUCCESS);
                        }
                    }
                }*/
                else if ((dv_cls == 0x00) && (if_cls == USB_DEV_CLASS_HID) &&
                           (((PXUSB_CFG_DESCR_LONG)Com_Buffer) -> itf_descr.bInterfaceSubClass <= 0x01)) { // It is a HID device, keyboard/mouse, etc.
                    s = AnalyzeHidIntEndp(Com_Buffer, 0); // Analyze the address of the HID interrupt endpoint from the descriptor
#if DE_PRINTF
                    printf("AnalyzeHidIntEndp %02x\n", (uint16_t)s);
#endif
                    if_cls = ((PXUSB_CFG_DESCR_LONG)Com_Buffer) -> itf_descr.bInterfaceProtocol;
#if DE_PRINTF
                    printf("CtrlSetUsbConfig %02x\n", (uint16_t)cfg);
#endif
                    s = CtrlSetUsbConfig(cfg); // Set USB device configuration
                    if (s == ERR_SUCCESS) {
#if DE_PRINTF
                        printf("GetHIDReport: ");
#endif
                        for (dv_cls = 0; dv_cls < ifc; dv_cls++) {
                            s = CtrlGetHIDDeviceReport(dv_cls); // Get the report descriptor
                            if (s == ERR_SUCCESS) {
#if DE_PRINTF
                                for (i = 0; i < 64; i++) {
                                    printf("x%02X ", (uint16_t)(Com_Buffer[i]));
                                }
                                printf("\n");
#endif
                            }
                        }
                        // Endpoint information needs to be saved so that the main program can perform USB transmission
                        ThisUsbDev.DeviceStatus = ROOT_DEV_SUCCESS;
                        if (if_cls == 1) {
                            ThisUsbDev.DeviceType = DEV_TYPE_KEYBOARD;
                            // Further initialization, such as device keyboard indicator LED, etc.
                            if (ifc > 1) {
#if DE_PRINTF
                                printf("USB_DEV_CLASS_HID Ready\n");
#endif
                                ThisUsbDev.DeviceType = USB_DEV_CLASS_HID; // Composite HID device
                            }
#if DE_PRINTF
                            printf("USB-Keyboard Ready\n");
#endif
                            SetUsbSpeed(1); // Default is full speed

                            return (ERR_SUCCESS);
                        } else if (if_cls == 2) {
                            ThisUsbDev.DeviceType = DEV_TYPE_MOUSE;
                            // In order to query the mouse status in the future, the descriptor should be analyzed to obtain the address,
                            // length and other information of the interrupt port.
                            if (ifc > 1) {
#if DE_PRINTF
                                printf("USB_DEV_CLASS_HID Ready\n");
#endif
                                ThisUsbDev.DeviceType = USB_DEV_CLASS_HID; // Composite HID device
                            }
#if DE_PRINTF
                            printf("USB-Mouse Ready\n");
#endif
                            SetUsbSpeed(1); // Default is full speed

                            return (ERR_SUCCESS);
                        }
                        s = ERR_USB_UNSUPPORT;
                    }
                 } /*
                 else if (dv_cls == USB_DEV_CLASS_HUB) { // It is a HUB type device, hub, etc.
                    s = AnalyzeHidIntEndp(Com_Buffer, 0); // Analyze the address of the HID interrupt endpoint from the descriptor
#if DE_PRINTF
                    printf("AnalyzeHidIntEndp %02x\n", (uint16_t)s);
#endif
#if DE_PRINTF
                    printf("GetHubDescr: ");
#endif
                    s = CtrlGetHubDescr();
                    if (s == ERR_SUCCESS) {
#if DE_PRINTF
                        for (i = 0; i < Com_Buffer[0]; i++) {
                            printf("x%02X ", (uint16_t)(Com_Buffer[i]));
                        }
                        printf("\n");
#endif
                        ThisUsbDev.GpHUBPortNum = ((PXUSB_HUB_DESCR)Com_Buffer ) -> bNbrPorts; // Save the number of HUB ports
                        if (ThisUsbDev.GpHUBPortNum > HUB_MAX_PORTS) {
                            // Because when defining the structure DevOnHubPort, it is artificially assumed that each
                            // HUB does not exceed HUB_MAX_PORTS ports.
                            ThisUsbDev.GpHUBPortNum = HUB_MAX_PORTS;
                        }
                        s = CtrlSetUsbConfig(cfg); // Set USB device configuration
                        if (s == ERR_SUCCESS) {
                            ThisUsbDev.DeviceStatus = ROOT_DEV_SUCCESS;
                            ThisUsbDev.DeviceType = USB_DEV_CLASS_HUB;
                            // Endpoint information needs to be saved so that the main program can perform USB transmission.
                            // Originally, the interrupt endpoint can be used for HUB event notification,
                            // but this program uses query status control transmission instead.
                            // Power on each port of the HUB, query the status of each port,
                            // initialize the HUB port to which the device is connected, and initialize the device.
                            for (i = 1; i <= ThisUsbDev.GpHUBPortNum; i++) { // Power on each HUB port
                                DevOnHubPort[i-1].DeviceStatus = ROOT_DEV_DISCONNECT; // Clear the status of the device on the external HUB port
                                s = HubSetPortFeature(i, HUB_PORT_POWER);
                                if (s != ERR_SUCCESS) {
#if DE_PRINTF
                                    printf("Ext-HUB Port_%1d# power on error\n", (uint16_t)i); // Port power on failed
#endif
                                }
                            }
                            SetUsbSpeed (1); // Default is full speed
                            return (ERR_SUCCESS);
                        }
                    }
                } */
                else { // Other devices
#if DE_PRINTF
                    printf("dv_cls %02x\n", (uint16_t)dv_cls);
                    printf("if_cls %02x\n", (uint16_t)if_cls);
                    printf("if_subcls %02x\n", (uint16_t)((PXUSB_CFG_DESCR_LONG)Com_Buffer) -> itf_descr.bInterfaceSubClass);
#endif
                    AnalyzeBulkEndp(Com_Buffer, 0); // Analyze the batch endpoint
#if DE_PRINTF
                    for (i = 0; i != 4; i++) {
                        printf("%02x ", (uint16_t)ThisUsbDev.GpVar[i]);
                    }
                    printf("\n");
#endif
                    s = CtrlSetUsbConfig(cfg); // Set USB device configuration
                    if (s == ERR_SUCCESS) {
#if DE_PRINTF
                        printf("%02x %02x\n", (uint16_t)ThisUsbDev.DeviceVID, (uint16_t)ThisUsbDev.DevicePID);
#endif
                        /*
                        if ((ThisUsbDev.DeviceVID == 0x18D1) && ((ThisUsbDev.DevicePID & 0xff00) == 0x2D00)) { // If it is an AOA accessory
                            printf("AOA Mode\n");
                            ThisUsbDev.DeviceStatus = ROOT_DEV_SUCCESS;
                            ThisUsbDev.DeviceType = DEF_AOA_DEVICE; // This is just a custom variable class and does not belong to the USB protocol class
                            SetUsbSpeed(1); // Default is full speed
                            return (ERR_SUCCESS);
                        } else { // If it is not AOA accessory mode, try to activate accessory mode.
                            s = TouchStartAOA();
                            if (s == ERR_SUCCESS) {
                                if (touchaoatm < 3) { // Limit the number of AOA startup attempts
                                    touchaoatm++;
                                    mDelaymS(500); // Some Android devices automatically disconnect and reconnect, so it is best to have a delay here
                                    // In fact, there is no need to jump here. The AOA protocol stipulates that the device will automatically reconnect to the bus.
                                    goto USBDevEnum;
                                }
                                // Execution to this point indicates that AOA may not be supported or other devices may be used.
                                ThisUsbDev.DeviceType = dv_cls ? dv_cls : if_cls;
                                ThisUsbDev.DeviceStatus = ROOT_DEV_SUCCESS;
                                SetUsbSpeed(1); // Default is full speed
                                return (ERR_SUCCESS); // Unknown device initialized successfully
                            }
                        }
                        */
                    }
                }
            }
        }
    }
#if DE_PRINTF
    printf("InitRootDev Err = %02X\n", (uint16_t)s);
#endif
    ThisUsbDev.DeviceStatus = ROOT_DEV_FAILED;
    SetUsbSpeed(1); // Default is full speed
    t++;
    goto USBDevEnum;
}


/*******************************************************************************
* Function Name  : EnumAllRootDevice
* Description    : Enumerate USB devices of all ROOT-HUB ports
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t EnumAllRootDevice(void)
{
    uint8_t s;
#if DE_PRINTF
    printf("EnumUSBDev\n");
#endif
    if (ThisUsbDev.DeviceStatus == ROOT_DEV_CONNECTED) {  // The device has just been inserted and has not yet been initialized.
         s = InitRootDevice(); // Initialize/enumerate the USB devices of the specified HUB port
        if (s != ERR_SUCCESS) {
            return s;
        }
    }
    return ERR_SUCCESS;
}


/*******************************************************************************
* Function Name  : InitDevOnHub
* Description    : Initialize the secondary USB device after enumerating the external HUB
* Input          : uint8_t HubPortIndex specifies the external HUB
* Output         : None
* Return         : ERR_SUCCESS successful
                   ERR_USB_UNKNOWN unknown device
*******************************************************************************/
/*
uint8_t InitDevOnHub(uint8_t HubPortIndex)
{
    uint8_t s, cfg, dv_cls, if_cls;
    uint8_t ifc;
#if DE_PRINTF
    printf("Init dev @ExtHub-port_%1d ", (uint16_t)HubPortIndex);
#endif
    if (HubPortIndex == 0) {
        return (ERR_USB_UNKNOWN);
    }
    SelectHubPort(HubPortIndex); // Select the specified port of the external HUB operating the specified ROOT-HUB port, and select the speed
#if DE_PRINTF
    printf("GetDevDescr: ");
#endif
    s = CtrlGetDeviceDescr(); // Get device descriptor
    if (s != ERR_SUCCESS) {
        return(s);
    }
    // Save VID PID information
    DevOnHubPort[HubPortIndex - 1].DeviceVID = (((uint16_t)((PXUSB_DEV_DESCR)Com_Buffer)->idVendorH) << 8) + ((PXUSB_DEV_DESCR)Com_Buffer)->idVendorL;
    DevOnHubPort[HubPortIndex - 1].DevicePID = (((uint16_t)((PXUSB_DEV_DESCR)Com_Buffer)->idProductH) << 8) + ((PXUSB_DEV_DESCR)Com_Buffer)->idProductL;

    dv_cls = ((PXUSB_DEV_DESCR)Com_Buffer) -> bDeviceClass; // Device class code
    cfg = (1 << 4) + HubPortIndex; // Calculate a USB address to avoid address overlap
    s = CtrlSetUsbAddress(cfg); //Set USB device address
    if (s != ERR_SUCCESS) {
        return(s);
    }
    DevOnHubPort[HubPortIndex - 1].DeviceAddress = cfg; // Save the assigned USB address
#if DE_PRINTF
    printf("GetCfgDescr: ");
#endif
    s = CtrlGetConfigDescr(); // Get the configuration descriptor
    if (s != ERR_SUCCESS) {
        return(s);
    }
    cfg = ((PXUSB_CFG_DESCR)Com_Buffer) -> bConfigurationValue;
#if DE_PRINTF
    for (uint8_t i = 0; i < ((PXUSB_CFG_DESCR)Com_Buffer) -> wTotalLengthL; i ++) {
        printf("x%02X ", (uint16_t)(Com_Buffer[i]));
    }
    printf("\n");
#endif
    // Analyze the configuration descriptor, obtain endpoint data/each endpoint address/each endpoint size, etc.,
    //   update variables endp_addr and endp_size, etc.
    if_cls = ((PXUSB_CFG_DESCR_LONG)Com_Buffer) -> itf_descr.bInterfaceClass; // Interface class code
    if ((dv_cls == 0x00) && (if_cls == USB_DEV_CLASS_STORAGE)) { // It is a USB storage device, basically confirmed to be a U disk
        AnalyzeBulkEndp(Com_Buffer, HubPortIndex);
#if DE_PRINTF
        for (uint8_t i = 0; i != 4; i++) {
            printf("%02x ", (uint16_t)DevOnHubPort[HubPortIndex - 1].GpVar[i]);
        }
        printf("\n");
#endif
        s = CtrlSetUsbConfig(cfg); // Set USB device configuration
        if (s == ERR_SUCCESS) {
            DevOnHubPort[HubPortIndex - 1].DeviceStatus = ROOT_DEV_SUCCESS;
            DevOnHubPort[HubPortIndex - 1].DeviceType = USB_DEV_CLASS_STORAGE;
#if DE_PRINTF
            printf("USB-Disk Ready\n");
#endif
            SetUsbSpeed(1); // Default is full speed
            return (ERR_SUCCESS);
        }
    } else if ((dv_cls == 0x00) && (if_cls == USB_DEV_CLASS_HID) &&
               (((PXUSB_CFG_DESCR_LONG)Com_Buffer) -> itf_descr.bInterfaceSubClass <= 0x01)) { // It is a HID device, keyboard/mouse, etc.
        ifc = ((PXUSB_CFG_DESCR_LONG)Com_Buffer) -> cfg_descr.bNumInterfaces;
        s = AnalyzeHidIntEndp(Com_Buffer, HubPortIndex); // Analyze the address of the HID interrupt endpoint from the descriptor
#if DE_PRINTF
        printf("AnalyzeHidIntEndp %02x\n", (uint16_t)s);
#endif
        if_cls = ((PXUSB_CFG_DESCR_LONG)Com_Buffer) -> itf_descr.bInterfaceProtocol;
        s = CtrlSetUsbConfig(cfg); // Set USB device configuration
        if (s == ERR_SUCCESS) {
            for (dv_cls = 0; dv_cls < ifc; dv_cls++) {
                s = CtrlGetHIDDeviceReport(dv_cls); // Get the report descriptor
                if (s == ERR_SUCCESS) {
#if DE_PRINTF
                    for (int i = 0; i < 64; i++) {
                        printf("x%02X ", (uint16_t)(Com_Buffer[i]));
                    }
                    printf("\n");
#endif
                }
            }
            // Endpoint information needs to be saved so that the main program can perform USB transmission
            DevOnHubPort[HubPortIndex - 1].DeviceStatus = ROOT_DEV_SUCCESS;
            if (if_cls == 1) {
                DevOnHubPort[HubPortIndex-1].DeviceType = DEV_TYPE_KEYBOARD;
                // Further initialization, such as device keyboard indicator LED, etc.
                if (ifc > 1) {
#if DE_PRINTF
                    printf("USB_DEV_CLASS_HID Ready\n");
#endif
                    DevOnHubPort[HubPortIndex - 1].DeviceType = USB_DEV_CLASS_HID; // Composite HID device
                }
#if DE_PRINTF
                printf("USB-Keyboard Ready\n");
#endif
                SetUsbSpeed(1); // Default is full speed

                return (ERR_SUCCESS);
            } else if (if_cls == 2) {
                DevOnHubPort[HubPortIndex - 1].DeviceType = DEV_TYPE_MOUSE;
                // In order to query the mouse status in the future, the descriptor should be analyzed to obtain
                // the address, length and other information of the interrupt port.
                if (ifc > 1) {
#if DE_PRINTF
                    printf("USB_DEV_CLASS_HID Ready\n");
#endif
                    DevOnHubPort[HubPortIndex - 1].DeviceType = USB_DEV_CLASS_HID; // Composite HID device
                }
#if DE_PRINTF
                printf("USB-Mouse Ready\n");
#endif
                SetUsbSpeed(1); // Default is full speed

                return (ERR_SUCCESS);
            }
            s = ERR_USB_UNSUPPORT;
        }
    } else if (dv_cls == USB_DEV_CLASS_HUB) { // It is a HUB type device, hub, etc.
        DevOnHubPort[HubPortIndex - 1].DeviceType = USB_DEV_CLASS_HUB;
#if DE_PRINTF
        printf("This program don't support Level 2 HUB\n"); // If you need to support multi-level HUB cascade, please refer to this program for expansion.
#endif
        s = HubClearPortFeature(HubPortIndex, HUB_PORT_ENABLE); // Disable HUB port
        if (s != ERR_SUCCESS) {
            return(s);
        }
        s = ERR_USB_UNSUPPORT;
     } else { // Other devices
        AnalyzeBulkEndp(Com_Buffer, HubPortIndex); // Analyze the batch endpoint
#if DE_PRINTF
        for (int i = 0; i != 4; i++) {
            printf("%02x ", (uint16_t)DevOnHubPort[HubPortIndex - 1].GpVar[i]);
        }
        printf("\n");
#endif
        s = CtrlSetUsbConfig(cfg); // Set USB device configuration
        if (s == ERR_SUCCESS) {
            // Endpoint information needs to be saved so that the main program can perform USB transmission
            DevOnHubPort[HubPortIndex - 1].DeviceStatus = ROOT_DEV_SUCCESS;
            DevOnHubPort[HubPortIndex - 1].DeviceType = dv_cls ? dv_cls : if_cls;
            SetUsbSpeed(1); // Default is full speed
            return (ERR_SUCCESS); // Unknown device initialized successfully
        }
    }
#if DE_PRINTF
    printf("InitDevOnHub Err = %02X\n", (uint16_t)s);
#endif
    DevOnHubPort[HubPortIndex - 1].DeviceStatus = ROOT_DEV_FAILED;
    SetUsbSpeed(1); // Default is full speed
    return s;
}*/


/*******************************************************************************
* Function Name  : EnumHubPort
* Description    : Enumerate each port of the external HUB hub on the specified ROOT-HUB port,
*                  check whether there is a connection or removal event on each port, and initialize the secondary USB device
* Input          : uint8_t RootHubIndex ROOT_HUB0 and ROOT_HUB1
* Output         : None
* Return         : ERR_SUCCESS successful
                   Other failed
*******************************************************************************/
/*uint8_t EnumHubPort()
{
    uint8_t i, s;

    for (i = 1; i <= ThisUsbDev.GpHUBPortNum; i++) { // Query whether the hub port has changed
        SelectHubPort(0); // Select the ROOT-HUB port specified by the operation, set the current USB speed and the USB address of the operated device
        s = HubGetPortStatus(i); // Get port status
        if (s != ERR_SUCCESS) {
            return (s); // Maybe the HUB is disconnected
        }
        if (((Com_Buffer[0] & (1 << (HUB_PORT_CONNECTION & 0x07))) && (Com_Buffer[2] & (1 << (HUB_C_PORT_CONNECTION & 0x07)))) ||
            (Com_Buffer[2] == 0x10)) {
            // Found a device connected
            DevOnHubPort[i - 1].DeviceStatus = ROOT_DEV_CONNECTED; // There is a device connected
            DevOnHubPort[i - 1].DeviceAddress = 0x00;
            s = HubGetPortStatus(i); // Get port status
            if (s != ERR_SUCCESS) {
                return (s); // Maybe the HUB is disconnected
            }
            DevOnHubPort[i - 1].DeviceSpeed = Com_Buffer[1] & (1 << (HUB_PORT_LOW_SPEED & 0x07)) ? 0 : 1; // Low speed or full speed
            if (DevOnHubPort[i-1].DeviceSpeed) {
#if DE_PRINTF
                printf("Found full speed device on port %1d\n", (uint16_t)i);
#endif
            } else {
#if DE_PRINTF
                printf("Found low speed device on port %1d\n", (uint16_t)i);
#endif
            }
            mDelaymS(200); // Wait for the device to power on and stabilize
            s = HubSetPortFeature(i, HUB_PORT_RESET); // Reset the port where the device is connected
            if (s != ERR_SUCCESS) {
                return (s); // Maybe the HUB is disconnected
            }
#if DE_PRINTF
            printf("Reset port and then wait in\n");
#endif
            do { // Query the reset port until the reset is completed and display the completed status
                mDelaymS(1);
                s = HubGetPortStatus(i);
                if (s != ERR_SUCCESS) {
                    return (s); // Maybe the HUB is disconnected
                }
            } while (Com_Buffer[0] & (1 << (HUB_PORT_RESET & 0x07))); // Wait while the port is being reset
            mDelaymS(100);
            s = HubClearPortFeature(i, HUB_C_PORT_RESET); // Clear reset completion flag
            s = HubClearPortFeature(i, HUB_C_PORT_CONNECTION); // Clear the connection or remove the change flag
            if (s != ERR_SUCCESS) {
                return(s);
            }
            s = HubGetPortStatus(i); // Read the status again and check whether the device is still there
            if (s != ERR_SUCCESS) {
                return(s);
            }
            if ((Com_Buffer[0] & (1 << (HUB_PORT_CONNECTION & 0x07))) == 0) {
                DevOnHubPort[i-1].DeviceStatus = ROOT_DEV_DISCONNECT; // The device is no longer there
            }
            s = InitDevOnHub(i); // Initialize secondary USB device
            if (s != ERR_SUCCESS) {
                return(s);
            }
            SetUsbSpeed(1); // Default is full speed
        } else if (Com_Buffer[2] & (1 << (HUB_C_PORT_ENABLE & 0x07))) { // Device connection error
            HubClearPortFeature(i, HUB_C_PORT_ENABLE); // Clear connection error flag
#if DE_PRINTF
            printf("Device on port error\n");
#endif
            s = HubSetPortFeature(i, HUB_PORT_RESET); // Reset the port where the device is connected
            if (s != ERR_SUCCESS) {
                return (s); // Maybe the HUB is disconnected
            }

            do { // Query the reset port until the reset is completed and display the completed status
                mDelaymS(1);
                s = HubGetPortStatus(i);
                if (s != ERR_SUCCESS) {
                    return (s); // Maybe the HUB is disconnected
                }
            } while (Com_Buffer[0] & (1 << (HUB_PORT_RESET & 0x07))); // Wait while the port is being reset
        } else if ((Com_Buffer[0] & (1 << (HUB_PORT_CONNECTION & 0x07))) == 0) { // The device has been disconnected
            if (DevOnHubPort[i - 1].DeviceStatus >= ROOT_DEV_CONNECTED) {
#if DE_PRINTF
                printf("Device on port %1d removed\n", (uint16_t)i);
#endif
            }
            DevOnHubPort[i - 1].DeviceStatus = ROOT_DEV_DISCONNECT; // There is a device connected
            if (Com_Buffer[2] & (1 << (HUB_C_PORT_CONNECTION & 0x07))) {
                HubClearPortFeature(i, HUB_C_PORT_CONNECTION); // Clear removal change flag
            }
        }
    }
    return (ERR_SUCCESS); // Return operation successful
}
*/

/*******************************************************************************
* Function Name  : EnumAllHubPort
* Description    : Enumerate secondary USB devices behind external HUB under all ROOT-HUB ports
* Input          : None
* Output         : None
* Return         : ERR_SUCCESS successful
                   Other failed
*******************************************************************************/
/*uint8_t EnumAllHubPort(void)
{
    uint8_t s;

    if ((ThisUsbDev.DeviceStatus >= ROOT_DEV_SUCCESS) && (ThisUsbDev.DeviceType == USB_DEV_CLASS_HUB)) { // HUB enumeration successful
        SelectHubPort(0); // Select the ROOT-HUB port specified by the operation, set the current USB speed and the USB address of the operated device
        // Enumerate each port of the external HUB hub on the specified ROOT-HUB port, and check whether there is a connection or removal event on each port.
        s = EnumHubPort();
        if (s != ERR_SUCCESS) { // Maybe the HUB is disconnected
#if DE_PRINTF
            printf("EnumAllHubPort err = %02X\n", (uint16_t)s);
#endif
        }
        SetUsbSpeed(1); // Default is full speed
    }
    return (ERR_SUCCESS);
}*/


/*******************************************************************************
* Function Name  : SearchTypeDevice
* Description    : Search the port number of the specified type of device on each port of ROOT-HUB and external HUB.
*                  If the output port number is 0xFFFF, it will not be found.
* Input          : uint8_t type device type to search for
* Output         : None
* Return         : The high 8 bits of the output are the ROOT-HUB port number,
*                  the low 8 bits are the port number of the external HUB,
*                  and if the low 8 bits are 0, the device is directly on the ROOT-HUB port.
                   Of course, you can also search based on the USB manufacturer's VID product PID
                   (record the VID and PID of each device in advance), and specify the search serial number.
*******************************************************************************/
uint16_t SearchTypeDevice(uint8_t type)
{
    uint8_t RootHubIndex; // CH554 has only one USB port, RootHubIndex = 0, just look at the lower eight bits of the return value.
    uint8_t HubPortIndex;

    RootHubIndex = 0;
    if ((ThisUsbDev.DeviceType == USB_DEV_CLASS_HUB) && (ThisUsbDev.DeviceStatus >= ROOT_DEV_SUCCESS)) { // External hub HUB and enumeration is successful
        for (HubPortIndex = 1; HubPortIndex <= ThisUsbDev.GpHUBPortNum; HubPortIndex ++) { // Search for each port of the external HUB
            if ((DevOnHubPort[HubPortIndex - 1].DeviceType == type) && (DevOnHubPort[HubPortIndex - 1].DeviceStatus >= ROOT_DEV_SUCCESS)) {
                return (((uint16_t)RootHubIndex << 8) | HubPortIndex); // The type matches and the enumeration is successful
            }
        }
    }
    if ((ThisUsbDev.DeviceType == type) && (ThisUsbDev.DeviceStatus >= ROOT_DEV_SUCCESS)) {
        return ((uint16_t)RootHubIndex << 8); // The type matches and the enumeration is successful, on the ROOT-HUB port
    }

    return (0xFFFF);
}


/*******************************************************************************
* Function Name  : SETorOFFNumLock
* Description    : NumLock lighting judgment
* Input          : Puint8_t buf lighting key value
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t SETorOFFNumLock(uint8_t *buf)
{
    uint8_t tmp[]= {0x21,0x09,0x00,0x02,0x00,0x00,0x01,0x00};
    uint8_t len,s;
    if ((buf[2] == 0x53) & (buf[0] | buf[1] | buf[3] | buf[4] | buf[5] | buf[6] | buf[7] == 0)) {
        if (HubLowSpeed) { // Low speed device under HUB
            ((__xdata uint8_t *)pSetupReq)[0] = 0x21;
            for (s = 1; s != sizeof(tmp) + 1; s++) {
                ((__xdata uint8_t *)pSetupReq)[s] = tmp[s];
            }
        }
        if (HubLowSpeed == 0) {
            for (s = 0; s != sizeof( tmp ); s++) {
                ((__xdata uint8_t *)pSetupReq)[s] = tmp[s];
            }
        }
        s = HostCtrlTransfer(Com_Buffer, &len); // Execute control transfer
        if (s != ERR_SUCCESS) {
            return(s);
        }
    }
    return (ERR_SUCCESS);
}
#endif


#ifdef DISK_BASE_BUF_LEN
uint8_t InitRootDevice(void) // Initialize USB device
{
    uint8_t i, s, cfg, dv_cls, if_cls;
#if DE_PRINTF
    printf("Reset host port\n");
#endif
    ResetRootHubPort(); // After detecting the device, reset the USB bus of the corresponding port
    for (i = 0, s = 0; i < 100; i++) { // Wait for the USB device to reset and then reconnect, 100mS timeout
        mDelaymS(1);
        if (EnableRootHubPort() == ERR_SUCCESS) { // Enable port
            i = 0;
            s ++; // Wait for the USB device to stabilize after being connected
            if (s > 100) {
                break; // The connection has been stable for 100mS
            }
        }
    }
    if (i) { //The device is not connected after reset
        DisableRootHubPort( );
#if DE_PRINTF
        printf("Disable host port because of disconnect\n");
#endif
        return (ERR_USB_DISCON);
    }
    SetUsbSpeed(1); // Set the current USB speed
    s = CtrlGetDeviceDescr(); // Get device descriptor
    if (s == ERR_SUCCESS) {
#if DE_PRINTF
        printf("GetDevDescr: ");
        for (i = 0; i < ((PUSB_SETUP_REQ)SetupGetDevDescr) -> wLengthL; i++) {
            printf("x%02X ", (uint16_t)(Com_Buffer[i]));
        }
        printf("\n"); // Display descriptor
#endif
        dv_cls = ((PXUSB_DEV_DESCR)Com_Buffer) -> bDeviceClass; // Device class code
        s = CtrlSetUsbAddress(((PUSB_SETUP_REQ)SetupSetUsbAddr) -> wValueL); // Set the USB device address
        if (s == ERR_SUCCESS) {
            s = CtrlGetConfigDescr(); // Get the configuration descriptor
            if (s == ERR_SUCCESS) {
                cfg = ((PXUSB_CFG_DESCR)Com_Buffer) -> bConfigurationValue;
#if DE_PRINTF
                printf("GetCfgDescr: ");
                for (i = 0; i < ((PXUSB_CFG_DESCR)Com_Buffer) -> wTotalLengthL; i++) {
                    printf("x%02X ", (uint16_t)(Com_Buffer[i]));
                }
                printf("\n");
#endif
                /* Analyze the configuration descriptor, obtain endpoint data/each endpoint address/each endpoint size, etc.,
                   update variables endp_addr and endp_size, etc. */
                if_cls = ((PXUSB_CFG_DESCR_LONG)Com_Buffer) -> itf_descr.bInterfaceClass; // Interface class code
                if ((dv_cls == 0x00) && (if_cls == USB_DEV_CLASS_STORAGE)) { // It is a USB storage device, basically confirmed to be a U disk
                    CH554DiskStatus = DISK_USB_ADDR;
                    return (ERR_SUCCESS);
                } else {
                    return (ERR_USB_UNSUPPORT);
                }
            }
        }
    }
#if DE_PRINTF
    printf("InitRootDev Err = %02X\n", (uint16_t)s);
#endif
    CH554DiskStatus = DISK_CONNECT;
    SetUsbSpeed(1); // Default is full speed
    return (s);
}
#endif


/************************************************************************************
* Function Name  : InitUSB_Host
* Description    : Initialize USB host
* Input          : None
* Output         : None
* Return         : None
**********************************************************************************/
void InitUSB_Host(void)
{
    uint8_t i;
    IE_USB = 0;
    USB_CTRL = bUC_HOST_MODE; // Set the mode first
    UHOST_CTRL &= ~bUH_PD_DIS; // Enable host drop-down
    USB_DEV_AD = 0x00;
    UH_EP_MOD = bUH_EP_TX_EN | bUH_EP_RX_EN;
    UH_RX_DMA = (uint16_t)RxBuffer;
    UH_TX_DMA = (uint16_t)TxBuffer;
    UH_RX_CTRL = 0x00;
    UH_TX_CTRL = 0x00;
    USB_CTRL = bUC_HOST_MODE | bUC_INT_BUSY; // Start the USB host and DMA, and automatically pause before the interrupt flag is cleared
    USB_INT_FG = 0xFF; // Clear interrupt flag
    for (i = 0; i != 2; i++) {
        DisableRootHubPort(); // Clear
    }
    USB_INT_EN = bUIE_TRANSFER | bUIE_DETECT;
}

