

// Each subroutine returns status code
#define ERR_SUCCESS         0x00    // Successful operation
#define ERR_USB_CONNECT     0x15    /* USB device connection event detected, connected */
#define ERR_USB_DISCON      0x16    /* USB device disconnection event detected and has been disconnected */
#define ERR_USB_BUF_OVER    0x17    /* The data transferred by USB is incorrect or there is too much data and the buffer overflows. */
#define ERR_USB_DISK_ERR    0x1F    /* The USB memory operation failed. The USB memory may not be supported during initialization.
                                       The disk may be damaged or disconnected during read and write operations. */
#define ERR_USB_TRANSFER    0x20    /* NAK/STALL and other error codes are in 0x20~0x2F */
#define ERR_USB_UNSUPPORT   0xFB    /* Unsupported USB device */
#define ERR_USB_UNKNOWN     0xFE    /* Device operation error */
#define ERR_AOA_PROTOCOL    0x41    /* Protocol version error */

/* USB device related information table, CH554 supports up to 1 device */
#define ROOT_DEV_DISCONNECT  0
#define ROOT_DEV_CONNECTED   1
#define ROOT_DEV_FAILED      2
#define ROOT_DEV_SUCCESS     3
#define DEV_TYPE_KEYBOARD   ( USB_DEV_CLASS_HID | 0x20 )
#define DEV_TYPE_MOUSE      ( USB_DEV_CLASS_HID | 0x30 )
#define DEF_AOA_DEVICE       0xF0


/*
Convention: USB device address allocation rules (refer to USB_DEVICE_ADDR)
Address   Device location
0x02      USB device or external HUB under built-in Root-HUB
0x1x      USB device under port x of the external HUB under the built-in Root-HUB, x is 1~n
*/
#define HUB_MAX_PORTS       4
#define WAIT_USB_TOUT_200US     400   // USB device under port x of the external HUB under the built-in Root-HUB,
                                      // x is 1~n, waiting for USB interrupt timeout 200uS@Fsys=12MHz

/* Array size definition */
#define COM_BUF_SIZE            120   // Can be dynamically modified to save memory based on the maximum descriptor size.

extern __code uint8_t  SetupGetDevDescr[];     //* Get device descriptor */
extern __code uint8_t  SetupGetCfgDescr[];     //* Get configuration descriptor */
extern __code uint8_t  SetupSetUsbAddr[];      //* Set USB address */
extern __code uint8_t  SetupSetUsbConfig[];    //* Set USB configuration */
extern __code uint8_t  SetupSetUsbInterface[]; //* Set USB interface configuration */
extern __code uint8_t  SetupClrEndpStall[];    //* Clear endpoint STALL */
#ifndef DISK_BASE_BUF_LEN
extern __code uint8_t  SetupGetHubDescr[];     //* Get HUB descriptor */
extern __code uint8_t  SetupSetHIDIdle[];
extern __code uint8_t  SetupGetHIDDevReport[]; //* Get HID device report descriptor */
extern __code uint8_t  XPrinterReport[];       //* Printer class commands */
#endif
extern __xdata uint8_t  UsbDevEndp0Size;       //* Maximum packet size for USB device endpoint 0 */

extern __code uint8_t  GetProtocol[];         // AOA gets protocol version
extern __code uint8_t TouchAOAMode[];         // Start accessory mode
extern __code uint8_t Sendlen[];              /* AOA related array definition */
extern __code uint8_t StringID[];             // String ID, string information related to mobile APP
extern __code uint8_t SetStringID[];          // Apply index string command

#ifndef DISK_BASE_BUF_LEN
typedef struct
{
    uint8_t DeviceStatus;  // Device status,
                           // 0-no device,
                           // 1-there is a device but not initialized yet,
                           // 2-there is a device but the initialization enumeration failed,
                           // 3-there is a device and the initialization enumeration is successful
    uint8_t DeviceAddress; // USB address assigned to the device
    uint8_t DeviceSpeed;   // 0 means low speed, non-0 means full speed
    uint8_t DeviceType;    // device type
    uint16_t DeviceVID;
    uint16_t DevicePID;
    uint8_t GpVar[4];      // General variable, stores endpoints
    uint8_t GpHUBPortNum;  // General variable, if it is a HUB, it represents the number of HUB ports
} _RootHubDev;

typedef struct
{
    uint8_t DeviceStatus;  // Device status,
                           // 0-no device,
                           // 1-there is a device but not initialized yet,
                           // 2-there is a device but the initialization enumeration failed,
                           // 3-there is a device and the initialization enumeration is successful
    uint8_t DeviceAddress; // USB address assigned to the device
    uint8_t DeviceSpeed;   // 0 means low speed, non-0 means full speed
    uint8_t DeviceType;    // device type
    uint16_t DeviceVID;
    uint16_t DevicePID;
    uint8_t GpVar[4]; // General variable
} _DevOnHubPort; // Assumption: no more than 1 external HUB, and each external HUB does not exceed HUB_MAX_PORTS ports (no matter if there are more)

extern __xdata _RootHubDev ThisUsbDev;
extern __xdata _DevOnHubPort DevOnHubPort[HUB_MAX_PORTS]; // Assumption: no more than 1 external HUB, and each external HUB
                                                          // does not exceed HUB_MAX_PORTS ports (it doesn’t matter if there are more)
extern uint8_t Set_Port;
#endif


extern __xdata uint8_t  Com_Buffer[];
extern __bit     FoundNewDev;
extern __bit     HubLowSpeed; // Low-speed devices under HUB require special treatment

#define pSetupReq   ((PXUSB_SETUP_REQ)TxBuffer)


void DisableRootHubPort(); // Close the ROOT-HUB port. In fact, the hardware has been automatically closed. Here we just clear some structural status.
uint8_t AnalyzeRootHub(void); // Analyze the ROOT-HUB status and handle the device plug-in and unplug events of the ROOT-HUB port
                              // Return ERR_SUCCESS if there is no situation, return ERR_USB_CONNECT if a new connection is detected,
                              // return ERR_USB_DISCON if a disconnection is detected
void SetHostUsbAddr(uint8_t addr); // Set the USB device address currently operated by the USB host
void SetUsbSpeed(uint8_t FullSpeed); // Set the current USB speed
void ResetRootHubPort(); // After detecting the device, reset the bus of the corresponding port to prepare for enumerating the device and
                         // set it to full speed by default
uint8_t EnableRootHubPort(); // Enable the ROOT-HUB port. Set the corresponding bUH_PORT_EN to 1 to open the port.
                             // Disconnection of the device may result in return failure.
void SelectHubPort(uint8_t HubPortIndex); // HubPortIndex=0 selects the ROOT-HUB port specified by the operation,
                                          // otherwise selects the specified port of the external HUB of the ROOT-HUB port specified by the operation.
uint8_t WaitUSB_Interrupt(void); // Wait for USB interrupt

// CH554 transmission transaction, input destination endpoint address/PID token, synchronization flag,
// total NAK retry time in 20uS units (0 means no retry, 0xFFFF infinite retry), return 0 for success, timeout/error retry
// endp_pid: The high 4 bits are the token_pid token, the low 4 bits are the endpoint address
uint8_t USBHostTransact(uint8_t endp_pid, uint8_t tog, uint16_t timeout);

// Execute control transfer, 8-byte request code is in pSetupReq,
// DataBuf is an optional transceiver buffer
// If you need to receive and send data, then DataBuf needs to point to a valid buffer to store subsequent data.
// The actual total length of successful transmission and reception is returned and stored in the byte variable pointed to by ReqLen.
uint8_t HostCtrlTransfer(__xdata uint8_t *DataBuf, uint8_t *RetLen);

void CopySetupReqPkg(__code uint8_t *pReqPkt); // Copy control transmission request packet
uint8_t CtrlGetDeviceDescr(void); // Get the device descriptor and return it in TxBuffer
uint8_t CtrlGetConfigDescr(void); // Get the configuration descriptor and return it in TxBuffer
uint8_t CtrlSetUsbAddress(uint8_t addr); // Set USB device address
uint8_t CtrlSetUsbConfig(uint8_t cfg); // Set USB device configuration
uint8_t CtrlClearEndpStall(uint8_t endp); // Clear endpoint STALL

#ifndef DISK_BASE_BUF_LEN
uint8_t CtrlSetUsbIntercace(uint8_t cfg); // Set USB device interface
uint8_t CtrlGetHIDDeviceReport(uint8_t infc); // HID class commands, SET_IDLE and GET_REPORT
uint8_t CtrlGetHubDescr(void); // Get the HUB descriptor and return it to TxBuffer
uint8_t HubGetPortStatus(uint8_t HubPortIndex); // Query the HUB port status and return it in TxBuffer
uint8_t HubSetPortFeature(uint8_t HubPortIndex, uint8_t FeatureSelt); // Set HUB port characteristics
uint8_t HubClearPortFeature(uint8_t HubPortIndex, uint8_t FeatureSelt); // Clear HUB port characteristics
uint8_t CtrlGetXPrinterReport1(void); // Printer class command
uint8_t AnalyzeHidIntEndp(__xdata uint8_t *buf, uint8_t HubPortIndex); // Analyze the address of the HID interrupt endpoint from the descriptor
uint8_t AnalyzeBulkEndp(__xdata uint8_t *buf, uint8_t HubPortIndex); // Analyze the batch endpoint
uint8_t TouchStartAOA(void); // Try AOA startup
uint8_t EnumAllRootDevice(void); // Enumerate USB devices of all ROOT-HUB ports
uint8_t InitDevOnHub(uint8_t HubPortIndex); // Initialize the secondary USB device after enumerating the external HUB
uint8_t EnumHubPort(); // Enumerate each port of the external HUB hub on the specified ROOT-HUB port,
                       // check whether there is a connection or removal event on each port, and initialize the secondary USB device
uint8_t EnumAllHubPort(void); // Enumerate the secondary USB devices behind the external HUB under all ROOT-HUB ports
uint16_t SearchTypeDevice(uint8_t type); // Search for the port number of the specified type of device on each port of ROOT-HUB and external HUB.
                                         // If the output port number is 0xFFFF, it is not found.
                                         // The high 8 bits of the output are the ROOT-HUB port number,
                                         // the low 8 bits are the port number of the external HUB, and if the low 8 bits are 0,
                                         // the device is directly on the ROOT-HUB port.
uint8_t SETorOFFNumLock(uint8_t *buf);
#endif

uint8_t InitRootDevice(void); // Initialize the USB device of the specified ROOT-HUB port
void InitUSB_Host(void); // Initialize USB host
