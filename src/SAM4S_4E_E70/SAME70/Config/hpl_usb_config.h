/* Auto-generated config file hpl_usb_config.h */
#ifndef HPL_USB_CONFIG_H
#define HPL_USB_CONFIG_H

// <<< Use Configuration Wizard in Context Menu >>>

#define CONF_USB_N_0 0
#define CONF_USB_N_1 1
#define CONF_USB_N_2 2
#define CONF_USB_N_3 3
#define CONF_USB_N_4 4
#define CONF_USB_N_5 5
#define CONF_USB_N_6 6
#define CONF_USB_N_7 7
#define CONF_USB_N_8 8
#define CONF_USB_N_9 9
#define CONF_USB_N_10 10

// <h> USB Device HAL Configuration

// <y> Max number of endpoints supported
// <i> Limits the number of endpoints (described by EP address) can be used in app.
// <CONF_USB_N_1"> 1 (EP0 only)
// <CONF_USB_N_2"> 2 (EP0 + 1 endpoint)
// <CONF_USB_N_3"> 3 (EP0 + 2 endpoints)
// <CONF_USB_N_4"> 4 (EP0 + 3 endpoints)
// <CONF_USB_N_5"> 5 (EP0 + 4 endpoints)
// <CONF_USB_N_6"> 6 (EP0 + 5 endpoints)
// <CONF_USB_N_7"> 7 (EP0 + 6 endpoints)
// <CONF_USB_N_8"> 8 (EP0 + 7 endpoints)
// <CONF_USB_N_9"> 9 (EP0 + 8 endpoints)
// <CONF_USB_N_10"> 10 (EP0 + 9 endpoints)
// <CONF_USB_D_N_EP_MAX"> Max possible (by "Max Endpoint Number" config)
// <id> usbd_num_ep_sp
#ifndef CONF_USB_D_NUM_EP_SP
#define CONF_USB_D_NUM_EP_SP CONF_USB_N_4
#endif

// </h>

// <h> USB Device Implement Configurations

// <y> Max Endpoint Number supported
// <i> Limits the max endpoint number.
// <CONF_USB_N_0"> 0 (EP0)
// <CONF_USB_N_1"> 1 (EP01/EP81)
// <CONF_USB_N_2"> 2 (EP02/EP82)
// <CONF_USB_N_3"> 3 (EP03/EP83)
// <CONF_USB_N_4"> 4 (EP04/EP84)
// <CONF_USB_N_5"> 5 (EP05/EP85)
// <CONF_USB_N_6"> 6 (EP06/EP86)
// <CONF_USB_N_7"> 7 (EP07/EP87)
// <CONF_USB_N_8"> 8 (EP08/EP88)
// <CONF_USB_N_9"> 9 (EP09/EP89)
// <id> usbd_arch_max_ep_n
#ifndef CONF_USB_D_MAX_EP_N
#define CONF_USB_D_MAX_EP_N CONF_USB_N_3
#endif

// <y> USB Device Speed Limit
// <i> Limits the working speed of the device.
// <USB_SPEED_HS"> High speed
// <USB_SPEED_FS"> Full speed
// <USB_SPEED_LS"> Low Speed
// <id> usbd_arch_speed
#ifndef CONF_USB_D_SPEED
#define CONF_USB_D_SPEED USB_SPEED_HS
#endif

// <q> USB Device DMA Operation Enable
// <i> This defines whether DMA operation is enabled or disabled.
// <i> Note that endpoint 1 ~ 7 can use DMA operation.
// <id> usbd_dma_enable
#ifndef CONF_USB_D_DMA_ENABLE
#define CONF_USB_D_DMA_ENABLE 1
#endif

// </h>

// <<< end of configuration section >>>

#endif // HPL_USB_CONFIG_H
