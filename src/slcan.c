//
// slcan: Parse incoming and generate outgoing slcan messages
// Improved version ported from CANable 2.0 with better extended ID handling
//

#include "stm32f0xx_hal.h"
#include <string.h>
#include "can.h"
#include "error.h"
#include "slcan.h"
#include "printf.h"
#include "usbd_cdc_if.h"


// Private variables
static char* fw_id = GIT_VERSION " " GIT_REMOTE "\r";


// Parse an incoming CAN frame into an outgoing slcan message
int8_t slcan_parse_frame(uint8_t *buf, CAN_RxHeaderTypeDef *frame_header, uint8_t* frame_data)
{
    // Clear buffer
    for (uint8_t j=0; j < SLCAN_MTU; j++)
        buf[j] = '\0';

    // Start building the slcan message string at idx 0 in buf[]
    uint8_t msg_idx = 0;

    // Add character for frame type
    if (frame_header->RTR == CAN_RTR_DATA)
    {
        buf[msg_idx] = 't';
    } else if (frame_header->RTR == CAN_RTR_REMOTE) {
        buf[msg_idx] = 'r';
    }

    // Assume standard identifier
    uint8_t id_len = SLCAN_STD_ID_LEN;
    uint32_t can_id = frame_header->StdId;

    // Check if extended
    if (frame_header->IDE == CAN_ID_EXT)
    {
        // Convert first char to upper case for extended frame
        buf[msg_idx] -= 32;
        id_len = SLCAN_EXT_ID_LEN;
        can_id = frame_header->ExtId;
    }
    msg_idx++;

    // Add identifier to buffer using corrected method
    // We need to fill positions msg_idx to msg_idx+id_len-1
    for(uint8_t i = 0; i < id_len; i++)
    {
        // Extract nibble from most significant position
        uint8_t nibble_pos = (id_len - 1 - i) * 4;
        uint8_t nibble = (can_id >> nibble_pos) & 0xF;
        buf[msg_idx + i] = nibble;
    }
    msg_idx += id_len;

    // Add DLC to buffer
    buf[msg_idx++] = frame_header->DLC;

    // Add data bytes
    for (uint8_t j = 0; j < frame_header->DLC; j++)
    {
        buf[msg_idx++] = (frame_data[j] >> 4);
        buf[msg_idx++] = (frame_data[j] & 0x0F);
    }

    // Convert to ASCII (2nd character to end)
    for (uint8_t j = 1; j < msg_idx; j++)
    {
        if (buf[j] < 0xA) {
            buf[j] += 0x30;
        } else {
            buf[j] += 0x37;
        }
    }

    // Add CR for slcan EOL
    buf[msg_idx++] = '\r';

    // Return string length
    return msg_idx;
}


// Parse an incoming slcan command from the USB CDC port
int8_t slcan_parse_str(uint8_t *buf, uint8_t len)
{
    // Set default header. All values overridden below as needed.
    CAN_TxHeaderTypeDef frame_header =
    {
        .RTR = CAN_RTR_DATA,      // default to data frame
        .IDE = CAN_ID_STD,        // default to standard ID
        .StdId = 0,
        .ExtId = 0,
        .DLC = 0,
        .TransmitGlobalTime = DISABLE
    };
    uint8_t frame_data[8] = {0};

    // Convert from ASCII (2nd character to end)
    for (uint8_t i = 1; i < len; i++)
    {
        // Lowercase letters
        if(buf[i] >= 'a')
            buf[i] = buf[i] - 'a' + 10;
        // Uppercase letters
        else if(buf[i] >= 'A')
            buf[i] = buf[i] - 'A' + 10;
        // Numbers
        else
            buf[i] = buf[i] - '0';
    }

    // Handle each incoming command
    switch(buf[0])
    {
        // Open channel
        case 'O':
            can_enable();
            return 0;

        // Close channel
        case 'C':
            can_disable();
            return 0;

        // Set nominal bitrate
        case 'S':
            // Check for valid bitrate
            if(buf[1] >= CAN_BITRATE_INVALID)
            {
                return -1;
            }
            can_set_bitrate(buf[1]);
            return 0;

        // Set mode command
        case 'M':
        case 'm':
            if (buf[1] == 1)
            {
                // Mode 1: silent
                can_set_silent(1);
            } else {
                // Default to normal mode
                can_set_silent(0);
            }
            return 0;

        // Set autoretry command  
        case 'A':
        case 'a':
            if (buf[1] == 1)
            {
                // Mode 1: autoretry enabled (default)
                can_set_autoretransmit(1);
            } else {
                // Mode 0: autoretry disabled
                can_set_autoretransmit(0);
            }
            return 0;

        // Report firmware version and remote
        case 'V':
        {
            CDC_Transmit_FS((uint8_t*)fw_id, strlen(fw_id));
            return 0;
        }

        // Report error register
        case 'E':
        {
            char errstr[64] = {0};
            snprintf_(errstr, 64, "CANable Error Register: %X", (unsigned int)error_reg());
            CDC_Transmit_FS((uint8_t*)errstr, strlen(errstr));
            return 0;
        }

        // Transmit data frame command
        case 'T':
            frame_header.IDE = CAN_ID_EXT;
            break;
        case 't':
            break;

        // Transmit remote frame command
        case 'r':
            frame_header.RTR = CAN_RTR_REMOTE;
            break;
        case 'R':
            frame_header.IDE = CAN_ID_EXT;
            frame_header.RTR = CAN_RTR_REMOTE;
            break;

        // Invalid command
        default:
            return -1;
    }

    // Start parsing at second byte (skip command byte)
    uint8_t parse_loc = 1;

    // Zero out identifier
    frame_header.StdId = 0;
    frame_header.ExtId = 0;

    // Default to standard ID
    uint8_t id_len = SLCAN_STD_ID_LEN;

    // Update length if message is extended ID
    if(frame_header.IDE == CAN_ID_EXT)
        id_len = SLCAN_EXT_ID_LEN;

    // Parse ID using improved method
    uint32_t identifier = 0;
    for(uint8_t i = 0; i < id_len; i++)
    {
        identifier *= 16;
        identifier += buf[parse_loc++];
    }

    // Store identifier in appropriate field
    if(frame_header.IDE == CAN_ID_EXT)
        frame_header.ExtId = identifier;
    else
        frame_header.StdId = identifier;

    // Attempt to parse DLC and check sanity
    uint8_t dlc_raw = buf[parse_loc++];
    if(dlc_raw > 8)
    {
        return -1;
    }
    frame_header.DLC = dlc_raw;

    // Parse data
    for (uint8_t i = 0; i < frame_header.DLC; i++)
    {
        frame_data[i] = (buf[parse_loc] << 4) + buf[parse_loc+1];
        parse_loc += 2;
    }

    // Transmit the message
    can_tx(&frame_header, frame_data);

    return 0;
}

