/*
 * Xbox 360 (XInput) driver — maps 20-byte input messages to NormalizedInput.
 */

#include "../../include/controllers/xbox360.h"
#include "../../include/transport/usb.h"

#include <string.h>
#include <ps5/klog.h>

static int16_t le16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/* XInput sticks are signed 16-bit with Y increasing upwards; PlayStation
 * sticks are 0..255 with Y increasing downwards. */
static uint8_t axis(int16_t v)     { return (uint8_t)(((int)v + 32768) >> 8); }
static uint8_t axis_inv(int16_t v) { return (uint8_t)((32767 - (int)v) >> 8); }

static int xbox360_init(Controller *ctrl)
{
    ctrl->priv = NULL;
    return 0;
}

/* Player LED command — Windows and Linux xpad send it on connect, and
 * multi-mode dongles use it to decide the host speaks XInput. */
static int xbox360_setup(Controller *ctrl)
{
    uint8_t led[3] = {0x01, 0x03, (uint8_t)(0x06 + (ctrl->slot & 3))};
    int n = usb_write_report(ctrl->fd, led, sizeof(led));
    klog_printf("[AnyController] xbox360: LED command -> %d\n", n);
    return 0;
}

static int xbox360_parse_input(Controller *ctrl, const uint8_t *r, int len,
                               NormalizedInput *out)
{
    (void)ctrl;
    /* Byte 0 is the message type; anything else (the wireless receiver's 0x08
     * frames, HID noise from another interface) belongs to a driver that knows
     * it. The length in byte 1 is not checked: adapters that emulate a wired
     * pad extend or pad the report — xpad accepts 20..32 bytes and reads only
     * the first 20, and an 8BitDo receiver reports 0x20 here, not 0x14. */
    if (len < X360_INPUT_LEN || r[0] != X360_MSG_INPUT)
        return -1;

    uint16_t b = (uint16_t)r[2] | ((uint16_t)r[3] << 8);
    memset(out, 0, sizeof(*out));

    /* Positional: bottom/right/left/top face buttons */
    out->cross      = !!(b & X360_A);
    out->circle     = !!(b & X360_B);
    out->square     = !!(b & X360_X);
    out->triangle   = !!(b & X360_Y);
    out->l1         = !!(b & X360_LB);
    out->r1         = !!(b & X360_RB);
    out->l3         = !!(b & X360_LSTICK);
    out->r3         = !!(b & X360_RSTICK);
    out->dpad_up    = !!(b & X360_DPAD_UP);
    out->dpad_down  = !!(b & X360_DPAD_DOWN);
    out->dpad_left  = !!(b & X360_DPAD_LEFT);
    out->dpad_right = !!(b & X360_DPAD_RIGHT);
    out->options    = !!(b & X360_START);
    out->create     = !!(b & X360_BACK);
    out->ps_button  = !!(b & X360_GUIDE);

    out->l2_analog = r[4];
    out->r2_analog = r[5];

    out->left_stick_x  = axis(le16(r + 6));
    out->left_stick_y  = axis_inv(le16(r + 8));
    out->right_stick_x = axis(le16(r + 10));
    out->right_stick_y = axis_inv(le16(r + 12));
    return 0;
}

static int xbox360_build_output(Controller *ctrl,
                                uint8_t large_motor, uint8_t small_motor,
                                uint8_t r, uint8_t g, uint8_t b,
                                uint8_t *buf, int max_len)
{
    (void)ctrl; (void)r; (void)g; (void)b;
    if (max_len < 8) return 0;
    uint8_t rumble[8] = {0x00, 0x08, 0x00, large_motor, small_motor, 0x00, 0x00, 0x00};
    memcpy(buf, rumble, sizeof(rumble));
    return sizeof(rumble);
}

static void xbox360_cleanup(Controller *ctrl)
{
    (void)ctrl;
}

static const ControllerDriver xbox360_driver_wired = {
    .name         = "Xbox 360 Controller",
    .type         = CTRL_TYPE_XBOX_360,
    .vendor_id    = XBOX_VID,
    .product_id   = XBOX360_PID,
    .init         = xbox360_init,
    .setup        = xbox360_setup,
    .parse_input  = xbox360_parse_input,
    .build_output = xbox360_build_output,
    .cleanup      = xbox360_cleanup,
};

/* Same protocol, different USB identity: the adapter presents itself as a
 * wired 360 pad until a controller links, then re-enumerates as 2dc8:3106.
 * Without this instance the pad drops out of the registry mid-session. */
static const ControllerDriver xbox360_driver_bitdo = {
    .name         = "8BitDo receiver",
    .type         = CTRL_TYPE_XBOX_360,
    .vendor_id    = BITDO_VID,
    .product_id   = BITDO_RX_PID,
    .init         = xbox360_init,
    .setup        = xbox360_setup,
    .parse_input  = xbox360_parse_input,
    .build_output = xbox360_build_output,
    .cleanup      = xbox360_cleanup,
};

const ControllerDriver *xbox360_get_driver(int variant)
{
    return variant == 1 ? &xbox360_driver_bitdo : &xbox360_driver_wired;
}
