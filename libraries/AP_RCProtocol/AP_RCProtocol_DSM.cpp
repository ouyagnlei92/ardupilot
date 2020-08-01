/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */
/*
 * See https://www.spektrumrc.com/ProdInfo/Files/Remote%20Receiver%20Interfacing%20Rev%20A.pdf for official
 * Spektrum documentation on the format.
 */

#include <AP_Vehicle/AP_Vehicle_Type.h>
#include "AP_RCProtocol_DSM.h"
#if !APM_BUILD_TYPE(APM_BUILD_iofirmware)
#include "AP_RCProtocol_SRXL2.h"
#endif
#include <stdio.h>

extern const AP_HAL::HAL& hal;

// #define DSM_DEBUG
#ifdef DSM_DEBUG
# define debug(fmt, args...)	printf(fmt "\n", ##args)
#else
# define debug(fmt, args...)	do {} while(0)
#endif


#define DSM_FRAME_SIZE		16		/**<DSM frame size in bytes*/
#define DSM_FRAME_CHANNELS	7		/**<Max supported DSM channels*/
#define DSM2_1024_22MS      0x01
#define DSM2_2048_11MS      0x12
#define DSMX_2048_22MS      0xa2
#define DSMX_2048_11MS      0xb2
#define SPEKTRUM_VTX_CONTROL_FRAME_MASK 0xf000f000
#define SPEKTRUM_VTX_CONTROL_FRAME      0xe000e000

#define SPEKTRUM_VTX_BAND_MASK          0x00e00000
#define SPEKTRUM_VTX_CHANNEL_MASK       0x000f0000
#define SPEKTRUM_VTX_PIT_MODE_MASK      0x00000010
#define SPEKTRUM_VTX_POWER_MASK         0x00000007

#define SPEKTRUM_VTX_BAND_SHIFT         21
#define SPEKTRUM_VTX_CHANNEL_SHIFT      16
#define SPEKTRUM_VTX_PIT_MODE_SHIFT     4
#define SPEKTRUM_VTX_POWER_SHIFT        0

void AP_RCProtocol_DSM::process_pulse(uint32_t width_s0, uint32_t width_s1)
{
    uint8_t b;
    if (ss.process_pulse(width_s0, width_s1, b)) {
<<<<<<< HEAD
        _process_byte(ss.get_byte_timestamp_us()/1000U, b);
    }
}

/**
 * Attempt to decode a single channel raw channel datum
 *
 * The DSM* protocol doesn't provide any explicit framing,
 * so we detect dsm frame boundaries by the inter-dsm frame delay.
 *
 * The minimum dsm frame spacing is 11ms; with 16 bytes at 115200bps
 * dsm frame transmission time is ~1.4ms.
 *
 * We expect to only be called when bytes arrive for processing,
 * and if an interval of more than 5ms passes between calls,
 * the first byte we read will be the first byte of a dsm frame.
 *
 * In the case where byte(s) are dropped from a dsm frame, this also
 * provides a degree of protection. Of course, it would be better
 * if we didn't drop bytes...
 *
 * Upon receiving a full dsm frame we attempt to decode it
 *
 * @param[in] raw 16 bit raw channel value from dsm frame
 * @param[in] shift position of channel number in raw data
 * @param[out] channel pointer to returned channel number
 * @param[out] value pointer to returned channel value
 * @return true=raw value successfully decoded
 */
bool AP_RCProtocol_DSM::dsm_decode_channel(uint16_t raw, unsigned shift, unsigned *channel, unsigned *value)
{

    if (raw == 0xffff) {
        return false;
    }

    *channel = (raw >> shift) & 0xf;

    uint16_t data_mask = (1 << shift) - 1;
    *value = raw & data_mask;

    //debug("DSM: %d 0x%04x -> %d %d", shift, raw, *channel, *value);

    return true;
}

/**
 * Attempt to guess if receiving 10 or 11 bit channel values
 *
 * @param[in] reset true=reset the 10/11 bit state to unknown
 */
void AP_RCProtocol_DSM::dsm_guess_format(bool reset, const uint8_t dsm_frame[16])
{
    /* reset the 10/11 bit sniffed channel masks */
    if (reset) {
        cs10 = 0;
        cs11 = 0;
        samples = 0;
        channel_shift = 0;
        return;
    }

    /* scan the channels in the current dsm_frame in both 10- and 11-bit mode */
    for (unsigned i = 0; i < DSM_FRAME_CHANNELS; i++) {

        const uint8_t *dp = &dsm_frame[2 + (2 * i)];
        uint16_t raw = (dp[0] << 8) | dp[1];
        unsigned channel, value;

        /* if the channel decodes, remember the assigned number */
        if (dsm_decode_channel(raw, 10, &channel, &value) && (channel < 31)) {
            cs10 |= (1 << channel);
        }

        if (dsm_decode_channel(raw, 11, &channel, &value) && (channel < 31)) {
            cs11 |= (1 << channel);
        }

        /* XXX if we cared, we could look for the phase bit here to decide 1 vs. 2-dsm_frame format */
    }

    /* wait until we have seen plenty of frames - 5 should normally be enough */
    if (samples++ < 5) {
        return;
    }

    /*
     * Iterate the set of sensible sniffed channel sets and see whether
     * decoding in 10 or 11-bit mode has yielded anything we recognize.
     *
     * XXX Note that due to what seem to be bugs in the DSM2 high-resolution
     *     stream, we may want to sniff for longer in some cases when we think we
     *     are talking to a DSM2 receiver in high-resolution mode (so that we can
     *     reject it, ideally).
     *     See e.g. http://git.openpilot.org/cru/OPReview-116 for a discussion
     *     of this issue.
     */
    static const uint32_t masks[] = {
        0x3f,	/* 6 channels (DX6) */
        0x7f,	/* 7 channels (DX7) */
        0xff,	/* 8 channels (DX8) */
        0x1ff,	/* 9 channels (DX9, etc.) */
        0x3ff,	/* 10 channels (DX10) */
        0x7ff,	/* 11 channels DX8 22ms */
        0xfff,	/* 12 channels DX8 22ms */
        0x1fff,	/* 13 channels (DX10t) */
        0x3fff	/* 18 channels (DX10) */
    };
    unsigned votes10 = 0;
    unsigned votes11 = 0;

    for (unsigned i = 0; i < sizeof(masks)/sizeof(masks[0]); i++) {

        if (cs10 == masks[i]) {
            votes10++;
        }

        if (cs11 == masks[i]) {
            votes11++;
        }
    }

    if ((votes11 == 1) && (votes10 == 0)) {
        channel_shift = 11;
        debug("DSM: 11-bit format");
        return;
    }

    if ((votes10 == 1) && (votes11 == 0)) {
        channel_shift = 10;
        debug("DSM: 10-bit format");
        return;
=======
        _process_byte(ss.get_byte_timestamp_us(), b);
>>>>>>> upstream/master
    }
}

/**
 * Decode the entire dsm frame (all contained channels)
 *
 */
bool AP_RCProtocol_DSM::dsm_decode(uint32_t frame_time_us, const uint8_t dsm_frame[16],
                                   uint16_t *values, uint16_t *num_values, uint16_t max_values)
{
    /* we have received something we think is a dsm_frame */
    last_frame_time_us = frame_time_us;
    // Get the VTX control bytes in a frame
    uint32_t vtxControl = ((dsm_frame[AP_DSM_FRAME_SIZE-4] << 24)
        | (dsm_frame[AP_DSM_FRAME_SIZE-3] << 16)
        | (dsm_frame[AP_DSM_FRAME_SIZE-2] <<  8)
        | (dsm_frame[AP_DSM_FRAME_SIZE-1] <<  0));

    uint8_t dsm_frame_data_size;
    // Handle VTX control frame.
    if ((vtxControl & SPEKTRUM_VTX_CONTROL_FRAME_MASK) == SPEKTRUM_VTX_CONTROL_FRAME 
        && (dsm_frame[2] & 0x80) == 0)  {
        dsm_frame_data_size = AP_DSM_FRAME_SIZE - 4;
#if !APM_BUILD_TYPE(APM_BUILD_iofirmware)
        AP_RCProtocol_SRXL2::configure_vtx(
            (vtxControl & SPEKTRUM_VTX_BAND_MASK)     >> SPEKTRUM_VTX_BAND_SHIFT,
            (vtxControl & SPEKTRUM_VTX_CHANNEL_MASK)  >> SPEKTRUM_VTX_CHANNEL_SHIFT,
            (vtxControl & SPEKTRUM_VTX_POWER_MASK)    >> SPEKTRUM_VTX_POWER_SHIFT,
            (vtxControl & SPEKTRUM_VTX_PIT_MODE_MASK) >> SPEKTRUM_VTX_PIT_MODE_SHIFT);
#endif
    } else {
        dsm_frame_data_size = AP_DSM_FRAME_SIZE;
    }

    // Get the RC control channel inputs
    for (uint8_t b = 3; b < dsm_frame_data_size; b += 2) {
        uint8_t channel = 0x0F & (dsm_frame[b - 1] >> channel_shift);

        uint32_t value = ((uint32_t)(dsm_frame[b - 1] & channel_mask) << 8) + dsm_frame[b];

        /* ignore channels out of range */
        if (channel >= max_values) {
            continue;
        }

        /* update the decoded channel count */
        if (channel >= *num_values) {
            *num_values = channel + 1;
        }

        /* convert 0-1024 / 0-2048 values to 1000-2000 ppm encoding. */
        if (channel_shift == 2) {
            value *= 2;
        }

        /* Spektrum scaling is defined as (see reference):
            2048: PWM_OUT = (ServoPosition x 58.3μs) + 903
            1024: PWM_OUT = (ServoPosition x 116.6μs) + 903 */
        /* scaled integer for decent accuracy while staying efficient */
        value = ((int32_t)value * 1194) / 2048 + 903;

        /*
        * Store the decoded channel into the R/C input buffer, taking into
        * account the different ideas about channel assignement that we have.
        *
        * Specifically, the first four channels in rc_channel_data are roll, pitch, thrust, yaw,
        * but the first four channels from the DSM receiver are thrust, roll, pitch, yaw.
        */
        switch (channel) {
        case 0:
            channel = 2;
            break;

        case 1:
            channel = 0;
            break;

        case 2:
            channel = 1;
            break;

        default:
            break;
        }

        values[channel] = value;
    }

    /*
     * XXX Note that we may be in failsafe here; we need to work out how to detect that.
     */
    return true;
}


/*
  start bind on DSM satellites
 */
void AP_RCProtocol_DSM::start_bind(void)
{
    bind_state = BIND_STATE1;
}


/*
  update function used for bind state machine
 */
void AP_RCProtocol_DSM::update(void)
{
#if defined(HAL_GPIO_SPEKTRUM_PWR) && defined(HAL_GPIO_SPEKTRUM_RC)
    switch (bind_state) {
    case BIND_STATE_NONE:
        break;

    case BIND_STATE1:
        hal.gpio->write(HAL_GPIO_SPEKTRUM_PWR, !HAL_SPEKTRUM_PWR_ENABLED);
        hal.gpio->pinMode(HAL_GPIO_SPEKTRUM_RC, 1);
        hal.gpio->write(HAL_GPIO_SPEKTRUM_RC, 1);
        bind_last_ms = AP_HAL::millis();
        bind_state = BIND_STATE2;
        break;

    case BIND_STATE2: {
        uint32_t now = AP_HAL::millis();
        if (now - bind_last_ms > 500) {
            hal.gpio->write(HAL_GPIO_SPEKTRUM_PWR, HAL_SPEKTRUM_PWR_ENABLED);
            bind_last_ms = now;
            bind_state = BIND_STATE3;
        }
        break;
    }

    case BIND_STATE3: {
        uint32_t now = AP_HAL::millis();
        if (now - bind_last_ms > 72) {
            // 9 pulses works with all satellite receivers, and supports the highest
            // available protocol
            const uint8_t num_pulses = 9;
            for (uint8_t i=0; i<num_pulses; i++) {
                hal.scheduler->delay_microseconds(120);
                hal.gpio->write(HAL_GPIO_SPEKTRUM_RC, 0);
                hal.scheduler->delay_microseconds(120);
                hal.gpio->write(HAL_GPIO_SPEKTRUM_RC, 1);
            }
            bind_last_ms = now;
            bind_state = BIND_STATE4;
        }
        break;
    }

    case BIND_STATE4: {
        uint32_t now = AP_HAL::millis();
        if (now - bind_last_ms > 50) {
            hal.gpio->pinMode(HAL_GPIO_SPEKTRUM_RC, 0);
            bind_state = BIND_STATE_NONE;
        }
        break;
    }
    }
#endif
}

/*
  parse one DSM byte, maintaining decoder state
 */
bool AP_RCProtocol_DSM::dsm_parse_byte(uint32_t frame_time_us, uint8_t b, uint16_t *values,
                                       uint16_t *num_values, uint16_t max_channels)
{
    /* this is set by the decoding state machine and will default to false
	 * once everything that was decodable has been decoded.
	 */
	bool decode_ret = false;

    // we took too long decoding, start again
    if (byte_input.ofs > 0 && (frame_time_us - start_frame_time_us) > 6000U) {
        start_frame_time_us = frame_time_us;
        byte_input.ofs = 0;
    }

    // there will be at least a 5ms gap between successive DSM frames. if we see it
    // assume we are starting a new frame
    if ((frame_time_us - last_rx_time_us) > 5000U) {
        start_frame_time_us = frame_time_us;
        byte_input.ofs = 0;
    }

    /* overflow check */
    if (byte_input.ofs >= AP_DSM_FRAME_SIZE) {
        start_frame_time_us = frame_time_us;
        byte_input.ofs = 0;
    }

    if (byte_input.ofs == 1) {
        // saw a beginning of frame marker
        if (b == DSM2_1024_22MS || b == DSM2_2048_11MS || b == DSMX_2048_22MS || b == DSMX_2048_11MS) {
            if (b == DSM2_1024_22MS) {
                // 10 bit frames
                channel_shift = 2;
                channel_mask = 0x03;
            } else {
                // 11 bit frames
                channel_shift = 3;
                channel_mask = 0x07;
            }
        // bad frame marker so reset
        } else {
            start_frame_time_us = frame_time_us;
            byte_input.ofs = 0;
        }
    }

    byte_input.buf[byte_input.ofs++] = b;

    /* decode whatever we got and expect */
    if (byte_input.ofs == AP_DSM_FRAME_SIZE) {
        log_data(AP_RCProtocol::DSM, frame_time_us, byte_input.buf, byte_input.ofs);
#ifdef DSM_DEBUG
        for (uint16_t i = 0; i < 16; i++) {
            printf("%02x", byte_input.buf[i]);
        }
        printf("\n%02x%02x", byte_input.buf[0], byte_input.buf[1]);
        for (uint16_t i = 2; i < 16; i+=2) {
            printf(" %01x/%03x", (byte_input.buf[i] & 0x78) >> 4, (byte_input.buf[i] & 0x7) << 8 | byte_input.buf[i+1]);
        }
        printf("\n");
#endif
        decode_ret = dsm_decode(frame_time_us, byte_input.buf, values, &chan_count, max_channels);

        /* we consumed the partial frame, reset */
        byte_input.ofs = 0;
    }

    if (decode_ret) {
		*num_values = chan_count;
	}

    last_rx_time_us = frame_time_us;

	/* return false as default */
	return decode_ret;
}

// support byte input
void AP_RCProtocol_DSM::_process_byte(uint32_t timestamp_us, uint8_t b)
{
    uint16_t v[AP_DSM_MAX_CHANNELS];
    uint16_t nchan;
    memcpy(v, last_values, sizeof(v));
    if (dsm_parse_byte(timestamp_us, b, v, &nchan, AP_DSM_MAX_CHANNELS)) {
        memcpy(last_values, v, sizeof(v));
        if (nchan >= MIN_RCIN_CHANNELS) {
            add_input(nchan, last_values, false);
        }
    }
}

// support byte input
void AP_RCProtocol_DSM::process_byte(uint8_t b, uint32_t baudrate)
{
    if (baudrate != 115200) {
        return;
    }
    _process_byte(AP_HAL::micros(), b);
}
