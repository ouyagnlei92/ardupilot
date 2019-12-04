/// write by Awesome  -- Joyton
/// @file	AP_GPS_NMEA.h
/// @brief	NMEA protocol parser
///
/// This is a lightweight NMEA parser, derived originally from the
/// TinyGPS parser by Mikal Hart.  It is frugal in its use of memory
/// and tries to avoid unnecessary arithmetic.
///
/// The parser handles GPGGA, GPRMC and GPVTG messages, and attempts to be
/// robust in the face of occasional corruption in the input stream.  It
/// makes a basic effort to configure GPS' that are likely to be connected in
/// NMEA mode (SiRF, MediaTek and ublox) to emit the correct message
/// stream, but does not validate that the correct stream is being received.
/// In particular, a unit emitting just GPRMC will show as having a fix
/// even though no altitude data is being received.
///
/// GPVTG data is parsed, but as the message may not contain the the
/// qualifier field (this is common with e.g. older SiRF units) it is
/// not considered a source of fix-valid information.
///
#pragma once

#include "AP_GPS.h"
#include "GPS_Backend.h"

#define JOYTON_GPS_CONFIG  "LOG EVENTALLA ONCHANGED\r\n" "CONFIG EVENT ENABLE NEGATIVE 20\r\n" \
	"gpgga com1 0.2\r\n" "gprmc com1 0.2\r\n" "gpvtg com1 0.2\r\n" "saveconfig\r\n"

#define EVENTA_MSG "EVENTALLA"
#define CRC32_POLYNOMIAL 0xEDB88320L

/// NMEA parser
///
class AP_GPS_JOYTON : public AP_GPS_Backend
{
    friend class AP_GPS_NMEA_Test;

public:
    AP_GPS_JOYTON(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    /// Checks the serial receive buffer for characters,
    /// attempts to parse NMEA data and updates internal state
    /// accordingly.
    bool        read();

	static bool _detect(struct NMEA_detect_state &state, uint8_t data);

    const char *name() const override { return "NMEA"; }

private:
    /// Coding for the GPS sentences that the parser handles
    enum _sentence_types {      //there are some more than 10 fields in some sentences , thus we have to increase these value.
        _GPS_SENTENCE_RMC = 32,
        _GPS_SENTENCE_GGA = 64,
        _GPS_SENTENCE_VTG = 96,
        _GPS_SENTENCE_OTHER = 0
    };

    /// Update the decode state machine with a new character
    ///
    /// @param	c		The next character in the NMEA input stream
    /// @returns		True if processing the character has resulted in
    ///					an update to the GPS state
    ///
    bool                        _decode(char c);

    /// Return the numeric value of an ascii hex character
    ///
    /// @param	a		The character to be converted
    /// @returns		The value of the character as a hex digit
    ///
    int16_t                     _from_hex(char a);

    /// Parses the @p as a NMEA-style decimal number with
    /// up to 3 decimal digits.
    ///
    /// @returns		The value expressed by the string in @p,
    ///					multiplied by 100.
    ///
    static int32_t _parse_decimal_100(const char *p);

    /// Parses the current term as a NMEA-style degrees + minutes
    /// value with up to four decimal digits.
    ///
    /// This gives a theoretical resolution limit of around 1cm.
    ///
    /// @returns		The value expressed by the string in _term,
    ///					multiplied by 1e7.
    ///
    uint32_t    _parse_degrees();

    /// Processes the current term when it has been deemed to be
    /// complete.
    ///
    /// Each GPS message is broken up into terms separated by commas.
    /// Each term is then processed by this function as it is received.
    ///
    /// @returns		True if completing the term has resulted in
    ///					an update to the GPS state.
    bool                        _term_complete();

    /// return true if we have a new set of NMEA messages
    bool _have_new_message(void);

    uint8_t _parity;                                                    ///< NMEA message checksum accumulator
    bool _is_checksum_term;                                     ///< current term is the checksum
    char _term[15];                                                     ///< buffer for the current term within the current sentence
    uint8_t _sentence_type;                                     ///< the sentence type currently being processed
    uint8_t _term_number;                                       ///< term index within the current sentence
    uint8_t _term_offset;                                       ///< character offset with the term being received
    bool _gps_data_good;                                        ///< set when the sentence indicates data is good

    // The result of parsing terms within a message is stored temporarily until
    // the message is completely processed and the checksum validated.
    // This avoids the need to buffer the entire message.
    int32_t _new_time;                                                  ///< time parsed from a term
    int32_t _new_date;                                                  ///< date parsed from a term
    int32_t _new_latitude;                                      ///< latitude parsed from a term
    int32_t _new_longitude;                                     ///< longitude parsed from a term
    int32_t _new_altitude;                                      ///< altitude parsed from a term
    int32_t _new_speed;                                                 ///< speed parsed from a term
    int32_t _new_course;                                        ///< course parsed from a term
    uint16_t _new_hdop;                                                 ///< HDOP parsed from a term
    uint8_t _new_satellite_count;                       ///< satellite count parsed from a term
    uint8_t _new_quality_indicator;                                     ///< GPS quality indicator parsed from a term

    uint32_t _last_RMC_ms = 0;
    uint32_t _last_GGA_ms = 0;
    uint32_t _last_VTG_ms = 0;

    /// @name	Init strings
    ///			In ::init, an attempt is made to configure the GPS
    ///			unit to send just the messages that we are interested
    ///			in using these strings
    //@{
    static const char _SiRF_init_string[];         ///< init string for SiRF units
    static const char _MTK_init_string[];                  ///< init string for MediaTek units
    static const char _ublox_init_string[];        ///< init string for ublox units
    //@}

    static const char _initialisation_blob[];

    /// EVENTALLA Camera feedback
	#define EVENT_MAX_LEN 12

    /* add by awesome */
        struct event_data {
        	double lat;
        	double lon;
        	float alt;
        	float alt_error;
        	unsigned int pos_type;
        	unsigned int event_index;
        	unsigned char stats_num;
        };

        typedef enum event_parse_enum
        {
        	EVENT_PARSE_NONE = 0,
        	EVENT_PARSE_AYNC,   	/* Aync # */
        	EVENT_PARSE_MSG,		/* msg name "EVENTALLA" */
        	EVENT_PARSE_HEADER,		/* ';'end */
        	EVENT_PARSE_DATA,
        	EVENT_PARSE_CRC,		/* CRC32  from '#' to '*' all anscii, but no '#' and '*'. */
        	EVENT_PARSE_SUCCESS
        }EVENT_PARSE_ENUM;

        struct event_parse_status
        {
        	unsigned char buff[30];  /* save a message */
        	unsigned long crc32;
        	unsigned int count;
        	unsigned int dot_count;  /* ','�ĸ��� */
        	event_parse_enum parse_status;
        };

        event_data eventa_data;
        event_parse_status eventa_status;

        unsigned long CRC32Value(int i);
        unsigned long CalulateSingleCRC32(unsigned char c, unsigned long crc);
        unsigned long CalculateBlockCRC32(unsigned long ulCount,unsigned char *ucBuffer );
        void CRC32ToString(unsigned long crc32, unsigned char* buff);

        void init_event()
        {
        	int i = 0;
        	for( i=0; i<30; ++i)
        	{
        		eventa_status.buff[i] = 0;
        	}
        	eventa_status.dot_count = 0;
        	eventa_status.parse_status = EVENT_PARSE_NONE;
        	eventa_status.count = 0;
        	eventa_status.crc32 = 0;
        };
        void eventa_process(const char *src, unsigned char index);
        bool parse_eventa(unsigned char c);
        void write_Log_mark_event(void);

        /* end add */

};
