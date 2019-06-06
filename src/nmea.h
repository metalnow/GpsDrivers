/** @file NMEA protocol definitions */

#pragma once

#include "gps_helper.h"
#include "base_station.h"
#include "../../definitions.h"
#include <cmath>

class RTCMParsing;

#define NMEA_RECV_BUFFER_SIZE 512

#define ASH_RESPONSE_TIMEOUT	200		// ms, timeout for waiting for a response

class GPSDriverNMEA : public GPSBaseStationSupport
{
public:
	/**
	 * @param heading_offset heading offset in radians [-pi, pi]. It is substracted from the measurement.
	 */
	GPSDriverNMEA(GPSCallbackPtr callback, void *callback_user, struct vehicle_gps_position_s *gps_position,
			 struct satellite_info_s *satellite_info, float heading_offset = 0.f);
	virtual ~GPSDriverNMEA();

	int receive(unsigned timeout) override;
	int configure(unsigned &baudrate, OutputMode output_mode) override;

private:
	enum class NMEADecodeState {
		uninit,
		got_sync1,
		got_nmea,
		got_first_cs_byte
	};

	int handleMessage(int len);
	int parseChar(uint8_t b);

	struct satellite_info_s*_satellite_info;
	struct vehicle_gps_position_s *_gps_position;
	uint64_t _last_timestamp_time;
	int _nmea_fd;
	NMEADecodeState _decode_state{NMEADecodeState::uninit};
	uint8_t _rx_buffer[NMEA_RECV_BUFFER_SIZE];
	uint16_t _rx_buffer_bytes;
	bool _got_pashr_pos_message;
	bool _parse_error;
	char *_parse_pos;
	float _heading_offset;
};

