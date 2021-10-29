#
# https://exiftool.org/TagNames/GPS.html
#
# Tag ID	Tag Name	Writable	Values / Notes
# 0x0000	GPSVersionID	int8u[4]:
uint8[4] gps_version_id
# 0x0001	GPSLatitudeRef	string[2]	(tags 0x0001-0x0006 used for camera location according to MWG 2.0. ExifTool will also accept a number when writing GPSLatitudeRef, positive for north latitudes or negative for south, or a string containing N, North, S or South)
# 						'N' = North
# 						'S' = South
string gps_latitude_ref
# 0x0002	GPSLatitude	rational64u[3]
float64[3] gps_latitude
# 0x0003	GPSLongitudeRef	string[2]	(ExifTool will also accept a number when writing this tag, positive for east longitudes or negative for west, or a string containing E, East, W or West)
# 						'E' = East
# 						'W' = West
string gps_longitude_ref
# 0x0004	GPSLongitude	rational64u[3]
float64[3] gps_longitude
# 0x0005	GPSAltitudeRef	int8u	(ExifTool will also accept number when writing this tag, with negative numbers indicating below sea level)
uint8 gps_altitude_ref
# 						0 = Above Sea Level
# 						1 = Below Sea Level
# 0x0006	GPSAltitude	rational64u
float64 gps_altitude
# 0x0007	GPSTimeStamp	rational64u[3]	(UTC time of GPS fix. When writing, date is stripped off if present, and time is adjusted to UTC if it includes a timezone)
float64 gps_time_stamp
# 0x0008	GPSSatellites	string
string gps_satellites
# 0x0009	GPSStatus	string[2]
# 						'A' = Measurement Active
# 						'V' = Measurement Void
string gps_status
# 0x000a	GPSMeasureMode	string[2]
# 						2 = 2-Dimensional Measurement
# 						3 = 3-Dimensional Measurement
string gps_measure_mode
# 0x000b	GPSDOP	rational64u
float64 gpf_sdop
# 0x000c	GPSSpeedRef	string[2]
# 						'K' = km/h
# 						'M' = mph
# 						'N' = knots
string gps_speed_ref
# 0x000d	GPSSpeed	rational64u
float64 gps_speed
# 0x000e	GPSTrackRef	string[2]
# 						'M' = Magnetic North
# 						'T' = True North
string gps_track_ref
# 0x000f	GPSTrack	rational64u
float64 gps_track
# 0x0010	GPSImgDirectionRef	string[2]
# 						'M' = Magnetic North
# 						'T' = True North
string gps_img_direction_ref
# 0x0011	GPSImgDirection	rational64u
float64 gps_img_direction
# 0x0012	GPSMapDatum	string
string gps_map_datum
# 0x0013	GPSDestLatitudeRef	string[2]	(tags 0x0013-0x001a used for subject location according to MWG 2.0)
# 						'N' = North
# 						'S' = South
string gps_dest_latitude_ref
# 0x0014	GPSDestLatitude	rational64u[3]
float64[3] gps_dest_latitude
# 0x0015	GPSDestLongitudeRef	string[2]
# 						'E' = East
# 						'W' = West
string gps_dest_longitude_ref
# 0x0016	GPSDestLongitude	rational64u[3]
float64[3] gps_dest_longitude
# 0x0017	GPSDestBearingRef	string[2]
# 						'M' = Magnetic North
# 						'T' = True North
string gps_dest_bearing_ref
# 0x0018	GPSDestBearing	rational64u
float64 gps_dest_bearing
# 0x0019	GPSDestDistanceRef	string[2]
# 						'K' = Kilometers
# 						'M' = Miles
# 						'N' = Nautical Miles
string gps_dest_distance_ref
# 0x001a	GPSDestDistance	rational64u
float64 gps_dest_distance
# 0x001b	GPSProcessingMethod	undef	(values of "GPS", "CELLID", "WLAN" or "MANUAL" by the EXIF spec.)
# 0x001c	GPSAreaInformation	undef
# 0x001d	GPSDateStamp	string[11]	(when writing, time is stripped off if present, after adjusting date/time to UTC if time includes a timezone. Format is YYYY:mm:dd)
string gps_date_stamp
# 0x001e	GPSDifferential	int16u
# 						0 = No Correction
# 						1 = Differential Corrected
uint16 gps_differential
# 0x001f	GPSHPositioningError	rational64u
float64 gps_hpositioning_error

