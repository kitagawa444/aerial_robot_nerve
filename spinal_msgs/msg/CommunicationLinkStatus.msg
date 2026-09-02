uint8 STATE_UNKNOWN = 0
uint8 STATE_DISCONNECTED = 1
uint8 STATE_CONNECTING = 2
uint8 STATE_CONNECTED = 3
uint8 STATE_STALE = 4
uint8 STATE_DISABLED = 5
uint8 STATE_ERROR = 6

string name
uint8 state
bool data_stale
bool interface_up
bool ip_assigned
bool transport_matched
uint32 last_receive_age_ms
float32 receive_rate_hz
float32 transmit_rate_hz
uint64 received_count
uint64 transmitted_count
uint64 error_count
int16 rssi_dbm
uint8 quality_percent
string detail
