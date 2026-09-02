uint8 STATE_UNKNOWN = 0
uint8 STATE_DISCONNECTED = 1
uint8 STATE_CONNECTING = 2
uint8 STATE_CONNECTED = 3
uint8 STATE_STALE = 4
uint8 STATE_DISABLED = 5
uint8 STATE_ERROR = 6

uint8 state
uint32 pending_count
uint32 oldest_pending_age_ms
uint64 received_count
uint64 retry_count
uint64 timeout_count
uint64 duplicate_count
uint64 rejected_count
uint32 last_rtt_ms
uint16 last_message_id
uint32 last_request_id
string detail
