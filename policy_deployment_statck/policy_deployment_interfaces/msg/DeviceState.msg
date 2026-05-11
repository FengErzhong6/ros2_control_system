uint8 LIFECYCLE_UNKNOWN=0
uint8 LIFECYCLE_IDLE=1
uint8 LIFECYCLE_STARTING=2
uint8 LIFECYCLE_READY=3
uint8 LIFECYCLE_ACTIVE=4
uint8 LIFECYCLE_STOPPING=5
uint8 LIFECYCLE_FAULT=6

uint8 HEALTH_UNKNOWN=0
uint8 HEALTH_OK=1
uint8 HEALTH_DEGRADED=2
uint8 HEALTH_FAILED=3

string device_id
string device_class
uint8 lifecycle_state
uint8 health_state
bool is_required
builtin_interfaces/Time last_ready_stamp
string summary
