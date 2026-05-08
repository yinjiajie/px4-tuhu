uint64 timestamp

bool connected                         # true when a valid reply has been received recently
float32 height_above_takeoff_m         # distance above takeoff point, decoded from bits[15:0] with 0.1 m resolution
float32 voltage_v                      # module voltage, decoded from bits[21:16] with 0.1 V resolution
uint8 release_source                   # bits[23:22]
uint8 flags                            # bits[27:24]
uint8 state                            # bits[31:28]
uint32 raw                             # raw 32-bit status word

# TOPICS parachute_status
