#ifndef PROTO_COMMON_H
#define PROTO_COMMON_H
/* Paths are relative to this file's location (Core/Inc/).
   ../../../ resolves to firmware/                          */
#include "../../../common/include/protocol.h"
/* Master spans ALL slaves — it uses the system config (NUM_SLAVES + per-slave
   motor counts), not a single slave's motor_user_config.h. */
#include "../../../common/include/master_config.h"
#include "../../../common/include/host_watchdog.h"   /* master host-death dead-man */
#include "../../../common/include/cmd_mailbox.h"     /* host-command mailbox (apply at n+1) */
#endif
