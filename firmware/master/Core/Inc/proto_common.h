#ifndef PROTO_COMMON_H
#define PROTO_COMMON_H
/* Paths are relative to this file's location (Core/Inc/).
   ../../../ resolves to firmware/                          */
#include "../../../common/include/protocol.h"
/* Master spans ALL slaves — it uses the system config (per-slave counts +
   CAN-id LUTs + global transport bounds), not a single slave's motor_config.h. */
#include "../../../common/include/system_config.h"
#include "../../../common/include/host_watchdog.h"   /* master host-death dead-man (task 6) */
#include "../../../common/include/cmd_mailbox.h"     /* host-command mailbox (apply at n+1) */
#include "../../../common/include/spi_gate.h"        /* SPI version gate (v9.1)             */
#endif
