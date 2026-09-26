#ifndef PROTO_COMMON_H
#define PROTO_COMMON_H
/* Paths are relative to this file's location (Core/Inc/).
   ../../../ resolves to firmware/                          */
#include "../../../common/include/protocol.h"
/* Master spans ALL slaves — it uses the system config (per-slave counts +
   CAN-id LUTs + global transport bounds), not a single slave's motor_config.h. */
#include "../../../common/include/system_config.h"
#endif
