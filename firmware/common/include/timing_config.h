/* Timing, rates & firmware tuning — HAND-WRITTEN FIRMWARE CONSTANTS.
 *
 * These are coupled to firmware behavior (tick derivations, watchdog cycle
 * counts, DMA arm deadlines), so they are not robot configuration: changing a
 * value here IS a firmware change — edit, rebuild, reflash (and rerun
 * scripts/gen_motor_config.py: the host reads the rates from this file).
 * Derived constants are computed by the preprocessor below; change the rate,
 * not the derived value, so real-world durations stay fixed.
 */
#ifndef TIMING_CONFIG_H
#define TIMING_CONFIG_H

/* ── Rates (Hz) — reported in MasterStatus and the .bin log header ─────────── */
#define MASTER_POLL_HZ  200u   /* SPI poll / command delivery                  */
#define TELEMETRY_HZ    200u   /* MSG_ROBOT_TELE emission                      */
#define SLAVE_TICK_HZ   200u   /* slave control tick (motor_runtime_update)    */
#define HOST_CMD_HZ     200u   /* expected host command rate (run_policy)      */

/* Master SPI1 clock divider off APB2 (72 MHz): MX_SPI1_Init maps it to the
 * SPI_BAUDRATEPRESCALER_* enum. Valid: 2,4,8,16,32,64,128,256. All of 64/32/
 * 16/8 are 2-min clean on the bench (short wiring); 16 = 4.5 MHz is two steps
 * of margin below the fastest tested (div 8), and the slave SPI resync
 * recovers isolated glitches — re-validate on the robot harness via the
 * CRC/resync counters. */
#define MASTER_SPI_PRESCALER_DIV  16u

/* ── Derived loop periods — do not hand-edit; change the rate instead ────────
 * Whole-ms periods floor (a 400 Hz tick counts as 2 ms); µs-precise paths use
 * MASTER_CYCLE_US. */
#define MASTER_CYCLE_US        (1000000u / MASTER_POLL_HZ)   /* one poll cycle */
#define MASTER_POLL_PERIOD_MS  (1000u / MASTER_POLL_HZ)
#define MASTER_TELE_PERIOD_MS  (1000u / TELEMETRY_HZ)
#define MOTOR_LOOP_PERIOD_MS   (1000u / SLAVE_TICK_HZ)
#define MOTOR_LOOP_DT_S        ((float)MOTOR_LOOP_PERIOD_MS * 0.001f)

/* ── Timeouts / debounces (ms) and tick counts derived from them ───────────── */
#define MOTOR_CAN_FB_TIMEOUT_MS  100u  /* Type-2 staleness → DRVF_CAN_TIMEOUT / ABSENT */
#define MOTOR_ENABLE_MON_MS       15u  /* not-Run dwell while enabled → DRVF_NOT_ENABLED */
#define MOTOR_ENABLE_TIMEOUT_MS  100u  /* ENABLING without rs_mode==Run → DRVF_ENABLE_FAILED */
#define MOTOR_CLEAR_TIMEOUT_MS   100u  /* CLEARING without a fresh reply → ABSENT */
#define CAN_ID_SWEEP_PROBE_GAP_MS  2u  /* reply window after each boot id-sweep
                                          Type-0 probe (replies land ~0.85 ms;
                                          an unpaced blast loses probes on some
                                          units — the verdict latches until
                                          reboot) */
#define BOOT_ZERO_STA_WINDOW_MS  100u  /* Type-17 reply collection per round of
                                          the boot zero_sta check (≤3 rounds) */
#define BOOT_FW_VER_WINDOW_MS      5u  /* reply window per fw-version read
                                          attempt (≤3 per motor; replies land
                                          well under 1 ms on a quiet bus) */

/* Enable-monitor dwell in slave ticks (rounded). */
#define MOTOR_ENABLE_MON_K \
    ((MOTOR_ENABLE_MON_MS + MOTOR_LOOP_PERIOD_MS / 2u) / MOTOR_LOOP_PERIOD_MS)

/* ── Dead-man watchdogs (task 6) ──────────────────────────────────────────────
 * Host-death is counted on the master in poll cycles; the slave master-loss
 * ramp is in ms (HAL_GetTick). Host timeout = 3 host periods, floored at 25 ms. */
#define HOST_CMD_TIMEOUT_RAW_MS  ((3u * 1000u + HOST_CMD_HZ / 2u) / HOST_CMD_HZ)
#define HOST_CMD_TIMEOUT_MS \
    (HOST_CMD_TIMEOUT_RAW_MS > 25u ? HOST_CMD_TIMEOUT_RAW_MS : 25u)
#define HOST_LOST_CYCLES_RAW \
    ((HOST_CMD_TIMEOUT_MS * MASTER_POLL_HZ + 500u) / 1000u)
#define HOST_LOST_CYCLES  (HOST_LOST_CYCLES_RAW > 1u ? HOST_LOST_CYCLES_RAW : 1u)

#define MASTER_LOST_GRACE_MS   50u   /* slave: hold (v/tau=0) after exchanges stop */
#define MASTER_LOST_DAMP_MS   300u   /* stop-ramp damp dwell (STOPPING; also master
                                        host-death dwell)                          */

/* Slave SPI TX-arm deadline (µs after an exchange): wait for the motor replies,
 * then arm the next exchange's DMA so the fresh reply rides it (removes the
 * one-exchange telemetry lag). 0.6 cycle — after the ≤2 ms reply window,
 * comfortably before the next NSS. */
#define TX_ARM_DEADLINE_US  ((6u * MASTER_CYCLE_US) / 10u)

/* ── Motion-safety guards (firmware tuning, not per-motor identity) ─────────── */
#define MOTOR_FAULT_CHECK_DISABLED 0u    /* 1 = vendor-bit/Cali check in DISABLED
                                            (masked off until bench boot-rest data
                                            is clean — protocol_v9.md §8)         */

/* Whole-ms derivations above need ≥1 ms periods. */
#if (MASTER_POLL_HZ > 1000u) || (TELEMETRY_HZ > 1000u) || (SLAVE_TICK_HZ > 1000u)
#error "rates above 1000 Hz break the whole-ms period derivations"
#endif

#endif /* TIMING_CONFIG_H */
