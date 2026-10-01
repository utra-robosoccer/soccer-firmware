/* cmd_seq_track — pairs a motor's MIT command frames with their replies to decide
 * which host cmd_seq was actually APPLIED (reached the motor and got a reply).
 *
 * Pure, HAL-free, so it is unit-testable. Rules:
 *   - A MIT (Type-1) frame opens a reply window carrying its cmd_seq.
 *   - A reply (fresh Type-2) that arrives while a window is open sets last_applied to
 *     that window's seq and closes it.
 *   - Sending the next MIT frame (or ANY other frame type, e.g. enable/mode-change)
 *     closes the current window WITHOUT updating last_applied — so a frame whose reply
 *     didn't arrive in its window never counts, and an enable/mode reply never counts.
 *   - last_applied is CMD_SEQ_NONE(0) after a reset (arm / goto-zero / disable), and is
 *     frozen at the last applied seq while no new MIT is being confirmed (HOLD/IDLE).
 */
#ifndef CMD_SEQ_TRACK_H
#define CMD_SEQ_TRACK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t cur_seq;       /* cmd_seq of the in-flight MIT frame */
    uint8_t  awaiting;      /* 1 = a MIT frame is awaiting its reply */
    uint16_t last_applied;  /* last cmd_seq confirmed applied (0 = none) */
} CmdSeqTrack;

/* Reset to "none applied", no open window (call on arm / goto-zero / disable). */
void cmd_seq_reset(CmdSeqTrack *t);

/* A MIT (Type-1) frame carrying cmd_seq `seq` was just sent to the motor. */
void cmd_seq_on_mit_frame(CmdSeqTrack *t, uint16_t seq);

/* Any non-MIT frame (enable / mode-change / set-zero / disable / read) was sent —
 * closes the current window without crediting it. */
void cmd_seq_on_other_frame(CmdSeqTrack *t);

/* A fresh Type-2 reply arrived; credit the open window (if any). */
void cmd_seq_on_reply(CmdSeqTrack *t);

#ifdef __cplusplus
}
#endif

#endif /* CMD_SEQ_TRACK_H */
