#include "cmd_seq_track.h"

void cmd_seq_reset(CmdSeqTrack *t)
{
    t->cur_seq = 0u;
    t->awaiting = 0u;
    t->last_applied = 0u;   /* CMD_SEQ_NONE */
}

void cmd_seq_on_mit_frame(CmdSeqTrack *t, uint16_t seq)
{
    /* Opening a new window closes the previous one unupdated (its reply, if any,
       didn't arrive before this frame). */
    t->cur_seq = seq;
    t->awaiting = 1u;
}

void cmd_seq_on_other_frame(CmdSeqTrack *t)
{
    t->awaiting = 0u;       /* close the window; a non-MIT reply must not count */
}

void cmd_seq_on_reply(CmdSeqTrack *t)
{
    if (t->awaiting) {
        t->last_applied = t->cur_seq;
        t->awaiting = 0u;
    }
}
