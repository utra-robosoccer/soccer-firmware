/* Host unit tests for cmd_seq_track (the slave's reply-window pairing).
 *
 * Build (see host/tests/test_cmd_seq_track.py):
 *   gcc -std=c11 -I firmware/slave/slave_general/Core/Inc \
 *       firmware/common/test/test_cmd_seq_track.c \
 *       firmware/slave/slave_general/Core/Src/cmd_seq_track.c -o test_cmd_seq_track
 */
#include "cmd_seq_track.h"
#include <stdio.h>

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    fprintf(stderr, "FAIL: %s (%s:%d)\n", (msg), __FILE__, __LINE__); failures++; } } while (0)

int main(void)
{
    CmdSeqTrack t;

    /* reset → nothing applied, no window */
    cmd_seq_reset(&t);
    CHECK(t.last_applied == 0 && t.awaiting == 0, "reset clears state");

    /* normal pairing: frame then reply credits that seq */
    cmd_seq_on_mit_frame(&t, 1);
    cmd_seq_on_reply(&t);
    CHECK(t.last_applied == 1, "seq 1 applied");
    cmd_seq_on_mit_frame(&t, 2);
    cmd_seq_on_reply(&t);
    CHECK(t.last_applied == 2, "seq 2 applied");

    /* dropped reply: seq 2's window closes unupdated when seq 3's frame opens */
    cmd_seq_reset(&t);
    cmd_seq_on_mit_frame(&t, 1); cmd_seq_on_reply(&t);
    CHECK(t.last_applied == 1, "seq 1 applied");
    cmd_seq_on_mit_frame(&t, 2);            /* no reply arrives */
    cmd_seq_on_mit_frame(&t, 3); cmd_seq_on_reply(&t);
    CHECK(t.last_applied == 3, "seq 2 dropped, seq 3 applied, last stays 3");

    /* dropped first frame of a new seq: frame 6 lost (no reply), 7 applies */
    cmd_seq_reset(&t);
    cmd_seq_on_mit_frame(&t, 5); cmd_seq_on_reply(&t);
    CHECK(t.last_applied == 5, "seq 5 applied");
    cmd_seq_on_mit_frame(&t, 6);            /* frame lost → no reply */
    cmd_seq_on_mit_frame(&t, 7); cmd_seq_on_reply(&t);
    CHECK(t.last_applied == 7, "seq 6 never applied, seq 7 applied");

    /* an enable/mode-change reply must NOT count */
    cmd_seq_reset(&t);
    cmd_seq_on_mit_frame(&t, 5); cmd_seq_on_reply(&t);
    CHECK(t.last_applied == 5, "seq 5 applied");
    cmd_seq_on_other_frame(&t);             /* e.g. an enable or mode-change frame */
    cmd_seq_on_reply(&t);                    /* its Type-2 reply */
    CHECK(t.last_applied == 5, "enable/mode reply does not update last_applied");

    /* a stray reply with no open window changes nothing */
    cmd_seq_reset(&t);
    cmd_seq_on_mit_frame(&t, 9); cmd_seq_on_reply(&t);
    cmd_seq_on_reply(&t);                    /* second reply, window already closed */
    CHECK(t.last_applied == 9, "stray reply ignored");

    if (failures) { fprintf(stderr, "%d failure(s)\n", failures); return 1; }
    printf("cmd_seq_track: all tests passed\n");
    return 0;
}
