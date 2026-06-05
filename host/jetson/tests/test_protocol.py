import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from protocol import (
    MOTOR_CMD_SIZE,
    MOTOR_STATE_SIZE,
    MsgType,
    NodeId,
    MotorCmd,
    MotorState,
    ProtocolError,
    decode_frame,
    encode_message,
)


class ProtocolRoundTripTest(unittest.TestCase):
    def assert_round_trip(self, msg_type, payload=b""):
        frame = encode_message(
            msg_type,
            seq=int(msg_type),
            source=NodeId.JETSON,
            target=NodeId.MASTER,
            ts_us=123456 + int(msg_type),
            payload=payload,
        )

        self.assertTrue(frame.endswith(b"\x00"))
        decoded = decode_frame(frame)

        self.assertEqual(decoded.header.type, msg_type)
        self.assertEqual(decoded.header.seq, int(msg_type))
        self.assertEqual(decoded.header.source, NodeId.JETSON)
        self.assertEqual(decoded.header.target, NodeId.MASTER)
        self.assertEqual(decoded.header.ts_us, 123456 + int(msg_type))
        self.assertEqual(decoded.header.length, len(payload))
        self.assertEqual(decoded.payload, payload)

    def test_round_trips_every_message_type(self):
        payloads = {
            MsgType.PING: b"",
            MsgType.DISCOVER: b"",
            MsgType.MOTOR_STATE_REQ: b"",
            MsgType.ARM_HOLD: MotorCmd(pos=1.0, vel=0.0, kp=15.0, kd=1.0, tau=0.0).pack(),
            MsgType.DISABLE: b"",
            MsgType.MOTOR_STATE: MotorState(
                pos=1.25,
                vel=-0.5,
                tau=0.2,
                temp=36.5,
                fault=0x00000004,
                last_cmd_seq=44,
            ).pack(),
        }

        for msg_type, payload in payloads.items():
            with self.subTest(msg_type=msg_type):
                self.assert_round_trip(msg_type, payload)

    def test_motor_cmd_payload_size(self):
        payload = MotorCmd(pos=1.0, vel=2.0, kp=3.0, kd=4.0, tau=5.0).pack()
        self.assertEqual(len(payload), MOTOR_CMD_SIZE)
        self.assertEqual(MotorCmd.unpack(payload), MotorCmd(1.0, 2.0, 3.0, 4.0, 5.0))

    def test_motor_state_payload_size(self):
        state = MotorState(pos=1.0, vel=2.0, tau=3.0, temp=4.0, fault=5, last_cmd_seq=6)
        payload = state.pack()
        self.assertEqual(len(payload), MOTOR_STATE_SIZE)
        self.assertEqual(MotorState.unpack(payload), state)

    def test_crc_failure_is_rejected(self):
        frame = bytearray(
            encode_message(
                MsgType.PING,
                seq=99,
                source=NodeId.JETSON,
                target=NodeId.MASTER,
                ts_us=1,
            )
        )
        frame[-2] ^= 0x01
        with self.assertRaises(ProtocolError):
            decode_frame(bytes(frame))


if __name__ == "__main__":
    unittest.main()
