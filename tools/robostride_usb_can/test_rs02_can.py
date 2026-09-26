import struct
import unittest

from rs02_can import (
    COMM_FEEDBACK,
    COMM_GET_ID,
    COMM_MIT,
    COMM_SET_ID,
    MASTER_ID,
    P_MAX,
    P_MIN,
    T_MAX,
    T_MIN,
    V_MAX,
    V_MIN,
    decode_feedback,
    decode_device_id,
    decode_raw_frame,
    encode_disable,
    encode_enable,
    encode_change_motor_mode,
    encode_get_motor_id,
    encode_mit,
    encode_raw_frame,
    encode_set_motor_id,
    encode_set_zero,
    make_id,
    parse_id,
    RUN_MODE_MIT,
)
from rs02_motor import sine_setpoint


def midpoint(lo: float, hi: float) -> int:
    return int((0.0 - lo) * 65535 / (hi - lo))


class RS02CanTest(unittest.TestCase):
    def test_mit_frame_uses_extended_id_layout(self) -> None:
        arb_id, payload = encode_mit(3, pos=0.0, vel=0.0, kp=0.0, kd=0.0, torque=0.0)

        mode, data, node_id = parse_id(arb_id)
        self.assertEqual(mode, COMM_MIT)
        self.assertEqual(node_id, 3)
        self.assertEqual(data, midpoint(T_MIN, T_MAX))
        self.assertEqual(
            payload,
            struct.pack(
                ">HHHH",
                midpoint(P_MIN, P_MAX),
                midpoint(V_MIN, V_MAX),
                0,
                0,
            ),
        )

    def test_get_motor_id_uses_type_0_master_id(self) -> None:
        arb_id, payload = encode_get_motor_id(9)
        mode, data, node_id = parse_id(arb_id)

        self.assertEqual(mode, COMM_GET_ID)
        self.assertEqual(data, MASTER_ID)
        self.assertEqual(node_id, 9)
        self.assertEqual(payload, bytes(8))

    def test_device_id_decode(self) -> None:
        arb_id = make_id(COMM_GET_ID, node_id=MASTER_ID, data=4)
        payload = bytes.fromhex("08 07 06 05 04 03 02 01")

        device_id = decode_device_id(arb_id, payload)
        self.assertIsNotNone(device_id)
        assert device_id is not None
        self.assertEqual(device_id.motor_id, 4)
        self.assertEqual(device_id.mcu_id, 0x0102030405060708)

    def test_enable_disable_and_zero_use_master_id(self) -> None:
        for encoder in (encode_enable, encode_disable, encode_set_zero):
            arb_id, _ = encoder(7)
            _, data, node_id = parse_id(arb_id)
            self.assertEqual(node_id, 7)
            self.assertEqual(data, MASTER_ID)

        _, zero_payload = encode_set_zero(7)
        self.assertEqual(zero_payload, bytes([1]) + bytes(7))

    def test_change_motor_mode_writes_run_mode_register(self) -> None:
        arb_id, payload = encode_change_motor_mode(7, RUN_MODE_MIT)
        mode, data, node_id = parse_id(arb_id)

        self.assertEqual(mode, 18)
        self.assertEqual(data, MASTER_ID)
        self.assertEqual(node_id, 7)
        self.assertEqual(payload, bytes([0x05, 0x70, 0, 0, RUN_MODE_MIT, 0, 0, 0]))

    def test_set_motor_id_uses_type_7_data_field(self) -> None:
        arb_id, payload = encode_set_motor_id(1, 2)
        mode, data, node_id = parse_id(arb_id)

        self.assertEqual(mode, COMM_SET_ID)
        self.assertEqual(node_id, 1)
        self.assertEqual(data, (2 << 8) | MASTER_ID)
        self.assertEqual(payload, bytes(8))

    def test_raw_ch341_frame_round_trip(self) -> None:
        arb_id, payload = encode_mit(3, pos=0.0, vel=0.0, kp=0.0, kd=0.0, torque=0.0)
        raw = encode_raw_frame(arb_id, payload)
        frame = decode_raw_frame(b"\x00noise" + raw)

        self.assertIsNotNone(frame)
        assert frame is not None
        self.assertEqual(frame.arb_id, arb_id)
        self.assertEqual(frame.data, payload)

    def test_feedback_decode(self) -> None:
        data_field = 4 | (0b000010 << 8) | (2 << 14)
        arb_id = make_id(COMM_FEEDBACK, node_id=MASTER_ID, data=data_field)
        payload = struct.pack(
            ">HHHH",
            midpoint(P_MIN, P_MAX),
            midpoint(V_MIN, V_MAX),
            midpoint(T_MIN, T_MAX),
            425,
        )

        fb = decode_feedback(arb_id, payload)
        self.assertIsNotNone(fb)
        assert fb is not None
        self.assertEqual(fb.motor_id, 4)
        self.assertEqual(fb.faults, 0b000010)
        self.assertEqual(fb.mode, 2)
        self.assertAlmostEqual(fb.temp, 42.5)

    def test_sine_setpoint_velocity_is_position_derivative(self) -> None:
        center = 1.0
        amplitude = 0.5
        frequency = 0.5

        pos, vel = sine_setpoint(center, amplitude, frequency, 0.5, duration=4.0, ramp_time=0.0)

        self.assertAlmostEqual(pos, center + amplitude, places=6)
        self.assertAlmostEqual(vel, 0.0, places=6)

    def test_sine_setpoint_ramp_starts_and_ends_at_center(self) -> None:
        start_pos, start_vel = sine_setpoint(1.0, 0.5, 0.5, 0.0, duration=4.0, ramp_time=1.0)
        end_pos, end_vel = sine_setpoint(1.0, 0.5, 0.5, 4.0, duration=4.0, ramp_time=1.0)

        self.assertAlmostEqual(start_pos, 1.0, places=6)
        self.assertAlmostEqual(start_vel, 0.0, places=6)
        self.assertAlmostEqual(end_pos, 1.0, places=6)
        self.assertAlmostEqual(end_vel, 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
