import struct
import unittest
from unittest.mock import patch
import udp_motor as protocol


def response(sequence=1, command=0, index=0):
    data = bytearray(56)
    struct.pack_into('<4sIBBBBBBBb', data, 0, b'ABR1', sequence, command,
                     index, 0, 3, 0, 1, 5, 3)
    struct.pack_into('<HHiiIIIIII', data, 16, 0x1237, 0, -10, -8,
                     0x06020000, 42, 41, 0x100030, 1234, 1000)
    data[52] = 1
    return bytes(data)


class ProtocolTests(unittest.TestCase):
    def test_direction_selection_and_signed_limits(self):
        for direction, sign in [('Forward (+)', 1), ('Reverse (-)', -1)]:
            self.assertEqual(protocol.direction_velocity('250', direction), sign * 250)
            self.assertEqual(protocol.direction_velocity('0', direction), 0)
        self.assertEqual(protocol.direction_velocity(str(2**31), 'Reverse (-)'), -(2**31))
        for speed, direction in [('-250', 'Forward (+)'), ('x', 'Reverse (-)'),
                                  (str(2**31), 'Forward (+)'), (str(2**31+1), 'Reverse (-)'), ('1', 'CW')]:
            with self.assertRaises(ValueError):
                protocol.direction_velocity(speed, direction)

    def test_reversal_diagnostic_flags(self):
        frame = bytearray(response(1, 9)) + bytearray(112)
        frame[103] = 0x81
        parsed = protocol.decode(bytes(frame), 1, 9, 0)
        self.assertTrue(parsed['reversal_supported'])
        self.assertTrue(parsed['reversal_pending'])
        frame[103] = 0
        parsed = protocol.decode(bytes(frame), 1, 9, 0)
        self.assertFalse(parsed['reversal_supported'])
        self.assertEqual(protocol.ERRORS[12], 'REVERSAL_TIMEOUT')

    def test_pdo_diagnostics_and_error_label(self):
        frame = bytearray(response(1, 9)) + bytearray(112)
        struct.pack_into('<H', frame, 130, 1)
        struct.pack_into('<I', frame, 164, 12345)
        frame[12] = 11
        parsed = protocol.decode(bytes(frame), 1, 9, 0)
        self.assertEqual(parsed['pdo_ready'], 1)
        self.assertEqual(parsed['pdo_rx'], 12345)
        self.assertEqual(protocol.label(protocol.ERRORS, parsed['error']), 'PDO_TIMEOUT')

    def test_board_identity_reply_fields(self):
        for board_id in range(1, 5):
            frame = bytearray(response())
            frame[53:56] = bytes((board_id, board_id - 1, 3))
            parsed = protocol.decode(bytes(frame), 1, 0, 0)
            self.assertEqual(parsed['board_id'], board_id)
            self.assertEqual(parsed['board_raw'], board_id - 1)
            self.assertEqual(parsed['board_flags'], 3)

    @staticmethod
    def hand_sample():
        return dict(state=1, statusword=0x40, valid=0x1fff, hall=1, motor_ms=1000,
                    hall_ms=990, position_ms=990, statusword_ms=990, node=1,
                    position=0, hall_edges=0, hall_skipped=0, hall_invalid=0,
                    pole_pairs=1, encoder_ppr=1024, position_unit=0xb50000)

    def test_hand_turn_candidates(self):
        start = self.hand_sample()
        end = dict(start, motor_ms=31000, hall_ms=30990, position_ms=30990,
                   statusword_ms=30990, position=4096, hall_edges=42)
        report = protocol.hand_turn_report(start, end)
        self.assertIn('Pole-pair candidate=7', report)
        self.assertIn('Encoder PPR candidate=1024', report)
        end['position'] = -4096
        self.assertIn('Encoder PPR candidate=1024', protocol.hand_turn_report(start, end))

    def test_hand_turn_rejects_active_stale_and_missed_edges(self):
        start = self.hand_sample()
        for bad in (dict(start, state=3), dict(start, statusword=0x27), dict(start, hall_ms=1), dict(start, hall=7)):
            with self.assertRaises(ValueError):
                protocol.validate_hand_sample(bad)
        end = dict(start, motor_ms=1100, hall_edges=6, hall_skipped=1)
        self.assertIn('inconclusive', protocol.hand_turn_report(start, end))

    def test_hall_extension_wire(self):
        frame = bytearray(response(20, 9, 0)) + bytearray(112)
        struct.pack_into('<HHIIIIIIII', frame, 128, 5, 0, 900, 42, 0, 0, 1024, 7, 0xb50000, 901)
        parsed = protocol.decode(bytes(frame), 20, 9, 0)
        self.assertEqual(parsed['hall'], 5)
        self.assertEqual(parsed['hall_edges'], 42)
        self.assertEqual(parsed['encoder_ppr'], 1024)
        self.assertEqual(parsed['pole_pairs'], 7)
        self.assertEqual(parsed['statusword_ms'], 901)

    def test_velocity_has_no_100_rpm_cap_and_preserves_int32(self):
        for velocity in (101, 500, 1000, -1000, -(2**31), 2**31 - 1):
            packet = protocol.encode(10, 2, 0, velocity)
            self.assertEqual(struct.unpack_from('<i', packet, 12)[0], velocity)
        for velocity in (-(2**31) - 1, 2**31):
            with self.assertRaises(struct.error):
                protocol.encode(10, 2, 0, velocity)

    def test_extended_feedback_signed_units_and_offsets(self):
        frame = bytearray(response(55, 8, 0)) + bytearray(72)
        struct.pack_into('<iiHhiiiiIIIIHBBIIIIII', frame, 56,
                         10, 10, 15, -150, 240, -220, -10001, -4,
                         511, 0, 100, 0, 0, 0, 0, 101, 102, 120, 103, 104, 105)
        result = protocol.decode(bytes(frame), 55, 8, 0)
        self.assertEqual(result['current_actual'], -220)
        self.assertEqual(result['position'], -10001)
        self.assertEqual(result['position_delta'], -4)
        self.assertEqual(result['torque'], -150)
        self.assertEqual(result['valid'], 511)
        self.assertEqual(result['motor_ms'] - result['position_ms'], 20)
        self.assertEqual(result['controlword'], 15)
        self.assertEqual(result['drive_target_ms'], 105)

    def test_reset_command_and_resetting_state(self):
        packet = protocol.encode(22, 7, 0)
        self.assertEqual(packet[8], 7)
        frame = bytearray(response(22, 7, 0))
        frame[11] = 7
        parsed = protocol.decode(bytes(frame), 22, 7, 0)
        self.assertEqual(protocol.label(protocol.STATES, parsed['state']), 'RESETTING')

    def test_signed_command_and_wire_offsets(self):
        packet = protocol.encode(0xffffffff, 2, 1, -100)
        self.assertEqual(len(packet), 16)
        self.assertEqual(packet[:4], b'ABS1')
        self.assertEqual(packet[8:12], bytes([2, 1, 0, 0]))
        self.assertEqual(struct.unpack_from('<i', packet, 12)[0], -100)

    def test_full_reply(self):
        parsed = protocol.decode(response(), 1, 0, 0)
        self.assertEqual(parsed['target'], -10)
        self.assertEqual(parsed['actual'], -8)
        self.assertEqual(parsed['abort'], 0x06020000)
        self.assertEqual(parsed['esr'], 0x100030)
        self.assertEqual(parsed['lease'], 1000)
        self.assertEqual(parsed['count'], 1)

    def test_wrong_reply_is_ignored(self):
        self.assertIsNone(protocol.decode(response(2), 1, 0, 0))
        self.assertIsNone(protocol.decode(response(command=2), 1, 0, 0))
        self.assertIsNone(protocol.decode(response(index=1), 1, 0, 0))

    def test_bad_frames(self):
        for frame in [b'', response()[:-1], b'BAD!' + response()[4:], response()+b'X']:
            with self.assertRaises(ValueError):
                protocol.decode(frame, 1, 0, 0)

    def test_client_ignores_delayed_reply_without_retransmitting(self):
        class FakeSocket:
            def __init__(self): self.sent = []
            def send(self, value): self.sent.append(value)
            def settimeout(self, value): pass
            def recv(self, length): return frames.pop(0)
        client = protocol.Client.__new__(protocol.Client)
        client.sock, client.sequence = FakeSocket(), 0
        frames = [response(0), response(1)]
        self.assertEqual(client.request(0)['state'], 3)
        self.assertEqual(len(client.sock.sent), 1)

    def test_sequence_wrap(self):
        class FakeSocket:
            def send(self, value): self.packet = value
            def settimeout(self, value): pass
            def recv(self, length): return response(0)
        client = protocol.Client.__new__(protocol.Client)
        client.sock, client.sequence = FakeSocket(), 0xffffffff
        client.request(0)
        self.assertEqual(struct.unpack_from('<I', client.sock.packet, 4)[0], 0)


if __name__ == '__main__':
    unittest.main()
