"""Offline protocol/lifecycle tests. No camera, bridge or serial device is opened."""
import contextlib
import io
import json
import socket
import unittest
from unittest import mock

import test_straight_mechanical as straight


FRAME = b'{"Img":"fake-jpeg"}'


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeSocket:
    def __init__(self, responses=(), clock=None, recv_delay=0):
        self.responses = list(responses)
        self.pending = []
        self.sent = []
        self.timeouts = []
        self.closed = False
        self.clock = clock
        self.recv_delay = recv_delay

    def settimeout(self, seconds):
        self.timeouts.append(seconds)

    def sendall(self, command):
        if self.pending:
            raise AssertionError("next command sent before full response drained")
        self.sent.append(command)
        self.pending = list(self.responses.pop(0)) if self.responses else [FRAME]

    def recv(self, count):
        if self.clock:
            self.clock.sleep(self.recv_delay)
        if not self.pending:
            return b""
        chunk = self.pending.pop(0)
        if isinstance(chunk, BaseException):
            raise chunk
        if len(chunk) > count:
            self.pending.insert(0, chunk[count:])
            chunk = chunk[:count]
        return chunk

    def close(self):
        self.closed = True


class FrameTests(unittest.TestCase):
    def read(self, chunks, max_bytes=straight.MAX_FRAME_BYTES, clock=None, delay=0):
        clock = clock or FakeClock()
        sock = FakeSocket(clock=clock, recv_delay=delay)
        sock.pending = list(chunks)
        return straight.recv_frame(sock, 1.0, max_bytes=max_bytes, clock=clock)

    def test_fragmented_strings_nested_objects_and_unicode(self):
        payload = {"Img": "a}b\\\"{c", "meta": [{"message": "xe chạy"}], "blocked": False}
        frame = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.assertEqual(self.read([bytes([byte]) for byte in frame]), payload)

    def test_whitespace(self):
        self.assertEqual(self.read([b" \n", FRAME + b"\r\n"]), {"Img": "fake-jpeg"})

    def test_oversized_frame(self):
        with self.assertRaises(straight.FrameProtocolError):
            self.read([FRAME], max_bytes=10)

    def test_disconnect_mid_frame(self):
        with self.assertRaises(ConnectionError):
            self.read([b'{"Img":"unfinished'])

    def test_slow_trickle_has_total_deadline(self):
        with self.assertRaises(socket.timeout):
            self.read([bytes([byte]) for byte in FRAME], delay=0.4)

    def test_final_chunk_cannot_arrive_after_deadline(self):
        with self.assertRaises(socket.timeout):
            self.read([FRAME], delay=1.1)

    def test_rejects_extra_object_and_invalid_frames(self):
        for frame in (FRAME + FRAME, b'[]', b'{"Img":null}', b'{"Img":""}',
                      b'{"Img":}', b'{"Img":"a", "bad": [}}',
                      b'{"Img":"\xff"}'):
            with self.subTest(frame=frame), self.assertRaises(straight.FrameProtocolError):
                self.read([frame])


class ClientTests(unittest.TestCase):
    def test_every_command_consumes_response_before_next_and_stop(self):
        sock = FakeSocket([[FRAME[:5], FRAME[5:]], [FRAME], [FRAME]])
        client = straight.BridgeClient(sock)
        client.exchange(0, 8)
        client.exchange(0, 8)
        self.assertTrue(client.stop_and_close())
        self.assertEqual(sock.sent, [b"0 8", b"0 8", b"0 0"])
        self.assertEqual(sock.pending, [])
        self.assertTrue(sock.closed)

    def test_partial_frame_timeout_disconnects_without_queuing_new_command(self):
        sock = FakeSocket([[b'{"Img":"', socket.timeout("stall")]])
        client = straight.BridgeClient(sock)
        with self.assertRaises(socket.timeout):
            client.exchange(0, 8)
        with self.assertRaises(straight.FrameProtocolError):
            client.exchange(0, 8)
        self.assertFalse(client.stop_and_close())
        self.assertEqual(sock.sent, [b"0 8"])
        self.assertTrue(sock.closed)

    def test_send_failure_disconnects(self):
        sock = FakeSocket()
        client = straight.BridgeClient(sock)
        sock.sendall = mock.Mock(side_effect=BrokenPipeError("gone"))
        with self.assertRaises(BrokenPipeError):
            client.exchange(0, 8)
        self.assertFalse(client.stop_and_close())
        self.assertTrue(sock.closed)

    def test_failed_stop_always_closes(self):
        sock = FakeSocket([[socket.timeout("stop reply lost")]])
        self.assertFalse(straight.BridgeClient(sock).stop_and_close())
        self.assertTrue(sock.closed)

    def test_interrupt_during_frame_closes_without_another_command(self):
        sock = FakeSocket([[FRAME[:5], KeyboardInterrupt()]])
        client = straight.BridgeClient(sock)
        with self.assertRaises(KeyboardInterrupt):
            client.exchange(0, 8)
        self.assertFalse(client.stop_and_close())
        self.assertEqual(sock.sent, [b"0 8"])
        self.assertTrue(sock.closed)

    def test_send_time_counts_toward_frame_deadline(self):
        clock = FakeClock()
        sock = FakeSocket(clock=clock, recv_delay=0.5)
        original_send = sock.sendall

        def slow_send(command):
            original_send(command)
            clock.sleep(0.7)

        sock.sendall = slow_send
        client = straight.BridgeClient(sock, clock=clock)
        with self.assertRaises(socket.timeout):
            client.exchange(0, 8)
        self.assertAlmostEqual(sock.timeouts[-1], 0.3)

    def test_drive_bounded_duration_and_cadence(self):
        clock = FakeClock()
        sock = FakeSocket(clock=clock, recv_delay=0.025)
        client = straight.BridgeClient(sock, clock=clock)
        straight.drive(client, 8, 0.3, clock=clock, sleep=clock.sleep)
        self.assertAlmostEqual(clock.now, 0.3)
        self.assertEqual(sock.sent, [b"0 8"] * 3)

    def test_drive_does_not_wait_past_duration_for_last_frame(self):
        clock = FakeClock()
        sock = FakeSocket(clock=clock, recv_delay=0.25)
        client = straight.BridgeClient(sock, clock=clock)
        with self.assertRaises(socket.timeout):
            straight.drive(client, 8, 0.2, clock=clock, sleep=clock.sleep)
        self.assertLessEqual(max(sock.timeouts), 0.2)

    def test_blocked_and_lifted_abort_measurement(self):
        for flag in ("blocked", "lifted"):
            with self.subTest(flag=flag), self.assertRaises(straight.FrameProtocolError):
                straight.check_safety({"Img": "image", flag: True})


class MainTests(unittest.TestCase):
    def run_main(self, sock, drive_effect=None, connect_effect=None):
        output = io.StringIO()
        with mock.patch("builtins.input", return_value=""), \
                mock.patch.object(straight.socket, "create_connection", return_value=sock,
                                  side_effect=connect_effect), \
                mock.patch.object(straight, "drive", side_effect=drive_effect), \
                contextlib.redirect_stdout(output):
            status = straight.main()
        return status, output.getvalue()

    def test_complete_sequence_gets_measurement_prompt(self):
        sock = FakeSocket()
        status, output = self.run_main(sock)
        self.assertEqual(status, 0)
        self.assertIn("Khi xe da dung, do do lech", output)
        self.assertEqual(sock.sent, [b"0 0", b"0 0"])
        self.assertTrue(sock.closed)

    def test_interrupt_is_not_success(self):
        sock = FakeSocket()
        status, output = self.run_main(sock, drive_effect=KeyboardInterrupt())
        self.assertEqual(status, 130)
        self.assertNotIn(">>>", output)
        self.assertIn("KHONG HOAN TAT", output)
        self.assertTrue(sock.closed)

    def test_connection_failure_is_not_success(self):
        status, output = self.run_main(None, connect_effect=ConnectionRefusedError("no bridge"))
        self.assertEqual(status, 1)
        self.assertNotIn(">>>", output)

    def test_stop_failure_invalidates_completion(self):
        sock = FakeSocket([[FRAME], [b""]])
        status, output = self.run_main(sock)
        self.assertEqual(status, 1)
        self.assertNotIn(">>>", output)
        self.assertTrue(sock.closed)

    def test_second_interrupt_during_stop_still_closes(self):
        sock = FakeSocket([[FRAME], [KeyboardInterrupt()]])
        status, output = self.run_main(sock, drive_effect=KeyboardInterrupt())
        self.assertEqual(status, 130)
        self.assertNotIn(">>>", output)
        self.assertTrue(sock.closed)

    def test_preflight_timeout_never_sends_motion_or_unsynchronized_stop(self):
        sock = FakeSocket([[FRAME[:5], socket.timeout("incomplete image")]])
        status, output = self.run_main(sock)
        self.assertEqual(status, 1)
        self.assertEqual(sock.sent, [b"0 0"])
        self.assertNotIn(">>>", output)
        self.assertTrue(sock.closed)

    def test_blocked_preflight_never_sends_motion(self):
        sock = FakeSocket([[b'{"Img":"image","blocked":true}'], [FRAME]])
        status, output = self.run_main(sock)
        self.assertEqual(status, 1)
        self.assertEqual(sock.sent, [b"0 0", b"0 0"])
        self.assertNotIn(">>>", output)


if __name__ == "__main__":
    unittest.main()
