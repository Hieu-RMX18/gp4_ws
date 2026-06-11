"""Tests cho tầng 1: direct_commands — parse deterministic, không LLM.

Hợp đồng: chỉ 5 lệnh an toàn (stop / home / get pose / alarm reset / wait N s),
khớp NGUYÊN CÂU sau khi fold dấu. Mọi text khác trả None → đi đường LLM.
"""

from __future__ import annotations

import pytest

from llm_gateway import direct_commands


class TestStop:
    @pytest.mark.parametrize(
        "text",
        ["stop", "STOP", " Stop. ", "stop motion", "cancel motion", "halt",
         "dừng", "dừng lại", "dừng ngay", "dung lai"],
    )
    def test_stop_variants(self, text):
        assert direct_commands.parse(text) == {"intent": "stop"}


class TestHome:
    @pytest.mark.parametrize(
        "text",
        ["home", "go home", "move home", "return home",
         "về nhà", "về home", "ve nha", "ve home"],
    )
    def test_home_variants(self, text):
        assert direct_commands.parse(text) == {"intent": "go_home"}


class TestGetPose:
    @pytest.mark.parametrize(
        "text",
        ["get pose", "get_pose", "get current pose", "current pose",
         "where is the robot", "vị trí hiện tại", "tọa độ hiện tại",
         "toa do hien tai", "robot đang ở đâu"],
    )
    def test_get_pose_variants(self, text):
        assert direct_commands.parse(text) == {
            "intent": "get_pose",
            "reference_frame": "base_link",
        }


class TestAlarmReset:
    @pytest.mark.parametrize(
        "text",
        ["alarm reset", "alarm_reset", "reset alarm", "clear alarm",
         "xóa lỗi", "reset lỗi", "xoa loi"],
    )
    def test_alarm_reset_variants(self, text):
        assert direct_commands.parse(text) == {"intent": "alarm_reset"}


class TestWait:
    @pytest.mark.parametrize(
        ("text", "expected_sec"),
        [
            ("wait 2 s", 2.0),
            ("wait 2s", 2.0),
            ("wait 0.5 seconds", 0.5),
            ("chờ 3 giây", 3.0),
            ("cho 3 giay", 3.0),
            ("đợi 10s", 10.0),
        ],
    )
    def test_wait_with_duration(self, text, expected_sec):
        assert direct_commands.parse(text) == {
            "intent": "wait",
            "wait_duration_sec": expected_sec,
        }

    @pytest.mark.parametrize("text", ["wait", "chờ", "wait 0 s", "wait 9999 s", "wait -2 s"])
    def test_wait_without_valid_duration_defers_to_llm(self, text):
        assert direct_commands.parse(text) is None


class TestEverythingElseGoesToLLM:
    @pytest.mark.parametrize(
        "text",
        [
            "", "   ", "move to pose A", "go to A", "move to red_box",
            "move to Cartesian x 300 mm y 0 z 400", "move down 2 cm",
            "stop and go home",
            "go home then wait one second",
            "đi tới A hạ xuống 5cm chờ 2s rồi về home",
            "xoay khớp số 3 +15 độ", "draw circle radius 5cm",
            "gắp từng vật trên băng tải qua gá phôi",
            "homer",
        ],
    )
    def test_returns_none(self, text):
        assert direct_commands.parse(text) is None
