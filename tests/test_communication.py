from __future__ import annotations

import json
import unittest
import urllib.request
from pathlib import Path

from communication.command_server import SimulationCommandServer
from interfaces.pendant.app import _read_asset
from interfaces.pendant.template import PENDANT_HTML


class CommunicationTests(unittest.TestCase):
    def test_state_and_command_round_trip(self) -> None:
        received: list[dict] = []
        try:
            server = SimulationCommandServer(
                "127.0.0.1",
                0,
                lambda: {"reachable": True},
                lambda command: received.append(command) or {"message": "done"},
            )
        except PermissionError:
            self.skipTest("当前沙箱禁止创建本机监听端口")
        server.start()
        host, port = server.address
        try:
            with urllib.request.urlopen(
                f"http://{host}:{port}/api/state", timeout=2.0
            ) as response:
                state = json.load(response)
            self.assertTrue(state["ok"])
            self.assertTrue(state["state"]["reachable"])

            body = json.dumps({"action": "solve"}).encode("utf-8")
            request = urllib.request.Request(
                f"http://{host}:{port}/api/command",
                data=body,
                method="POST",
                headers={"Content-Type": "application/json"},
            )
            with urllib.request.urlopen(request, timeout=2.0) as response:
                result = json.load(response)
            self.assertEqual(result["message"], "done")
            self.assertEqual(received, [{"action": "solve"}])
        finally:
            server.close()

    def test_pendant_uses_fixed_screen_without_range_sliders(self) -> None:
        assets = Path(__file__).resolve().parents[1] / "interfaces/pendant/assets"
        source = (
            PENDANT_HTML
            + (assets / "pendant.css").read_text(encoding="utf-8")
            + (assets / "pendant.js").read_text(encoding="utf-8")
        )
        self.assertIn("overflow: hidden", source)
        self.assertNotIn('type="range"', source)
        self.assertIn("关节运动", source)
        self.assertIn("笛卡尔运动", source)
        self.assertIn("同 TCP 臂型", source)
        self.assertIn("零空间关节点动", source)
        self.assertIn("零空间正方向", source)
        self.assertIn("速度百分比", source)
        self.assertIn("配置文件", source)
        self.assertIn("仿真", PENDANT_HTML)
        self.assertIn("__VISER_URL__", PENDANT_HTML)
        self.assertIn("viserDock", PENDANT_HTML)
        self.assertIn('src="about:blank" data-src="__VISER_URL__"', PENDANT_HTML)
        self.assertNotIn('allow="fullscreen"', PENDANT_HTML)
        self.assertIn("loadSimulationFrame", source)
        self.assertIn("move-active", source)
        self.assertIn("loadSimulationFrame", source)
        self.assertIn("let polling = false", source)
        self.assertIn("move-workspace", PENDANT_HTML)
        self.assertNotIn("move-live-data", PENDANT_HTML)
        self.assertNotIn("moveLivePose", source)
        self.assertNotIn("moveLiveJoints", source)
        self.assertIn("min-width: 1280px", source)
        self.assertNotIn("submoduleRail", source)
        self.assertNotIn("submoduleDefinitions", source)
        self.assertIn("示教程序", PENDANT_HTML)
        self.assertIn("start_continuous_jog", source)
        self.assertIn('cmd("jog_step", extra)', source)
        self.assertIn("cartStepMode", PENDANT_HTML)
        self.assertIn("jointStepMode", PENDANT_HTML)
        self.assertIn("nullStepMode", PENDANT_HTML)
        self.assertIn('window.addEventListener("pointerup", endHold)', source)
        self.assertNotIn("中间点", source)
        self.assertNotIn("起点读取", source)
        self.assertIn("关节角度 (deg)", source)
        self.assertIn("笛卡尔位姿 (m / deg)", source)
        self.assertIn("move_teach_point", source)
        self.assertIn("E1Pro初始化控制", PENDANT_HTML)
        self.assertIn("connect_hardware", source)
        self.assertIn("BRAK1", (Path(__file__).resolve().parents[1] / "interfaces/hardware/marvin.py").read_text(encoding="utf-8"))
        self.assertNotIn("BRAK0", (Path(__file__).resolve().parents[1] / "interfaces/hardware/marvin.py").read_text(encoding="utf-8"))

    def test_page_switch_never_navigates_the_viser_frame(self) -> None:
        assets = Path(__file__).resolve().parents[1] / "interfaces/pendant/assets"
        source = (assets / "pendant.js").read_text(encoding="utf-8")
        show_page = source[source.index("function showPage"):source.index("function loadSimulationFrame")]
        self.assertNotIn("viserFrame", show_page)
        self.assertNotIn(".src", show_page)

    def test_pendant_embeds_configured_viser_url(self) -> None:
        body, content_type = _read_asset(
            "/",
            "http://127.0.0.1:8123/viser",
        )
        self.assertEqual(content_type, "text/html; charset=utf-8")
        html = body.decode("utf-8")
        self.assertIn('src="http://127.0.0.1:8123/viser"', html)
        self.assertNotIn("__VISER_URL__", html)


if __name__ == "__main__":
    unittest.main()
