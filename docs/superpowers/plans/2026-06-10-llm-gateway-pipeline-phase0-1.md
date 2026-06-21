# LLM Gateway Pipeline — Phase 0+1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Xóa review cache, đính `code_version`/`parse_source` vào review response, và thay shortcut dict thô bằng module `direct_commands.py` (tầng 1 deterministic: stop / home / get pose / alarm reset / wait) có alias VN/EN và test đầy đủ.

**Architecture:** Đây là Plan 1 của spec `docs/superpowers/specs/2026-06-10-llm-gateway-factory-pipeline-design.md` (Phase 0 + Phase 1). Một câu lệnh chỉ có 2 đường: `direct_commands.parse()` bắt được → Semantic IR đơn đi thẳng; không bắt được → LLM→FactoryTask. Không cache kết quả LLM. Phase 2-6 có plan riêng sau checkpoint này.

**Tech Stack:** Python 3.10, ROS 2 Humble, pytest. Package `llm_gateway` cài egg-link (sửa code phải restart node mới có hiệu lực).

**Bối cảnh working tree:** branch `upgrade-react-8626` đang có sẵn thay đổi CHƯA commit đi đúng hướng spec (fast-path ~700 dòng đã bị thay bằng `_DIRECT_REVIEW_SHORTCUTS` 3 entry; tests legacy đã xóa). Task 0 chốt baseline này trước khi sửa tiếp. KHÔNG revert các thay đổi đang có.

**Môi trường chạy test:**

```bash
cd /home/hieu2/gp4_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
cd src/llm_gateway
```

---

### Task 0: Chốt baseline working tree

**Files:**
- Không sửa file nào — chỉ verify + commit trạng thái hiện có.

- [ ] **Step 1: Chạy toàn bộ test llm_gateway trên tree hiện tại**

Run: `python -m pytest tests/ -x -q 2>&1 | tail -20`
Expected: PASS toàn bộ (hoặc skip do thiếu interfaces build). Nếu FAIL: dừng lại, báo user — không commit baseline đỏ, không tự sửa test người khác đang viết dở.

- [ ] **Step 2: Commit baseline**

```bash
cd /home/hieu2/gp4_ws
git add -A src/llm_gateway hmi
git commit -m "refactor(llm_gateway): narrow direct review path to exact deterministic shortcuts (WIP baseline)"
```

---

### Task 1: Xóa review cache (Phase 0a)

Cache là nguyên nhân "bug ghim kết quả sai suốt phiên" (spec §2.2). Mỗi submit = parse mới.

**Files:**
- Modify: `src/llm_gateway/llm_gateway/llm_gateway_node.py` — xóa các symbol: `_REVIEW_CACHE_VERSION`, `_REVIEW_CACHE_MAX_ENTRIES`, `self._semantic_review_cache` (init), `_review_cache_key`, `_get_semantic_review_cache`, `_store_semantic_review_cache`, lời gọi cache trong `_generate_review_semantic_ir` và `_on_review_intent`
- Test: `src/llm_gateway/tests/test_react_gateway_pipeline.py`

- [ ] **Step 1: Viết failing test (structural guard + behavioral)**

Thêm vào cuối `tests/test_react_gateway_pipeline.py`:

```python
def test_review_cache_is_fully_removed():
    """Spec 2026-06-10 §5: không cache kết quả LLM giữa các lần submit."""
    LLMGatewayNode = _gateway_node_type()
    for legacy_symbol in (
        "_get_semantic_review_cache",
        "_store_semantic_review_cache",
        "_review_cache_key",
    ):
        assert not hasattr(LLMGatewayNode, legacy_symbol), legacy_symbol


def test_generate_review_semantic_ir_reparses_every_submission():
    node = _make_gateway_shell({"intent": "go_home"})
    inner_agent = node._react_agent
    calls = []

    class _CountingAgent:
        def run(self, text):
            calls.append(text)
            return inner_agent.run(text)

    node._react_agent = _CountingAgent()
    first = node._generate_review_semantic_ir("về nhà")
    second = node._generate_review_semantic_ir("về nhà")
    assert len(calls) == 2
    assert first.get("error") is None
    assert second.get("error") is None
```

- [ ] **Step 2: Chạy test xác nhận fail**

Run: `python -m pytest tests/test_react_gateway_pipeline.py::test_review_cache_is_fully_removed -v`
Expected: FAIL — `AssertionError: _get_semantic_review_cache` (method vẫn tồn tại).

(`test_generate_review_semantic_ir_reparses_every_submission` có thể pass sẵn vì cache chỉ được ghi trong `_on_review_intent` — vẫn giữ làm guard hồi quy.)

- [ ] **Step 3: Xóa cache khỏi node**

Trong `llm_gateway/llm_gateway_node.py`:

1. Xóa 2 hằng module-level:
```python
_REVIEW_CACHE_VERSION = "react_semantic_review_v1"
_REVIEW_CACHE_MAX_ENTRIES = 128
```
2. Trong `__init__`, xóa dòng:
```python
self._semantic_review_cache: Dict[str, Dict[str, Any]] = {}
```
3. Trong `_generate_review_semantic_ir`, xóa block đầu hàm:
```python
cached_review = self._get_semantic_review_cache(intent_text)
if cached_review is not None:
    return cached_review
```
4. Trong `_on_review_intent`, xóa dòng (ngay trước `return response` cuối nhánh accepted):
```python
self._store_semantic_review_cache(intent_text, semantic_ir)
```
5. Xóa nguyên 3 method: `_review_cache_key`, `_get_semantic_review_cache`, `_store_semantic_review_cache`.

- [ ] **Step 4: Xác nhận không còn tham chiếu sót**

Run: `grep -rn "review_cache\|_semantic_review_cache\|_REVIEW_CACHE" llm_gateway/ tests/ | grep -v test_review_cache_is_fully_removed`
Expected: không có output.

- [ ] **Step 5: Chạy test pass**

Run: `python -m pytest tests/test_react_gateway_pipeline.py -q 2>&1 | tail -5`
Expected: PASS toàn bộ file.

- [ ] **Step 6: Commit**

```bash
cd /home/hieu2/gp4_ws
git add src/llm_gateway/llm_gateway/llm_gateway_node.py src/llm_gateway/tests/test_react_gateway_pipeline.py
git commit -m "fix(llm_gateway): remove semantic review cache so every submission reparses"
```

---

### Task 2: Đính `code_version` + `parse_source` vào review response (Phase 0b)

Chống bug "stale process âm thầm" (spec §5): nhìn response biết ngay node chạy code nào, đường parse nào trả lời.

**Files:**
- Modify: `src/llm_gateway/llm_gateway/llm_gateway_node.py`
- Test: `src/llm_gateway/tests/test_react_gateway_pipeline.py`

- [ ] **Step 1: Viết failing test**

Thêm vào cuối `tests/test_react_gateway_pipeline.py`:

```python
def test_review_response_carries_code_version_and_parse_source():
    """Spec 2026-06-10 §5: review response đính parse_source + code_version."""
    node = _make_gateway_shell({"intent": "go_home"})
    node._code_version = "abc1234"
    payload = node._generate_review_semantic_ir("về nhà")
    prepared = node._prepare_review_semantic_ir(payload)
    stamped = node._stamp_review_provenance(prepared)
    assert stamped["_parse_source"] in {"direct", "react_factory_task", "llm_factory_task", "react", "llm"}
    assert stamped["_code_version"] == "abc1234"


def test_resolve_code_version_returns_short_hash_or_unknown():
    LLMGatewayNode = _gateway_node_type()
    version = LLMGatewayNode._resolve_code_version()
    assert isinstance(version, str)
    assert version == "unknown" or (4 <= len(version) <= 16)
```

- [ ] **Step 2: Chạy test xác nhận fail**

Run: `python -m pytest tests/test_react_gateway_pipeline.py::test_review_response_carries_code_version_and_parse_source -v`
Expected: FAIL — `AttributeError: ... has no attribute '_stamp_review_provenance'`.

- [ ] **Step 3: Implement**

Trong `llm_gateway/llm_gateway_node.py`:

1. Đầu file đã có `import os, json,...` — thêm nếu thiếu:
```python
import subprocess
```
2. Thêm 2 method vào class `LLMGatewayNode` (cạnh `_resolve_runtime_mode`):
```python
@staticmethod
def _resolve_code_version() -> str:
    """Git short hash của code đang chạy, 'unknown' nếu không xác định được."""
    try:
        result = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=str(Path(__file__).resolve().parent),
            capture_output=True,
            text=True,
            timeout=2.0,
            check=True,
        )
        return result.stdout.strip() or "unknown"
    except Exception:
        return "unknown"

def _stamp_review_provenance(self, payload: Dict[str, Any]) -> Dict[str, Any]:
    """Đính parse_source + code_version vào mọi review payload (immutable)."""
    if not isinstance(payload, dict):
        return payload
    stamped = dict(payload)
    stamped.setdefault("_parse_source", "unknown")
    stamped["_code_version"] = getattr(self, "_code_version", "unknown")
    return stamped
```
(`Path` đã được import sẵn trong file — nếu chưa: `from pathlib import Path`.)

3. Trong `__init__`, sau `self._runtime_mode = runtime_mode` thêm:
```python
self._code_version = self._resolve_code_version()
```
4. Trong `_on_review_intent`, ngay SAU dòng `semantic_ir = self._prepare_review_semantic_ir(semantic_ir)` và TRƯỚC `contract = validate_semantic_ir_contract(semantic_ir)` thêm:
```python
semantic_ir = self._stamp_review_provenance(semantic_ir)
```

- [ ] **Step 4: Chạy test pass + kiểm tra contract không reject key mới**

Run: `python -m pytest tests/test_react_gateway_pipeline.py tests/test_contracts.py -q 2>&1 | tail -5`
Expected: PASS. Nếu `validate_semantic_ir_contract` reject `_code_version`: sửa `semantic_ir_contract.py` để bỏ qua mọi key bắt đầu bằng `_` (giống `_parse_source` hiện được chấp nhận).

- [ ] **Step 5: Commit**

```bash
cd /home/hieu2/gp4_ws
git add src/llm_gateway/llm_gateway/llm_gateway_node.py src/llm_gateway/tests/test_react_gateway_pipeline.py
git commit -m "feat(llm_gateway): stamp review responses with parse_source and code_version"
```

---

### Task 3: Module `direct_commands.py` — tầng 1 deterministic (Phase 1a)

**Files:**
- Create: `src/llm_gateway/llm_gateway/direct_commands.py`
- Create: `src/llm_gateway/tests/test_direct_commands.py`

- [ ] **Step 1: Viết failing tests (toàn bộ hành vi tầng 1)**

Tạo `tests/test_direct_commands.py`:

```python
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
            "stop and go home",                # câu ghép → LLM
            "go home then wait one second",    # câu ghép → LLM
            "đi tới A hạ xuống 5cm chờ 2s rồi về home",
            "xoay khớp số 3 +15 độ", "draw circle radius 5cm",
            "gắp từng vật trên băng tải qua gá phôi",
            "homer",                           # không khớp prefix lỏng lẻo
        ],
    )
    def test_returns_none(self, text):
        assert direct_commands.parse(text) is None
```

- [ ] **Step 2: Chạy test xác nhận fail**

Run: `python -m pytest tests/test_direct_commands.py -q 2>&1 | tail -3`
Expected: FAIL — `ModuleNotFoundError`/`ImportError: cannot import name 'direct_commands'`.

- [ ] **Step 3: Implement module**

Tạo `llm_gateway/direct_commands.py`:

```python
"""Tầng 1 của pipeline parse: lệnh deterministic an toàn, không cần LLM.

Spec: docs/superpowers/specs/2026-06-10-llm-gateway-factory-pipeline-design.md §3.
Hợp đồng:
  - Chỉ 5 lệnh: stop / go home / get pose / alarm reset / wait N giây.
  - Khớp NGUYÊN CÂU (sau khi fold dấu tiếng Việt, lowercase, gộp khoảng trắng).
  - Trả Semantic IR đơn (dict) khi khớp, None khi không — None nghĩa là
    "chuyển cho task_planner (LLM)", không bao giờ đoán mò.
"""

from __future__ import annotations

import re
import unicodedata
from typing import Any, Dict

_WAIT_MAX_SEC = 300.0

_STOP_TEXTS = frozenset({
    "stop", "stop motion", "cancel motion", "halt",
    "dung", "dung lai", "dung ngay",
})
_HOME_TEXTS = frozenset({
    "home", "go home", "move home", "return home",
    "ve nha", "ve home",
})
_GET_POSE_TEXTS = frozenset({
    "get pose", "get_pose", "get current pose", "current pose",
    "where is the robot", "where is robot", "where is tcp",
    "vi tri hien tai", "toa do hien tai", "robot dang o dau",
})
_ALARM_RESET_TEXTS = frozenset({
    "alarm reset", "alarm_reset", "reset alarm", "clear alarm",
    "xoa loi", "reset loi",
})
_WAIT_RE = re.compile(
    r"^(?:wait|cho|doi|cho doi)\s+(\d+(?:\.\d+)?)\s*"
    r"(?:s|sec|secs|second|seconds|giay)?$"
)


def _fold(text: str) -> str:
    """Bỏ dấu tiếng Việt, lowercase, gộp khoảng trắng, cắt dấu câu cuối."""
    folded = unicodedata.normalize("NFD", str(text or ""))
    folded = "".join(ch for ch in folded if unicodedata.category(ch) != "Mn")
    folded = folded.replace("đ", "d").replace("Đ", "D").lower()
    return " ".join(folded.split()).strip(" .!?")


def parse(intent_text: str) -> Dict[str, Any] | None:
    """Parse tầng 1. Trả Semantic IR đơn hoặc None (→ LLM)."""
    folded = _fold(intent_text)
    if not folded:
        return None
    if folded in _STOP_TEXTS:
        return {"intent": "stop"}
    if folded in _HOME_TEXTS:
        return {"intent": "go_home"}
    if folded in _GET_POSE_TEXTS:
        return {"intent": "get_pose", "reference_frame": "base_link"}
    if folded in _ALARM_RESET_TEXTS:
        return {"intent": "alarm_reset"}
    wait_match = _WAIT_RE.match(folded)
    if wait_match is not None:
        duration = float(wait_match.group(1))
        if 0.0 < duration <= _WAIT_MAX_SEC:
            return {"intent": "wait", "wait_duration_sec": duration}
    return None
```

- [ ] **Step 4: Chạy test pass**

Run: `python -m pytest tests/test_direct_commands.py -v 2>&1 | tail -10`
Expected: PASS toàn bộ (~45 case).

- [ ] **Step 5: Commit**

```bash
cd /home/hieu2/gp4_ws
git add src/llm_gateway/llm_gateway/direct_commands.py src/llm_gateway/tests/test_direct_commands.py
git commit -m "feat(llm_gateway): add direct_commands tier-1 deterministic parser with VN/EN aliases"
```

---

### Task 4: Wire `direct_commands` vào node, xóa shortcut dict cũ (Phase 1b)

**Files:**
- Modify: `src/llm_gateway/llm_gateway/llm_gateway_node.py` — thay `_DIRECT_REVIEW_SHORTCUTS` + `_direct_review_semantic_ir` bằng `direct_commands.parse`
- Modify: `src/llm_gateway/tests/test_direct_review_regex.py` — cập nhật fixture sang module mới
- Test: `src/llm_gateway/tests/test_react_gateway_pipeline.py`

- [ ] **Step 1: Viết failing test (node dùng module mới)**

Thêm vào cuối `tests/test_react_gateway_pipeline.py`:

```python
def test_direct_tier_handles_safe_commands_without_react():
    """Tầng 1 trả lời stop/home/get_pose/alarm_reset/wait — ReAct không được gọi."""
    node = _make_gateway_shell({"intent": "go_home"})

    class _MustNotRun:
        def run(self, text):
            raise AssertionError("ReAct must not run for tier-1 commands")

    node._react_agent = _MustNotRun()
    assert node._generate_review_semantic_ir("dừng lại")["intent"] == "stop"
    assert node._generate_review_semantic_ir("về nhà")["intent"] == "go_home"
    assert node._generate_review_semantic_ir("wait 2 s") == {
        "intent": "wait",
        "wait_duration_sec": 2.0,
        "_parse_source": "direct",
    }


def test_legacy_direct_review_shortcut_symbols_are_removed():
    import llm_gateway.llm_gateway_node as node_module

    LLMGatewayNode = _gateway_node_type()
    assert not hasattr(node_module, "_DIRECT_REVIEW_SHORTCUTS")
    assert not hasattr(LLMGatewayNode, "_direct_review_semantic_ir")
```

- [ ] **Step 2: Chạy test xác nhận fail**

Run: `python -m pytest tests/test_react_gateway_pipeline.py::test_direct_tier_handles_safe_commands_without_react tests/test_react_gateway_pipeline.py::test_legacy_direct_review_shortcut_symbols_are_removed -v`
Expected: FAIL — "dừng lại" không có trong shortcut dict cũ → ReAct bị gọi → AssertionError; symbols cũ vẫn tồn tại.

- [ ] **Step 3: Implement wiring**

Trong `llm_gateway/llm_gateway_node.py`:

1. Thêm import (cạnh các import llm_gateway khác):
```python
from llm_gateway import direct_commands
```
2. Xóa hằng module-level `_DIRECT_REVIEW_SHORTCUTS = {...}` (3 entry).
3. Xóa nguyên method `_direct_review_semantic_ir` của class.
4. Trong `_generate_review_semantic_ir`, thay block:
```python
direct_review = self._direct_review_semantic_ir(
    intent_text,
    runtime_mode=self._runtime_mode,
    station_scene_graph=getattr(self, "_station_scene_graph", None),
)
```
bằng:
```python
direct_review = direct_commands.parse(intent_text)
```
5. Ngay dưới đó, trong nhánh `if direct_review is not None:` đổi giá trị `_parse_source` thành `"direct"`:
```python
validated = dict(direct_review)
validated["_parse_source"] = "direct"
return validated
```
(giữ nguyên `self._emit_trace("direct_pre_parsed", ...)` nếu đang có, đổi `source="direct"`).

- [ ] **Step 4: Cập nhật test file cũ**

Trong `tests/test_direct_review_regex.py`:

1. Đổi docstring đầu file thành: `"""Hợp đồng tầng direct (đã chuyển sang llm_gateway.direct_commands) + draw params."""`
2. Thay fixture:
```python
@pytest.fixture
def direct_review():
    from llm_gateway import direct_commands

    return direct_commands.parse
```
3. Trong class `TestDirectReviewDeterministicSafety`, cập nhật parametrize khớp hành vi mới:
   - Case `("get_pose", {"intent": "get_pose"})` → `("get_pose", {"intent": "get_pose", "reference_frame": "base_link"})`.
   - Trong danh sách `test_free_form_language_goes_to_react_or_llm`, XÓA các dòng giờ đã được tầng 1 bắt: `"stop motion"`, `"cancel motion"`, `"halt"`, `"alarm reset"`, `"get pose"`, `"go home"`, `"wait 2 s"` (các case này đã được test trong `test_direct_commands.py`).
4. Giữ nguyên class `TestDrawParamsValidation` (test `LLMGatewayNode._validate_draw_params`, không liên quan).

- [ ] **Step 5: Chạy test pass**

Run: `python -m pytest tests/test_direct_commands.py tests/test_direct_review_regex.py tests/test_react_gateway_pipeline.py -q 2>&1 | tail -5`
Expected: PASS toàn bộ.

- [ ] **Step 6: Xác nhận không còn tham chiếu sót**

Run: `grep -rn "_DIRECT_REVIEW_SHORTCUTS\|_direct_review_semantic_ir" llm_gateway/ tests/ ../../hmi/backend --include="*.py" | grep -v "symbols_are_removed"`
Expected: không có output (nếu HMI backend còn gọi — sửa chỗ đó dùng service review như bình thường, KHÔNG import trực tiếp).

- [ ] **Step 7: Commit**

```bash
cd /home/hieu2/gp4_ws
git add src/llm_gateway/llm_gateway/llm_gateway_node.py src/llm_gateway/tests/
git commit -m "refactor(llm_gateway): route tier-1 parsing through direct_commands module"
```

---

### Task 5: Verify toàn cục + checkpoint

**Files:** không sửa — chỉ chạy kiểm tra.

- [ ] **Step 1: Full test suite llm_gateway**

Run: `python -m pytest tests/ -q 2>&1 | tail -5`
Expected: PASS toàn bộ.

- [ ] **Step 2: HMI backend tests (đảm bảo không vỡ contract)**

Run: `cd /home/hieu2/gp4_ws/hmi/backend && .venv/bin/python -m pytest tests/ -q 2>&1 | tail -5`
Expected: PASS (HMI gọi gateway qua service ROS, không import nội bộ — nếu fail vì import llm_gateway nội bộ thì đó là vi phạm ranh giới, báo lại trong notes).

- [ ] **Step 3: Build + smoke test sim**

```bash
cd /home/hieu2/gp4_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select llm_gateway --symlink-install
source install/setup.bash
```
Expected: build green.

- [ ] **Step 4: Nhắc restart node (quan trọng — egg-link)**

Ghi vào commit message / báo user: **mọi node llm_gateway + HMI backend đang chạy phải restart** thì code mới có hiệu lực. Đây chính là root cause của bug GET_POSE trước đó.

- [ ] **Step 5: Commit cuối (nếu có thay đổi phát sinh)**

```bash
cd /home/hieu2/gp4_ws
git status --short  # xác nhận sạch hoặc commit phần còn lại
```

---

## Self-review checklist (đã chạy)

1. **Spec coverage:** Phase 0 (xóa cache ✓ Task 1; code_version + parse_source ✓ Task 2), Phase 1 (direct_commands.py + tests ✓ Task 3; wire vào node + xóa đường cũ ✓ Task 4). Phase 2-6: plan riêng sau checkpoint.
2. **Placeholders:** không có TBD/TODO; mọi step code đều có code đầy đủ.
3. **Type consistency:** `direct_commands.parse(text) -> Dict[str, Any] | None`; `_stamp_review_provenance(dict) -> dict`; `_resolve_code_version() -> str` — dùng nhất quán giữa Task 2/3/4.

## Ghi chú cho executor

- Working tree có thay đổi chưa commit của người khác — Task 0 chốt baseline TRƯỚC, không rebase/reset.
- Nếu file `llm_gateway_node.py` đã bị sửa khác với mô tả (đang có người sửa song song): tìm theo TÊN SYMBOL, không theo số dòng; nếu symbol không còn tồn tại, dừng và báo lại.
- Test cần `source install/setup.bash` để import `interfaces` (một số test importorskip).
