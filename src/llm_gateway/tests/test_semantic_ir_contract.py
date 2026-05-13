"""Tests for the strict Semantic IR contract gate."""

from __future__ import annotations


from llm_gateway.semantic_ir_contract import validate_semantic_ir_contract


class TestAcceptsValidSemanticIR:
    def test_go_home(self):
        result = validate_semantic_ir_contract({"intent": "go_home"})
        assert result.valid is True
        assert result.reason == ""

    def test_sequence(self):
        result = validate_semantic_ir_contract(
            {"intent": "sequence", "steps": [{"intent": "go_home"}]}
        )
        assert result.valid is True

    def test_return_to_start(self):
        result = validate_semantic_ir_contract(
            {"intent": "sequence", "steps": [{"intent": "return_to_start"}]}
        )
        assert result.valid is True


class TestRejectsPrimitiveTypeLeakage:
    def test_rejects_primitive_type_field(self):
        result = validate_semantic_ir_contract(
            {"intent": "go_home", "primitive_type": "HOME"}
        )
        assert result.valid is False
        assert "primitive_type" in result.reason
        assert "hint" in result.hint.lower() or "backward" in result.hint.lower()

    def test_rejects_primitive_only_payload(self):
        result = validate_semantic_ir_contract({"primitive_type": "HOME"})
        assert result.valid is False
        assert "primitive_type" in result.reason

    def test_rejects_primitive_type_inside_sequence_step(self):
        result = validate_semantic_ir_contract(
            {"intent": "sequence", "steps": [{"primitive_type": "HOME"}]}
        )
        assert result.valid is False
        assert "$.steps[0].primitive_type" in result.reason
        assert "primitive_type" in result.reason


class TestRejectsRawTextLeakage:
    def test_rejects_raw_text_field(self):
        result = validate_semantic_ir_contract(
            {"intent": "go_home", "raw_text": "go home"}
        )
        assert result.valid is False
        assert "raw_text" in result.reason

    def test_rejects_raw_text_inside_sequence_step(self):
        result = validate_semantic_ir_contract(
            {
                "intent": "sequence",
                "steps": [{"intent": "go_home", "raw_text": "go home"}],
            }
        )
        assert result.valid is False
        assert "$.steps[0].raw_text" in result.reason
        assert "raw_text" in result.reason


class TestRejectsUnknownIntent:
    def test_rejects_fly_to_moon(self):
        result = validate_semantic_ir_contract({"intent": "fly_to_moon"})
        assert result.valid is False
        assert "Unsupported semantic intent" in result.reason
        assert "fly_to_moon" in result.reason
        assert "go_home" in result.hint

    def test_rejects_empty_intent(self):
        result = validate_semantic_ir_contract({"intent": ""})
        assert result.valid is False
        assert "non-empty 'intent' field" in result.reason

    def test_rejects_missing_intent(self):
        result = validate_semantic_ir_contract({"delta": {"x": 0.1}})
        assert result.valid is False
        assert "non-empty 'intent' field" in result.reason

    def test_rejects_top_level_return_to_start(self):
        result = validate_semantic_ir_contract({"intent": "return_to_start"})
        assert result.valid is False
        assert "only valid inside a sequence" in result.reason


class TestAcceptsErrorPayloads:
    def test_missing_slot_error(self):
        result = validate_semantic_ir_contract(
            {
                "error": "MISSING_SLOT",
                "intent": "move_relative",
                "missing_fields": ["delta"],
            }
        )
        assert result.valid is True

    def test_unsupported_or_ambiguous_error(self):
        result = validate_semantic_ir_contract(
            {"error": "UNSUPPORTED_OR_AMBIGUOUS_COMMAND"}
        )
        assert result.valid is True

    def test_react_handoff_error(self):
        result = validate_semantic_ir_contract(
            {
                "error": "REACT_HANDOFF",
                "message": "budget exceeded",
                "hint": "try again",
            }
        )
        assert result.valid is True


class TestRejectsNonDictPayload:
    def test_rejects_string(self):
        result = validate_semantic_ir_contract("go home")
        assert result.valid is False
        assert "JSON object" in result.reason

    def test_rejects_list(self):
        result = validate_semantic_ir_contract([{"intent": "go_home"}])
        assert result.valid is False

    def test_rejects_none(self):
        result = validate_semantic_ir_contract(None)
        assert result.valid is False
