import pytest


def test_parse_openai_tool_call(parser, openai_payload):
    parsed = parser.parse(openai_payload)
    assert parsed["name"] == "execute_motion"
    assert parsed["arguments"]["primitive_type"] == "LIN"


def test_parse_anthropic_tool_use(parser, anthropic_payload):
    parsed = parser.parse(anthropic_payload)
    assert parsed["name"] == "execute_motion"
    assert parsed["arguments"]["primitive_type"] == "LIN"


def test_parse_invalid_json_rejected(parser):
    with pytest.raises(ValueError, match="Invalid JSON format"):
        parser.parse("not-json")
