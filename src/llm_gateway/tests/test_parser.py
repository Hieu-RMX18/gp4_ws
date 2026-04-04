import pytest


def test_parse_openai_content_json(parser, openai_payload):
    parsed = parser.parse(openai_payload)
    assert parsed["primitive_type"] == "LIN"


def test_parse_direct_json_object(parser, direct_command_json):
    parsed = parser.parse(direct_command_json)
    assert parsed["primitive_type"] == "LIN"


def test_parse_legacy_openai_tool_call(parser, legacy_openai_tool_payload):
    parsed = parser.parse(legacy_openai_tool_payload)
    assert parsed["primitive_type"] == "LIN"


def test_parse_anthropic_tool_use(parser, anthropic_payload):
    parsed = parser.parse(anthropic_payload)
    assert parsed["primitive_type"] == "LIN"


def test_parse_model_error_json(parser, model_error_payload):
    parsed = parser.parse(model_error_payload)
    assert parsed == {"error": "UNSUPPORTED_OR_AMBIGUOUS_COMMAND"}


def test_parse_invalid_json_rejected(parser):
    with pytest.raises(ValueError, match="Invalid JSON format"):
        parser.parse("not-json")


def test_parse_non_json_content_rejected(parser):
    payload = '{"choices":[{"message":{"role":"assistant","content":"not-json"}}]}'
    with pytest.raises(ValueError, match="Model content must be a JSON object"):
        parser.parse(payload)
