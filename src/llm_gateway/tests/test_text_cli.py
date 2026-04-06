from llm_gateway.text_cli import build_argument_parser


def test_text_cli_defaults_to_llm_text_input():
    args = build_argument_parser().parse_args([])
    assert args.topic == "/llm_text_input"
    assert args.wait_timeout == 2.0
    assert args.text == []


def test_text_cli_accepts_one_shot_text_arguments():
    args = build_argument_parser().parse_args(["go", "home"])
    assert args.text == ["go", "home"]
