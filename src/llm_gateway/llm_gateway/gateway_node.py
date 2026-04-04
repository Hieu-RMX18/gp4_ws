"""Deprecated compatibility shim; runtime should use llm_gateway.llm_gateway_node."""

from llm_gateway.llm_gateway_node import LLMGatewayNode, main

__all__ = ["LLMGatewayNode", "main"]
