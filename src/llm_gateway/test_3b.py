import json

from llm_gateway.normalizer import Normalizer
from llm_gateway.parser import parse_llm_output
from llm_gateway.schema_validator import SchemaValidator
from llm_gateway.semantic_validator import SemanticValidator

print("=== Test Parser ===")
sample_llm_json = json.dumps(
    {
        "choices": [
            {
                "message": {
                    "role": "assistant",
                    "content": json.dumps(
                        {
                            "primitive_type": "LIN",
                            "velocity_scale": 0.2,
                            "target_pose": {
                                "position": {"x": 0.35, "y": 0.1, "z": 0.2},
                                "orientation": {
                                    "x": 0.0,
                                    "y": 0.707,
                                    "z": 0.0,
                                    "w": 0.707,
                                },
                            },
                        }
                    ),
                }
            }
        ]
    }
)
print(f"Raw LLM Input:\n{sample_llm_json}")
parsed = parse_llm_output(sample_llm_json)
print(f"-> Parsed Dict: {json.dumps(parsed)}")

print("\n=== Test Validator ===")
validator = SchemaValidator()
valid, err = validator.validate_against_schema(parsed)
print(f"-> Schema Valid: {valid}")
if not valid:
    print(f"-> Error: {err}")

print("\n=== Test Normalizer ===")
normalized = Normalizer().normalize(parsed)
pose_msg = normalized.get("target_pose_msg")
print(
    "-> Normalized Pose (meters/quaternion): "
    f"x={pose_msg.position.x}, y={pose_msg.position.y}, z={pose_msg.position.z}, "
    f"qx={pose_msg.orientation.x}, qy={pose_msg.orientation.y}, "
    f"qz={pose_msg.orientation.z}, qw={pose_msg.orientation.w}"
)

print("\n=== Test Semantic Validator ===")
semantic_validator = SemanticValidator()
print(f"-> Semantic Valid: {semantic_validator.validate(normalized)}")
