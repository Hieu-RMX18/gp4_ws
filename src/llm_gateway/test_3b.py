import json
from llm_gateway.parser import parse_llm_output
from llm_gateway.schema_validator import SchemaValidator
from llm_gateway.normalizer import Normalizer

print("=== Test Parser ===")
sample_llm_json = '''
{
  "choices": [
    {
      "message": {
        "function_call": {
          "name": "execute_robot_move",
          "arguments": "{\\"primitive_type\\":\\"LIN\\", \\"velocity_scale\\":0.3, \\"target_pose\\":{\\"position\\":{\\"x\\": 350.0, \\"y\\": 100.0, \\"z\\": 200.0}}}"
        }
      }
    }
  ]
}
'''
print(f"Raw LLM Input:\\n{sample_llm_json.strip()}")
parsed = parse_llm_output(sample_llm_json)
print(f"-> Parsed Dict: {json.dumps(parsed)}")

print("\n=== Test Validator ===")
validator = SchemaValidator()
valid, err = validator.validate_against_schema(parsed)
print(f"-> Schema Valid: {valid}")
if not valid:
    print(f"-> Error: {err}")

print("\n=== Test Normalizer ===")
raw_pose = parsed.get("target_pose", {})
print(f"Raw Pose (has large mm values): {raw_pose}")
pose_msg = Normalizer.normalize_pose(raw_pose)
print(f"-> Normalized Pose (meters): x={pose_msg.position.x}, y={pose_msg.position.y}, z={pose_msg.position.z}")
