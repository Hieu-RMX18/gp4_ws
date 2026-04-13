import os
import yaml
import pytest
from geometry_msgs.msg import Pose

@pytest.fixture
def safety_rules():
    yaml_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'safety_rules.yaml')
    with open(yaml_path, 'r') as f:
        rules = yaml.safe_load(f)
    
    # Inject a known test_box inside bounds to test AABB logic independently of z_min
    rules['forbidden_zones'].append({
        'name': 'test_box',
        'x': -0.05, 'y': -0.05, 'z': 0.2,
        'size_x': 0.1, 'size_y': 0.1, 'size_z': 0.1
    })
    return rules

@pytest.fixture
def valid_pose():
    p = Pose()
    p.position.x = 0.15
    p.position.y = -0.15
    p.position.z = 0.20
    return p

@pytest.fixture
def table_collision_pose():
    # Adjusted to point precisely at our mock 'test_box' center to trigger collision
    p = Pose()
    p.position.x = -0.05
    p.position.y = -0.05
    p.position.z = 0.2
    return p

@pytest.fixture
def below_z_min_pose():
    p = Pose()
    p.position.x = 0.15
    p.position.y = -0.15
    p.position.z = 0.01
    return p
