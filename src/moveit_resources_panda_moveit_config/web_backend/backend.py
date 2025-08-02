from flask import Flask, request, jsonify
from flask_cors import CORS
import subprocess

app = Flask(__name__)
CORS(app)  # Enable CORS for all routes

# Only accept FK joint angle parameters
FK_JOINT_PARAMS = [
    'fk_joint1', 'fk_joint2', 'fk_joint3', 'fk_joint4', 'fk_joint5', 'fk_joint6', 'fk_joint7'
]
ROS_NODE = '/panda_fk_node'

@app.route('/set_param', methods=['POST'])
def set_param():
    data = request.get_json()
    name = data.get('name')
    value = data.get('value')
    if name not in FK_JOINT_PARAMS:
        return jsonify({'status': 'error', 'message': f'Parameter {name} not allowed.'}), 400
    try:
        float_value = float(value)  # Ensure value is a float
        subprocess.run([
            'ros2', 'param', 'set', ROS_NODE, name, str(float_value)
        ], check=True)
        return jsonify({'status': 'success', 'param': name, 'value': float_value, 'node': ROS_NODE})
    except subprocess.CalledProcessError:
        return jsonify({'status': 'error', 'param': name, 'value': value, 'node': ROS_NODE}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
