import os
import json
import rclpy
from rclpy.node import Node
from example_interfaces.srv import String

DEFAULT_JSON = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 'artifacts', 'semantic_mock.json')

class SemanticMapper(Node):
    def __init__(self):
        super().__init__('semantic_mapper')
        self.declare_parameter('mock_db', DEFAULT_JSON)
        path = self.get_parameter('mock_db').get_parameter_value().string_value
        self.get_logger().info(f'Loading semantic DB: {path}')
        try:
            with open(path, 'r') as f:
                self.db = json.load(f)
        except Exception as e:
            self.get_logger().warn(f'Failed to load semantic DB: {e}; using defaults')
            self.db = {"toilet":[2.0,-1.5,0.0],"pantry":[4.2,0.8,1.57],"meeting_room":[6.0,2.0,0.0]}
        self.srv = self.create_service(String, 'get_location', self.handle_get_location)

    def handle_get_location(self, request, response):
        label = request.data.strip().lower()
        if label in self.db:
            pose = self.db[label]
            response.data = json.dumps({"label":label,"x":pose[0],"y":pose[1],"theta":pose[2]})
        else:
            response.data = json.dumps({"error": f"label '{label}' not found"})
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
