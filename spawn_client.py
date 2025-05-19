import sys
import rclpy
from gazebo_msgs.srv import SpawnEntity
from xml.etree import ElementTree as ET  # For XML parsing

def main():
    rclpy.init()

    if len(sys.argv) < 7:  # Updated argument count
        print("Usage: spawn_client.py <robot_name> <x> <y> <namespace> <urdf_path> <group_name>")
        sys.exit(1)

    robot_name = sys.argv[1]
    x = float(sys.argv[2])
    y = float(sys.argv[3])
    namespace = sys.argv[4]
    urdf_file_path = sys.argv[5]
    group_name = sys.argv[6]  # New argument: arm_a or arm_b

    node = rclpy.create_node(f'spawn_{robot_name}')
    client = node.create_client(SpawnEntity, "/spawn_entity")

    node.get_logger().info("Waiting for `/spawn_entity` service...")
    client.wait_for_service()

    # --- URDF Parsing ---
    try:
        tree = ET.parse(urdf_file_path)
        root = tree.getroot()

        # Find the correct group and extract the robot description
        target_group = None
        for group in root.findall('group'):
            if group.get('name') == group_name:
                target_group = group
                break

        if target_group is not None:
            robot_element = target_group.find('robot')
            if robot_element is not None:
                robot_description_xml = ET.tostring(robot_element, encoding='utf-8').decode('utf-8')
            else:
                raise ValueError(f"No 'robot' element found in group '{group_name}'")
        else:
            raise ValueError(f"Group '{group_name}' not found in URDF")

    except ET.ParseError as e:
        node.get_logger().error(f"Error parsing URDF: {e}")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    except ValueError as e:
        node.get_logger().error(str(e))
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    # --- End URDF Parsing ---

    request = SpawnEntity.Request()
    request.name = robot_name
    request.robot_namespace = namespace
    request.xml = robot_description_xml  # Use the extracted robot description
    request.initial_pose.position.x = x
    request.initial_pose.position.y = y
    request.initial_pose.position.z = 0.1

    node.get_logger().info(f"Spawning {robot_name} from group '{group_name}' in namespace {namespace} at ({x}, {y})")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(f"Spawned: {future.result().status_message}")
    else:
        node.get_logger().error(f"Failed to spawn: {future.exception()}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
