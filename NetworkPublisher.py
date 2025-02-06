import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String
import yaml
import networkx as nx

GLOBAL_PATH = ['V_1', 'V_0', 'V_56', 'V_55', 'V_20', 'V_69', 'V_68', 'V_67', 'V_70', 
               'V_71', 'V_72', 'V_73', 'V_74', 'V_54', 'V_75', 'V_76', 'V_77', 'V_78', 'V_97',
               'V_5', 'V_98', '']    

def load_network():
    # Load network
    with open('inffeld.yaml', 'r') as file:
        data = yaml.safe_load(file)
    
    G = nx.Graph()
    
    for graph in data['graphs']: 
        for vertex in graph['vertices']:
            G.add_node(vertex['name'], position=(vertex['x'], vertex['y'], vertex['z']))
        
        for edge in graph['edges']:
            G.add_edge(edge['source'], edge['sink'], costs=edge['costs'])
    
    return G

def graph_to_markers(graph):
    marker_array = MarkerArray()

    id = 0
    # Create markers for nodes
    for node, attr in graph.nodes(data=True):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "vertex_text"
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = attr['position'][0]
        marker.pose.position.y = attr['position'][1]
        marker.pose.position.z = attr['position'][2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker_array.markers.append(marker)
        id += 1
    
        # Create the text marker for the node label
        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = "nodes"
        text_marker.id = id + 1000  # Make sure it's a unique ID (e.g., 1000 offset)
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = attr['position'][0]
        text_marker.pose.position.y = attr['position'][1]
        text_marker.pose.position.z = attr['position'][2] + 0.5  # Slightly above the node
        text_marker.pose.orientation.x = 0.0
        text_marker.pose.orientation.y = 0.0
        text_marker.pose.orientation.z = 0.0
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.x = 0.5  # Text size
        text_marker.scale.y = 0.2
        text_marker.scale.z = 0.2
        text_marker.color.a = 1.0
        text_marker.color.r = 0.0
        text_marker.color.g = 0.0
        text_marker.color.b = 1.0
        text_marker.text = str(node)  # Add the node label as text
        marker_array.markers.append(text_marker)

    # Create markers for edges
    edge_id = 0
    for u, v in graph.edges():
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "edges"
        marker.id = edge_id
        edge_id += 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # Line width
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Add points for the edge
        p1 = Point(*graph.nodes[u]['position'])
        p2 = Point(*graph.nodes[v]['position'])
        marker.points.append(p1)
        marker.points.append(p2)

        marker_array.markers.append(marker)

    return marker_array

def publish_road_graph(graph):
    """ Convert networkx graph to MarkerArray and publish it """
    marker_array = MarkerArray()
    marker_id = 0  # Unique ID counter for markers

    # Publish Nodes (Vertices)
    for node, attr in graph.nodes(data=True):
        marker = Marker()
        marker.header.frame_id = "nav_sat"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "vertex_text"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = attr['position'][0]
        marker.pose.position.y = attr['position'][1]
        marker.pose.position.z = attr['position'][2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.text = str(node)
        marker_array.markers.append(marker)
        marker_id += 1  # Increment unique marker ID

        # Create the text marker for the node label
        text_marker = Marker()
        text_marker.header.frame_id = "nav_sat"
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = "nodes"
        text_marker.id = marker_id + 1000  # Make sure it's a unique ID (e.g., 1000 offset)
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = attr['position'][0] + 0.5 # Slightly above the node
        text_marker.pose.position.y = attr['position'][1] + 0.5  # Slightly above the node
        text_marker.pose.position.z = attr['position'][2] + 0.5  # Slightly above the node
        text_marker.pose.orientation.x = 0.0
        text_marker.pose.orientation.y = 0.0
        text_marker.pose.orientation.z = 0.0
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.x = 0.5  # Text size
        text_marker.scale.y = 0.2
        text_marker.scale.z = 0.2
        text_marker.color.a = 1.0
        text_marker.color.r = 0.0
        text_marker.color.g = 1.0
        text_marker.color.b = 0.0
        text_marker.text = str(node)  # Add the node label as text
        marker_array.markers.append(text_marker)


    # Publish Edges (Connections)
    for u, v, data in graph.edges(data=True):
        edge_marker = Marker()
        edge_marker.header.frame_id = "nav_sat"
        edge_marker.header.stamp = rospy.Time.now()
        edge_marker.ns = "edges"
        edge_marker.id = marker_id
        edge_marker.type = Marker.LINE_STRIP
        edge_marker.action = Marker.ADD
        edge_marker.scale.x = 0.1  # Line thickness
        edge_marker.color.a = 1.0
        edge_marker.color.r = 0.0
        edge_marker.color.g = 0.0
        edge_marker.color.b = 1.0
        edge_marker.pose.orientation.x = 0.0
        edge_marker.pose.orientation.y = 0.0
        edge_marker.pose.orientation.z = 0.0
        edge_marker.pose.orientation.w = 1.0
        

        # Add edge points
        p1 = Point(*graph.nodes[u]['position'])
        p2 = Point(*graph.nodes[v]['position'])
        edge_marker.points.append(p1)
        edge_marker.points.append(p2)

        marker_array.markers.append(edge_marker)
        marker_id += 1  # Increment unique marker ID

    return marker_array

def graph_publisher():
    rospy.init_node('graph_visualizer')
    pub_graph = rospy.Publisher('/arti_move_base/network_planner/planner_processing/navigation_network_interpolated', MarkerArray, queue_size=10)
    pub_goal = rospy.Publisher('/arti_move_base/network_planner/planner_processing/global_graph_network', String, queue_size=10)
    
    rate = rospy.Rate(10)

    msg = String()
    msg.data = ';'.join(GLOBAL_PATH)

    graph = load_network()
    while not rospy.is_shutdown():
        # marker_array = graph_to_markers(graph)
        marker_array = publish_road_graph(graph)
        pub_graph.publish(marker_array)
        pub_goal.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        graph_publisher()
    except rospy.ROSInterruptException:
        rospy.logerr("Could not start graph publisher")