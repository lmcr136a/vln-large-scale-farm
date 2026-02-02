"""
Get Path Mode
Collect clicked points on map and prepare for autonomous driving
"""


class PathPlanner:
    def __init__(self, config):
        self.config = config
        self.path_nodes = []
        self.is_path_mode = False
        
    def handle_map_clicked(self, data):
        """Handle map click event"""
        img_x = data.get('img_x')
        img_y = data.get('img_y')
        world_x = data.get('world_x')
        world_y = data.get('world_y')
        
        print(f"Map clicked - Image: ({img_x}, {img_y}), World: ({world_x:.2f}, {world_y:.2f})")
        
        # Add waypoint in path mode
        if self.is_path_mode:
            self.add_waypoint(world_x, world_y)
    
    def add_waypoint(self, world_x, world_y):
        """Add waypoint"""
        waypoint = {
            'x': world_x,
            'y': world_y
        }
        self.path_nodes.append(waypoint)
        print(f"Waypoint added: ({world_x:.2f}, {world_y:.2f}) - Total: {len(self.path_nodes)}")
    
    def toggle_path_mode(self):
        """Toggle path mode"""
        self.is_path_mode = not self.is_path_mode
        
        if self.is_path_mode:
            print("Path mode enabled")
            self.path_nodes = []
        else:
            print("Map creation mode enabled")
            self.path_nodes = []
        
        return self.is_path_mode
    
    def get_waypoints(self):
        """Return waypoint list"""
        return self.path_nodes
    
    def validate_waypoints(self):
        """Validate waypoints"""
        min_waypoints = self.config['autonomous']['min_waypoints']
        if len(self.path_nodes) < min_waypoints:
            return False, f"Need at least {min_waypoints} waypoints (current: {len(self.path_nodes)})"
        return True, f"Valid path with {len(self.path_nodes)} waypoints"
    
    def clear_waypoints(self):
        """Delete all waypoints"""
        self.path_nodes = []
        print("All waypoints cleared")
    
    def is_in_path_mode(self):
        """Return path mode status"""
        return self.is_path_mode