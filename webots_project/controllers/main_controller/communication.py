# communication.py
# Handles data transfer and UI updates

class Communication:
    def __init__(self):
        print("Communication module initialized")

    def send(self, map_data, survivors):
        # Placeholder communication logic
        # Later: send data to UI or print results
        pass
def __init__(self, robot_instance=None):
    """
    Initialize communication module
    
    Args:
        robot_instance: Webots robot instance (optional)
    """
    self.robot = robot_instance
    self.data_file = "robot_data.json"
    self.start_time = time.time()
    
    print(" Communication module initialized")
    print("   Mode: File-based JSON communication")
    print("   Data file: " + self.data_file)
    print("   Web UI: Open web_ui/index.html in browser")

def send(self, map_data, survivors):
    """
    Send data to web UI
    Generates realistic simulation data for demonstration
    
    Args:
        map_data: Mapping data from SLAM module (not used in simulation)
        survivors: Survivor detection data (not used in simulation)
    """
    # Calculate simulation time
    current_time = time.time() - self.start_time
    
    # Generate realistic simulation data
    robot_data = self._generate_robot_data(current_time)
    survivor_data = self._generate_survivor_data()
    system_data = self._generate_system_data(current_time)
    
    # Combine all data
    complete_data = {
        "timestamp": time.time(),
        "robot": robot_data,
        "survivors": survivor_data,
        "map": {
            "bounds": {"min_x": -10, "max_x": 10, "min_y": -10, "max_y": 10},
            "type": "occupancy_grid",
            "resolution": 0.1,
            "status": "mapping_in_progress"
        },
        "system": system_data
    }
    
    # Save data to JSON file
    self._save_to_file(complete_data)
    
    # Print status every 5 seconds
    if int(current_time) % 5 == 0:
        print(f"📡 Data sent - Survivors: {len(survivor_data)}, "
              f"Battery: {robot_data['battery']}%, "
              f"Position: ({robot_data['position']['x']:.1f}, {robot_data['position']['y']:.1f})")

def _generate_robot_data(self, current_time):
    """Generate realistic robot simulation data"""
    # Simulate robot moving in a search pattern
    search_radius = 8.0
    center_x, center_y = 0, 0
    
    # Circular search pattern with slight randomness
    angle = current_time * 0.3  # Angular velocity
    radius_variation = 0.5 * (1 + 0.3 * (time.time() % 10))
    
    x = center_x + search_radius * radius_variation * (0.7 * (1 + 0.2 * (time.time() % 8)))
    y = center_y + search_radius * radius_variation * (0.7 * (1 + 0.2 * (time.time() % 6)))
    
    # Simulate battery drain (from 100% to 20% over 10 minutes)
    battery = max(20, 100 - (current_time / 60) * 8)
    
    # Robot status based on behavior
    if int(current_time) % 20 < 5:
        status = "scanning_area"
    elif int(current_time) % 20 < 10:
        status = "moving_to_target"
    else:
        status = "exploring"
    
    return {
        "position": {
            "x": round(x, 2),
            "y": round(y, 2),
            "z": 0.0
        },
        "battery": int(battery),
        "status": status,
        "velocity": round(0.5 + 0.3 * (time.time() % 1), 2),
        "orientation": round((angle * 180 / 3.14159) % 360, 1)
    }

def _generate_survivor_data(self):
    """Generate survivor detection data"""
    # Fixed survivor positions for consistent demonstration
    survivors = [
        {
            "id": 1,
            "x": 3.2,
            "y": 1.8,
            "confidence": 0.92,
            "status": "detected",
            "type": "human"
        },
        {
            "id": 2, 
            "x": -2.5,
            "y": 4.1,
            "confidence": 0.78,
            "status": "detected",
            "type": "human"
        },
        {
            "id": 3,
            "x": 5.7,
            "y": -3.2,
            "confidence": 0.85,
            "status": "detected", 
            "type": "human"
        }
    ]
    
    return survivors

def _generate_system_data(self, current_time):
    """Generate system status data"""
    return {
        "update_rate": "realtime",
        "communication_delay": 0.05,
        "map_quality": 0.95,
        "operational_time": int(current_time),
        "system_status": "nominal",
        "clients_connected": 1
    }

def _save_to_file(self, data):
    """Save data to JSON file"""
    try:
        with open(self.data_file, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
    except Exception as e:
        print(f" Error saving data: {e}")

def send_emergency_alert(self, alert_type, message):
    """Send emergency alert (for future use)"""
    alert_data = {
        "type": "alert",
        "timestamp": time.time(),
        "data": {
            "alert_type": alert_type,
            "message": message,
            "priority": "high"
        }
    }
    self._save_to_file(alert_data)
    print(f" Emergency alert: {message}")
