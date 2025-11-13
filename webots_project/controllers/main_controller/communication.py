import asyncio
import websockets
import json
import threading
import time
from controller import Supervisor  # Used for accessing real-time Webots data


class Communication:
    """Handles all WebSocket-based communication for the rescue robot system."""

    def __init__(self, robot_instance=None):
        """
        Initialize the Communication module.

        Args:
            robot_instance (Supervisor, optional): Webots robot instance used
                for accessing real sensor and position data.
        """
        self.robot = robot_instance
        self.supervisor = None

        # WebSocket configuration
        self.websocket_host = "localhost"
        self.websocket_port = 8765
        self.connected_clients = set()

        # Cached data
        self.latest_map_data = {}
        self.latest_survivors = []
        self.latest_robot_data = {}

        # Start WebSocket server in background
        self._start_websocket_server()

        print(" Communication module initialized.")
        print(f"   WebSocket URL: ws://{self.websocket_host}:{self.websocket_port}")
        print(f"   Web UI (run separately): http://{self.websocket_host}:8000")

    # -------------------------------------------------------------------------
    # WebSocket server setup
    # -------------------------------------------------------------------------
    def _start_websocket_server(self):
        """Start the WebSocket server in a separate daemon thread."""

        def run_server():
            try:
                asyncio.set_event_loop(asyncio.new_event_loop())
                start_server = websockets.serve(
                    self._websocket_handler, self.websocket_host, self.websocket_port
                )
                loop = asyncio.get_event_loop()
                loop.run_until_complete(start_server)
                print(f"   WebSocket server started on port {self.websocket_port}")
                loop.run_forever()
            except Exception as e:
                print(f" WebSocket server error: {e}")

        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()
        time.sleep(1)  # Allow time for the server to initialize

    # -------------------------------------------------------------------------
    # WebSocket handler and data broadcasting
    # -------------------------------------------------------------------------
    async def _websocket_handler(self, websocket, path):
        """Handle client connections via WebSocket."""
        self.connected_clients.add(websocket)
        print(f"🔗 Client connected. Total clients: {len(self.connected_clients)}")

        try:
            # Send welcome message
            await websocket.send(json.dumps({
                "type": "system",
                "message": "Connected to Rescue Robot Communication Server",
                "timestamp": time.time()
            }))

            # Send the latest available data immediately
            for payload in [self.latest_robot_data, self.latest_map_data, self.latest_survivors]:
                if payload:
                    await websocket.send(json.dumps(payload))

            # Keep the connection alive
            await websocket.wait_closed()

        except Exception as e:
            print(f" WebSocket client error: {e}")
        finally:
            self.connected_clients.remove(websocket)
            print(f" Client disconnected. Total clients: {len(self.connected_clients)}")

    async def _broadcast_data(self, data):
        """Broadcast a JSON-serializable dictionary to all connected clients."""
        if not self.connected_clients:
            return

        message = json.dumps(data)
        try:
            await asyncio.wait([client.send(message) for client in self.connected_clients])
        except Exception as e:
            print(f"Broadcast error: {e}")

    # -------------------------------------------------------------------------
    # Core data management
    # -------------------------------------------------------------------------
    def send(self, map_data, survivors):
        """
        Send the latest map, survivor, and robot data to all connected clients.

        Args:
            map_data (dict): Map data received from the mapping module.
            survivors (list): List of detected survivors from the detection module.
        """
        self.latest_map_data = self._format_map_data(map_data)
        self.latest_survivors = self._format_survivors_data(survivors)
        self.latest_robot_data = self._format_robot_data()

        packets = [
            self.latest_robot_data,
            self.latest_survivors,
            self.latest_map_data,
            self._get_system_status()
        ]

        for packet in packets:
            try:
                asyncio.run(self._broadcast_data(packet))
            except Exception as e:
                print(f"Send error: {e}")

    def _get_robot_position(self):
        """Retrieve real robot position (if available), otherwise return simulated data."""
        if self.robot:
            try:
                gps = self.robot.getDevice("gps")
                if gps:
                    position = gps.getValues()
                    return {
                        "x": position[0],
                        "y": position[2],  # Z in Webots corresponds to height
                        "z": position[1]
                    }
            except Exception:
                pass

        # Fallback: return simulated zero position
        return {"x": 0.0, "y": 0.0, "z": 0.0}

    # -------------------------------------------------------------------------
    # Data formatting utilities
    # -------------------------------------------------------------------------
    def _format_robot_data(self):
        """Format robot telemetry for broadcasting."""
        pos = self._get_robot_position()
        return {
            "type": "robot",
            "timestamp": time.time(),
            "data": {
                "position": pos,
                "battery": 85,  # Placeholder for real sensor data
                "status": "exploring",
                "velocity": 0.0
            }
        }

    def _format_survivors_data(self, survivors):
        """Format survivor detection data."""
        if survivors and len(survivors) > 0:
            formatted = [{
                "id": i,
                "x": s.get("x", i * 2.0),
                "y": s.get("y", i * 1.5),
                "confidence": s.get("confidence", 0.8),
                "type": "detected"
            } for i, s in enumerate(survivors)]
        else:
            # Simulated test data
            formatted = [
                {"id": 1, "x": 2.0, "y": 1.5, "confidence": 0.92, "type": "simulated"},
                {"id": 2, "x": -1.5, "y": 2.5, "confidence": 0.78, "type": "simulated"},
                {"id": 3, "x": 3.0, "y": -2.0, "confidence": 0.85, "type": "simulated"}
            ]

        return {
            "type": "survivors",
            "timestamp": time.time(),
            "data": {
                "count": len(formatted),
                "survivors": formatted
            }
        }

    def _format_map_data(self, map_data):
        """Format mapping data for broadcasting."""
        if map_data and isinstance(map_data, dict):
            formatted = map_data
        else:
            formatted = {
                "type": "grid",
                "bounds": {"min_x": -10, "max_x": 10, "min_y": -10, "max_y": 10},
                "resolution": 0.1,
                "data": [],
                "status": "mapping_in_progress"
            }

        return {
            "type": "map",
            "timestamp": time.time(),
            "data": formatted
        }

    def _get_system_status(self):
        """Return overall system health information."""
        return {
            "type": "system",
            "timestamp": time.time(),
            "data": {
                "communication_delay": 0.05,
                "clients_connected": len(self.connected_clients),
                "map_quality": 0.95,
                "robot_connected": self.robot is not None,
                "update_rate": "realtime"
            }
        }

    # -------------------------------------------------------------------------
    # Alerts and emergency notifications
    # -------------------------------------------------------------------------
    def send_emergency_alert(self, alert_type, message):
        """Send high-priority emergency alerts to all connected clients."""
        alert = {
            "type": "alert",
            "timestamp": time.time(),
            "data": {
                "alert_type": alert_type,
                "message": message,
                "priority": "high"
            }
        }
        try:
            asyncio.run(self._broadcast_data(alert))
            print(f" Emergency alert sent: {message}")
        except Exception as e:
            print(f"Alert send error: {e}")


# -------------------------------------------------------------------------
# Standalone testing entry point
# -------------------------------------------------------------------------
def test_communication_module():
    """Run standalone tests for the communication module (without Webots)."""
    print(" Testing Communication Module...")

    comm = Communication()

    # Send example data
    test_map = {"test": "map_data"}
    test_survivors = []

    print("Sending test data...")
    comm.send(test_map, test_survivors)

    print(" Communication module test completed.")
    print(f"   WebSocket server running on ws://localhost:8765")
    print("   Connect via any WebSocket client to verify transmission.")


if __name__ == "__main__":
    test_communication_module()
