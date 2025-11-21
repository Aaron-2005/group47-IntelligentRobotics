/**
 * Rescue Robot Monitor - Web UI
 * Group 47 - Intelligent Robotics Course
 * Real-time monitoring of robot position, navigation, and sensor data
 */

class RescueRobotMonitor {
    constructor() {
        this.dataFile = '../robot_data.json';
        this.currentData = null;
        this.startTime = Date.now();
        this.updateInterval = 500; // Update every 500ms
        this.survivorMarkers = [];
        this.goalMarker = null;
        
        this.init();
    }
    
    init() {
        console.log('Rescue Robot Monitor Initializing...');
        this.setupEventListeners();
        this.startDataPolling();
        this.startUptimeCounter();
        
        console.log('Monitor ready - waiting for robot data...');
    }
    
    setupEventListeners() {
        console.log('Setting up UI interactions...');
    }
    
    startDataPolling() {
        setInterval(() => this.loadData(), this.updateInterval);
        this.loadData();
    }
    
    startUptimeCounter() {
        setInterval(() => this.updateUptime(), 1000);
    }
    
    async loadData() {
        try {
            const response = await fetch(`${this.dataFile}?t=${Date.now()}`);
            
            if (response.ok) {
                const data = await response.json();
                this.currentData = data;
                this.updateAllDisplays(data);
            }
        } catch (error) {
            // Robot might not be running yet
        }
    }
    
    updateAllDisplays(data) {
        this.updateSystemStatus(data);
        this.updateRobotDisplay(data);
        this.updateNavigationDisplay(data);
        this.updateSurvivorsDisplay(data);
        this.updateMappingDisplay(data);
        this.updateSensorsDisplay(data);
    }
    
    updateSystemStatus(data) {
        document.getElementById('last-update').textContent = 
            new Date().toLocaleTimeString();
        document.getElementById('update-count').textContent = 
            data.update_count || 0;
        document.getElementById('data-source').textContent = 
            data.system?.data_quality || 'Real Sensors';
    }
    
    updateRobotDisplay(data) {
        const robot = data.robot;
        if (!robot) return;
        
        const position = robot.position;
        
        document.getElementById('robot-pos').textContent = 
            `X: ${position.x.toFixed(2)}, Y: ${position.y.toFixed(2)}`;
        document.getElementById('robot-position').textContent = 
            `X: ${position.x.toFixed(2)}, Y: ${position.y.toFixed(2)}`;
        
        document.getElementById('robot-orientation').textContent = 
            `${position.theta_degrees || 0}°`;
        
        const batteryElement = document.getElementById('robot-battery');
        batteryElement.textContent = `${robot.battery}%`;
        this.setBatteryColor(batteryElement, robot.battery);
        
        document.getElementById('robot-velocity').textContent = 
            `${robot.velocity || 0} m/s`;
        
        this.updateRobotPosition(position);
    }
    
    updateRobotPosition(position) {
        const robotMarker = document.getElementById('robot-marker');
        if (!robotMarker) return;
        
        const scale = 20;
        const centerX = 200;
        const centerY = 200;
        
        const mapX = centerX + (position.x * scale);
        const mapY = centerY - (position.y * scale);
        
        const boundedX = Math.max(30, Math.min(370, mapX));
        const boundedY = Math.max(30, Math.min(370, mapY));
        
        robotMarker.style.left = `${boundedX}px`;
        robotMarker.style.top = `${boundedY}px`;
        
        const rotation = position.theta_degrees || 0;
        robotMarker.style.transform = `translate(-50%, -50%) rotate(${rotation}deg)`;
    }
    
    setBatteryColor(element, battery) {
        if (battery > 70) {
            element.style.color = '#4ade80';
        } else if (battery > 30) {
            element.style.color = '#fbbf24';
        } else {
            element.style.color = '#ef4444';
        }
    }
    
    updateNavigationDisplay(data) {
        const nav = data.navigation;
        if (!nav) return;
        
        document.getElementById('nav-state').textContent = nav.current_state || '--';
        document.getElementById('current-state').textContent = nav.current_state || '--';
        document.getElementById('nav-algorithm').textContent = nav.algorithm || '--';
        document.getElementById('movement-status').textContent = nav.movement_status || '--';
        document.getElementById('goal-pos').textContent = 
            Array.isArray(nav.goal_position) ? `(${nav.goal_position[0]}, ${nav.goal_position[1]})` : '--';
        
        const obstacleElement = document.getElementById('obstacle-status');
        obstacleElement.textContent = nav.obstacle_detected ? 'YES' : 'NO';
        obstacleElement.className = nav.obstacle_detected ? 'status-warning' : 'status-good';
        
        this.updateGoalMarker(nav.goal_position);
    }
    
    updateGoalMarker(goalPosition) {
        if (!Array.isArray(goalPosition)) return;
        
        const [goalX, goalY] = goalPosition;
        const scale = 20;
        const centerX = 200;
        const centerY = 200;
        
        const mapX = centerX + (goalX * scale);
        const mapY = centerY - (goalY * scale);
        
        if (!this.goalMarker) {
            this.createGoalMarker(mapX, mapY);
        } else {
            this.goalMarker.style.left = `${mapX}px`;
            this.goalMarker.style.top = `${mapY}px`;
        }
    }
    
    createGoalMarker(x, y) {
        const coordinateSystem = document.getElementById('coordinate-system');
        this.goalMarker = document.createElement('div');
        this.goalMarker.className = 'goal-marker';
        this.goalMarker.style.left = `${x}px`;
        this.goalMarker.style.top = `${y}px`;
        this.goalMarker.innerHTML = `
            <div class="goal-icon"></div>
            <div class="goal-info">Goal</div>
        `;
        coordinateSystem.appendChild(this.goalMarker);
    }
    
    updateSurvivorsDisplay(data) {
        const survivors = data.survivors || [];
        
        document.getElementById('survivors-count').textContent = survivors.length;
        
        this.updateSurvivorsList(survivors);
        this.updateSurvivorMarkers(survivors);
    }
    
    updateSurvivorsList(survivors) {
        const container = document.getElementById('survivors-list');
        
        if (survivors.length === 0) {
            container.innerHTML = '<div class="no-data">No survivors detected yet</div>';
            return;
        }
        
        container.innerHTML = survivors.map(survivor => `
            <div class="survivor-item">
                <div class="survivor-header">
                    <span>Survivor #${survivor.id}</span>
                    <span class="survivor-confidence">${(survivor.confidence * 100).toFixed(1)}%</span>
                </div>
                <div class="survivor-details">
                    <span>Position: X${survivor.x.toFixed(2)}, Y${survivor.y.toFixed(2)}</span>
                    <span>Status: ${survivor.status}</span>
                </div>
                <div class="survivor-details">
                    <span>Data Source: ${survivor.data_source}</span>
                </div>
            </div>
        `).join('');
    }
    
    updateSurvivorMarkers(survivors) {
        this.survivorMarkers.forEach(marker => marker.remove());
        this.survivorMarkers = [];
        
        const coordinateSystem = document.getElementById('coordinate-system');
        const scale = 20;
        const centerX = 200;
        const centerY = 200;
        
        survivors.forEach(survivor => {
            const mapX = centerX + (survivor.x * scale);
            const mapY = centerY - (survivor.y * scale);
            
            const marker = document.createElement('div');
            marker.className = 'survivor-marker';
            marker.style.left = `${mapX}px`;
            marker.style.top = `${mapY}px`;
            marker.innerHTML = `
                <div class="survivor-icon"></div>
                <div class="survivor-info">Survivor #${survivor.id}</div>
            `;
            
            coordinateSystem.appendChild(marker);
            this.survivorMarkers.push(marker);
        });
    }
    
    updateMappingDisplay(data) {
        const mapping = data.mapping;
        if (!mapping) return;
        
        document.getElementById('map-status').textContent = mapping.status || '--';
        document.getElementById('map-size').textContent = mapping.map_size || '--';
        document.getElementById('map-resolution').textContent = mapping.resolution || '--';
        document.getElementById('map-source').textContent = mapping.data_source || '--';
    }
    
    updateSensorsDisplay(data) {
        const sensors = data.sensors;
        if (!sensors) return;
        
        document.getElementById('lidar-status').textContent = 
            `${sensors.lidar?.status || '--'} (${sensors.lidar?.points || 0} points)`;
        document.getElementById('camera-status').textContent = sensors.camera?.status || '--';
        document.getElementById('imu-status').textContent = sensors.imu?.status || '--';
    }
    
    updateUptime() {
        const uptime = Math.floor((Date.now() - this.startTime) / 1000);
        const hours = Math.floor(uptime / 3600);
        const minutes = Math.floor((uptime % 3600) / 60);
        const seconds = uptime % 60;
    }
}

document.addEventListener('DOMContentLoaded', () => {
    new RescueRobotMonitor();
});

