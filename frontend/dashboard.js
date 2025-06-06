// WebSocket connection with auto-reconnect functionality
let ws = null;
let reconnectInterval = null;
let isReconnecting = false;

// Reconnection settings
const RECONNECT_INTERVAL = 1000; // 1 second
const MAX_RECONNECT_ATTEMPTS = Infinity; // Keep trying forever
let reconnectAttempts = 0;

// Interval at which faults element is updated
const FAULT_UPDATE_INTERVAL = 1000; // ms
// List of active faults
var faults = ['Websocket Disconnected'];
var fault_index = 0;

// Get all DOM elements
var faults_elm = document.getElementById('faults');
var bot_elm = document.getElementById('bot');
var brb_elm = document.getElementById('brb');
var imd_elm = document.getElementById('imd');
var bms_elm = document.getElementById('bms');
var cvc_elm = document.getElementById('cvc');
var drivestate_elm = document.getElementById('drivestate');

var acctemp_elm = document.getElementById('acctemp');
var leftinvtemp_elm = document.getElementById('leftinvtemp');
var rightinvtemp_elm = document.getElementById('rightinvtemp');

var throttlebar_elm = document.getElementById('throttlebar');
var throttleval_elm = document.getElementById('throttleval');

var rpm_elm = document.getElementById('rpm');
var speed_elm = document.getElementById('speed');
var lap_elm = document.getElementById('lap');
var laptime_elm = document.getElementById('laptime');

var batterybar_elm = document.getElementById('batterybar');
var batteryval_elm = document.getElementById('batteryval');

var hvvoltage_elm = document.getElementById('hvvoltage');
var acccurrent_elm = document.getElementById('acccurrent');
var range_elm = document.getElementById('range');

var tractioncontrol_elm = document.getElementById('tractioncontrol');
var mileage_elm = document.getElementById('mileage');
var vehiclestate_elm = document.getElementById('vehiclestate');

// Function to create WebSocket connection
function createWebSocketConnection() {
    console.log('Attempting to connect to WebSocket...');
    
    try {
        ws = new WebSocket("ws://localhost:9000", "can-protocol");
        
        ws.onopen = function() {
            console.log('Connected to server');
            reconnectAttempts = 0;
            isReconnecting = false;
            
            // Clear reconnect interval if it exists
            if (reconnectInterval) {
                clearInterval(reconnectInterval);
                reconnectInterval = null;
            }
            
            // Remove WebSocket disconnected fault
            if (faults.includes('Websocket Disconnected')) {
                faults = faults.filter(fault => fault !== 'Websocket Disconnected');
            }
        };

        ws.onerror = function(error) {
            console.log('WebSocket error:', error);
        };

        ws.onclose = function(event) {
            console.log('Disconnected from server. Code:', event.code, 'Reason:', event.reason);
            
            // Add WebSocket disconnected fault
            if (!faults.includes('Websocket Disconnected')) {
                faults.push('Websocket Disconnected');
            }
            
            // Start reconnection process if not already reconnecting
            if (!isReconnecting) {
                startReconnection();
            }
        };

        ws.onmessage = function(event) {
            const data = JSON.parse(event.data);
            
            // Handle CAN connection status
            if (!data.canconnected) {
                if (!faults.includes('CAN Disconnected')) {
                    faults.push('CAN Disconnected');
                }
            } else {
                faults = faults.filter(fault => fault !== 'CAN Disconnected');
            }

            // Handle BOT (Brake Over Travel) status
            if (data.bot) {
                bot_elm.classList.remove('text-bg-danger');
                bot_elm.classList.add('text-bg-success');
                faults = faults.filter(fault => fault !== 'BOT Pressed');
            } else {
                bot_elm.classList.remove('text-bg-success');
                bot_elm.classList.add('text-bg-danger');
                if (!faults.includes('BOT Pressed')) {
                    faults.push('BOT Pressed');
                }
            }

            // Handle BRB (Brake Ready Button) status
            if (data.brb) {
                brb_elm.classList.remove('text-bg-danger');
                brb_elm.classList.add('text-bg-success');
                faults = faults.filter(fault => fault !== 'BRB Pressed');
            } else {
                brb_elm.classList.remove('text-bg-success');
                brb_elm.classList.add('text-bg-danger');
                if (!faults.includes('BRB Pressed')) {
                    faults.push('BRB Pressed');
                }
            }

            // Handle IMD (Insulation Monitoring Device) status
            if (data.imd) {
                imd_elm.classList.remove('text-bg-danger');
                imd_elm.classList.add('text-bg-success');
                faults = faults.filter(fault => fault !== 'IMD Fault');
            } else {
                imd_elm.classList.remove('text-bg-success');
                imd_elm.classList.add('text-bg-danger');
                if (!faults.includes('IMD Fault')) {
                    faults.push('IMD Fault');
                }
            }

            // Handle BMS (Battery Management System) status
            if (data.bms) {
                bms_elm.classList.remove('text-bg-danger');
                bms_elm.classList.add('text-bg-success');
                faults = faults.filter(fault => fault !== 'BMS Fault');
            } else {
                bms_elm.classList.remove('text-bg-success');
                bms_elm.classList.add('text-bg-danger');
                if (!faults.includes('BMS Fault')) {
                    faults.push('BMS Fault');
                }
            }

            // Handle CVC (Control Voltage Check) status
            if (data.cvc_overflow) {
                cvc_elm.classList.remove('text-bg-success');
                cvc_elm.classList.add('text-bg-danger');
            } else {
                cvc_elm.classList.remove('text-bg-danger');
                cvc_elm.classList.add('text-bg-success');
            }
            cvc_elm.innerHTML = data.cvc_time + 'ms';

            // Handle drive states
            // Drive/Reverse: Green
            // Neutral/Precharging: Yellow
            // Discharged: Red      
            if (data.drive_state.toLowerCase() == 'drive' || data.drive_state.toLowerCase() == 'reverse') {
                drivestate_elm.classList.remove('text-bg-warning');
                drivestate_elm.classList.remove('text-bg-danger');
                drivestate_elm.classList.add('text-bg-success');
            } else if (data.drive_state.toLowerCase() == 'neutral' || data.drive_state.toLowerCase() == 'precharging') {
                drivestate_elm.classList.remove('text-bg-success');
                drivestate_elm.classList.remove('text-bg-danger');
                drivestate_elm.classList.add('text-bg-warning');
            } else {
                drivestate_elm.classList.remove('text-bg-success');
                drivestate_elm.classList.remove('text-bg-warning');
                drivestate_elm.classList.add('text-bg-danger');
            }
            drivestate_elm.innerHTML = data.drive_state;

            // Update temperature readings
            acctemp_elm.innerHTML = data.acctemp.toFixed(1) + ' °C';
            leftinvtemp_elm.innerHTML = data.leftinvtemp.toFixed(1) + ' °C';
            rightinvtemp_elm.innerHTML = data.rightinvtemp.toFixed(1) + ' °C';

            // Update throttle display
            throttlebar_elm.style.height = data.throttle + '%';
            throttleval_elm.innerHTML = data.throttle.toFixed(1) + '%';

            // Update RPM and speed
            rpm_elm.innerHTML = data.rpm.toFixed(0) + ' RPM';
            speed_elm.innerHTML = data.speed.toFixed(1) + ' MPH';

            // Update lap information
            lap_elm.innerHTML = 'Lap ' + data.lap;
            laptime_elm.innerHTML = 'Lap time: ' + data.lap_time;

            // Update battery display
            batterybar_elm.style.height = data.battery + '%';
            batteryval_elm.innerHTML = data.battery.toFixed(1) + '%';

            // Update high voltage readings
            hvvoltage_elm.innerHTML = data.accumulator_voltage.toFixed(1) + ' V';
            acccurrent_elm.innerHTML = data.accumulator_current.toFixed(1) + ' A';
            range_elm.innerHTML = data.estimated_range.toFixed(1) + ' mi';

            // Handle traction control status
            if (data.tractioncontrol) {
                tractioncontrol_elm.classList.remove('text-bg-dark');
                tractioncontrol_elm.classList.add('text-bg-success');
                tractioncontrol_elm.innerHTML = 'Traction Control On';
            } else {
                tractioncontrol_elm.classList.remove('text-bg-success');
                tractioncontrol_elm.classList.add('text-bg-dark');
                tractioncontrol_elm.innerHTML = 'Traction Control Off';
            }

            // Update mileage
            mileage_elm.innerHTML = data.mileage.toFixed(1) + ' KM';

            // Handle vehicle state with color coding
            vehiclestate_elm.innerHTML = data.vehicle_state;
            if (data.vehicle_state.toLowerCase().includes('precharge')) {
                // Yellow background for precharge states
                vehiclestate_elm.classList.remove('text-bg-danger', 'text-bg-success', 'text-bg-primary');
                vehiclestate_elm.classList.add('text-bg-warning');
            } else if (data.vehicle_state.toLowerCase() == 'ready to drive' || 
                       data.vehicle_state.toLowerCase() == 'buzzer' || 
                       data.vehicle_state.toLowerCase() == 'not ready to drive') {
                // Green background for normal operational states
                vehiclestate_elm.classList.remove('text-bg-danger', 'text-bg-warning', 'text-bg-primary');
                vehiclestate_elm.classList.add('text-bg-success');
            } else if (data.vehicle_state.toLowerCase() == 'charging') {
                // Blue background for charging
                vehiclestate_elm.classList.remove('text-bg-danger', 'text-bg-success', 'text-bg-warning');
                vehiclestate_elm.classList.add('text-bg-primary');
            } else {
                // Red background for error states
                vehiclestate_elm.classList.remove('text-bg-primary', 'text-bg-success', 'text-bg-warning');
                vehiclestate_elm.classList.add('text-bg-danger');
            }
        };
        
    } catch (error) {
        console.error('Failed to create WebSocket connection:', error);
        startReconnection();
    }
}

// Function to start reconnection process
function startReconnection() {
    if (isReconnecting) {
        return; // Already trying to reconnect
    }
    
    isReconnecting = true;
    console.log('Starting reconnection process...');
    
    reconnectInterval = setInterval(function() {
        if (reconnectAttempts < MAX_RECONNECT_ATTEMPTS) {
            reconnectAttempts++;
            console.log(`Reconnection attempt ${reconnectAttempts}...`);
            
            // Clean up existing connection if it exists
            if (ws) {
                ws.onopen = null;
                ws.onclose = null;
                ws.onerror = null;
                ws.onmessage = null;
                if (ws.readyState === WebSocket.CONNECTING || ws.readyState === WebSocket.OPEN) {
                    ws.close();
                }
                ws = null;
            }
            
            // Try to create new connection
            createWebSocketConnection();
            
            // Check if connection was successful after a short delay
            setTimeout(function() {
                if (ws && ws.readyState === WebSocket.OPEN) {
                    // Connection successful, stop reconnecting
                    clearInterval(reconnectInterval);
                    reconnectInterval = null;
                    isReconnecting = false;
                    console.log('Reconnection successful!');
                }
            }, 500);
        } else {
            console.log('Max reconnection attempts reached');
            clearInterval(reconnectInterval);
            reconnectInterval = null;
            isReconnecting = false;
        }
    }, RECONNECT_INTERVAL);
}

// Function to manually trigger reconnection (can be called from console or UI)
function reconnectWebSocket() {
    console.log('Manual reconnection triggered');
    if (ws) {
        ws.close();
    }
    startReconnection();
}

// Initialize the WebSocket connection
createWebSocketConnection();

// Fault cycling system
// If no faults, display "No Faults" in green
// Otherwise make background red and cycle through faults
setInterval(function() {
    if (faults.length == 0) {
        faults_elm.innerHTML = 'No Faults';
        faults_elm.classList.remove('text-bg-danger');
        faults_elm.classList.add('text-bg-success');
    } else {
        faults_elm.classList.remove('text-bg-success');
        faults_elm.classList.add('text-bg-danger');
        faults_elm.innerHTML = faults[fault_index];
        fault_index = (fault_index + 1) % faults.length;
    }
}, FAULT_UPDATE_INTERVAL);

// Optional: Add visibility change handler to reconnect when tab becomes visible
// This helps if the connection is lost while the tab was hidden
document.addEventListener('visibilitychange', function() {
    if (!document.hidden && ws && ws.readyState !== WebSocket.OPEN && !isReconnecting) {
        console.log('Tab became visible, checking connection...');
        startReconnection();
    }
});