// dashboard.js - New version using WebSocket API (instead of socket.io)
// This script listens for JSON updates from the backend and updates the dashboard accordingly.

// WebSocket connection URL: adjust the host and port if needed.
const ws = new WebSocket("ws://" + location.hostname + ":8080");

// Fault update interval (in ms)
const FAULT_UPDATE_INTERVAL = 1500;

// Faults list and index for cycling display
let faults = ["Websocket Disconnected"];
let fault_index = 0;

// Cache references to dashboard elements
const faults_elm = document.getElementById("faults");
const bot_elm = document.getElementById("bot");
const brb_elm = document.getElementById("brb");
const imd_elm = document.getElementById("imd");
const bms_elm = document.getElementById("bms");
const cvc_elm = document.getElementById("cvc");
const drivestate_elm = document.getElementById("drivestate");
const acctemp_elm = document.getElementById("acctemp");
const leftinvtemp_elm = document.getElementById("leftinvtemp");
const rightinvtemp_elm = document.getElementById("rightinvtemp");
const throttlebar_elm = document.getElementById("throttlebar");
const throttleval_elm = document.getElementById("throttleval");
const rpm_elm = document.getElementById("rpm");
const speed_elm = document.getElementById("speed");
const lap_elm = document.getElementById("lap");
const laptime_elm = document.getElementById("laptime");
const batterybar_elm = document.getElementById("batterybar");
const batteryval_elm = document.getElementById("batteryval");
const hvvoltage_elm = document.getElementById("hvvoltage");
const acccurrent_elm = document.getElementById("acccurrent");
const range_elm = document.getElementById("range");
const tractioncontrol_elm = document.getElementById("tractioncontrol");
const mileage_elm = document.getElementById("mileage");
const vehiclestate_elm = document.getElementById("vehiclestate");

// WebSocket event handlers
ws.onopen = () => {
    console.log("Connected to server via WS");
    // Remove the "Websocket Disconnected" fault if present
    faults = faults.filter(fault => fault !== "Websocket Disconnected");
};

ws.onerror = (error) => {
    console.error("WebSocket error:", error);
};

ws.onclose = () => {
    console.log("Disconnected from server");
    if (!faults.includes("Websocket Disconnected")) {
        faults.push("Websocket Disconnected");
    }
};

ws.onmessage = (event) => {
    try {
        const data = JSON.parse(event.data);
        console.log("Received data:", data);

        // Handle CAN connection fault detection.
        if (!data.canconnected) {
            if (!faults.includes("CAN Disconnected")) {
                faults.push("CAN Disconnected");
            }
        } else {
            faults = faults.filter(f => f !== "CAN Disconnected");
        }

        // BOT indicator update.
        if (data.bot) {
            bot_elm.classList.remove("text-bg-danger");
            bot_elm.classList.add("text-bg-success");
            faults = faults.filter(f => f !== "BOT Pressed");
        } else {
            bot_elm.classList.remove("text-bg-success");
            bot_elm.classList.add("text-bg-danger");
            if (!faults.includes("BOT Pressed")) {
                faults.push("BOT Pressed");
            }
        }

        // BRB indicator update.
        if (data.brb) {
            brb_elm.classList.remove("text-bg-danger");
            brb_elm.classList.add("text-bg-success");
            faults = faults.filter(f => f !== "BRB Pressed");
        } else {
            brb_elm.classList.remove("text-bg-success");
            brb_elm.classList.add("text-bg-danger");
            if (!faults.includes("BRB Pressed")) {
                faults.push("BRB Pressed");
            }
        }

        // IMD indicator update.
        if (data.imd) {
            imd_elm.classList.remove("text-bg-danger");
            imd_elm.classList.add("text-bg-success");
            faults = faults.filter(f => f !== "IMD Fault");
        } else {
            imd_elm.classList.remove("text-bg-success");
            imd_elm.classList.add("text-bg-danger");
            if (!faults.includes("IMD Fault")) {
                faults.push("IMD Fault");
            }
        }

        // BMS indicator update.
        if (data.bms) {
            bms_elm.classList.remove("text-bg-danger");
            bms_elm.classList.add("text-bg-success");
            faults = faults.filter(f => f !== "BMS Fault");
        } else {
            bms_elm.classList.remove("text-bg-success");
            bms_elm.classList.add("text-bg-danger");
            if (!faults.includes("BMS Fault")) {
                faults.push("BMS Fault");
            }
        }

        // CVC Overflow update.
        if (data.cvc_overflow) {
            cvc_elm.classList.remove("text-bg-success");
            cvc_elm.classList.add("text-bg-danger");
        } else {
            cvc_elm.classList.remove("text-bg-danger");
            cvc_elm.classList.add("text-bg-success");
        }
        cvc_elm.innerHTML = data.cvc_time + "ms";

        // Drive state update - change background color based on state.
        let ds = data.drive_state.toLowerCase();
        if (ds === "drive" || ds === "reverse") {
            drivestate_elm.classList.remove("text-bg-warning", "text-bg-danger");
            drivestate_elm.classList.add("text-bg-success");
        } else if (ds === "neutral" || ds === "precharging") {
            drivestate_elm.classList.remove("text-bg-success", "text-bg-danger");
            drivestate_elm.classList.add("text-bg-warning");
        } else {
            drivestate_elm.classList.remove("text-bg-success", "text-bg-warning");
            drivestate_elm.classList.add("text-bg-danger");
        }
        drivestate_elm.innerHTML = data.drive_state;

        // Temperature updates (formatted to one decimal place).
        if (data.acctemp !== undefined) {
            acctemp_elm.innerHTML = parseFloat(data.acctemp).toFixed(1) + " °C";
        }
        if (data.leftinvtemp !== undefined) {
            leftinvtemp_elm.innerHTML = parseFloat(data.leftinvtemp).toFixed(1) + " °C";
        }
        if (data.rightinvtemp !== undefined) {
            rightinvtemp_elm.innerHTML = parseFloat(data.rightinvtemp).toFixed(1) + " °C";
        }

        // Throttle indicator update.
        if (data.throttle_position !== undefined) {
            throttlebar_elm.style.height = data.throttle_position + "%";
            throttleval_elm.innerHTML = parseFloat(data.throttle_position).toFixed(1) + "%";
        }

        // RPM and Speed updates.
        if (data.rpm !== undefined) {
            rpm_elm.innerHTML = parseFloat(data.rpm).toFixed(0) + " RPM";
        }
        if (data.speed !== undefined) {
            speed_elm.innerHTML = parseFloat(data.speed).toFixed(1) + " MPH";
        }

        // Lap information.
        if (data.lap !== undefined) {
            lap_elm.innerHTML = "Lap " + data.lap;
        }
        if (data.lap_time !== undefined) {
            laptime_elm.innerHTML = "Lap time: " + data.lap_time;
        }

        // Battery updates.
        if (data.battery_percentage !== undefined) {
            batterybar_elm.style.height = data.battery_percentage + "%";
            batteryval_elm.innerHTML = parseFloat(data.battery_percentage).toFixed(1) + "%";
        }
        if (data.accumulator_voltage !== undefined) {
            hvvoltage_elm.innerHTML = parseFloat(data.accumulator_voltage).toFixed(1) + " V";
        }
        if (data.accumulator_current !== undefined) {
            acccurrent_elm.innerHTML = parseFloat(data.accumulator_current).toFixed(1) + " A";
        }
        if (data.estimated_range !== undefined) {
            range_elm.innerHTML = parseFloat(data.estimated_range).toFixed(1) + " mi";
        }

        // Traction control indicator.
        if (data.tractioncontrol) {
            tractioncontrol_elm.classList.remove("text-bg-dark");
            tractioncontrol_elm.classList.add("text-bg-success");
            tractioncontrol_elm.innerHTML = "Traction Control On";
        } else {
            tractioncontrol_elm.classList.remove("text-bg-success");
            tractioncontrol_elm.classList.add("text-bg-dark");
            tractioncontrol_elm.innerHTML = "Traction Control Off";
        }

        // Mileage update.
        if (data.mileage !== undefined) {
            mileage_elm.innerHTML = parseFloat(data.mileage).toFixed(1) + " Miles";
        }

        // Vehicle state update with color coding.
        if (data.vehicle_state !== undefined) {
            vehiclestate_elm.innerHTML = data.vehicle_state;
            let vs = data.vehicle_state.toLowerCase();
            vehiclestate_elm.classList.remove("text-bg-danger", "text-bg-success", "text-bg-warning", "text-bg-primary");
            if (vs.includes("precharge")) {
                vehiclestate_elm.classList.add("text-bg-warning");
            } else if (vs === "ready to drive" || vs === "buzzer" || vs === "not ready to drive") {
                vehiclestate_elm.classList.add("text-bg-success");
            } else if (vs === "charging") {
                vehiclestate_elm.classList.add("text-bg-primary");
            } else {
                vehiclestate_elm.classList.add("text-bg-danger");
            }
        }
    } catch (error) {
        console.error("Error processing message:", error);
    }
};

// Periodically update the faults element.
// If there are no faults, display "No Faults" with a green background.
setInterval(() => {
    if (faults.length === 0) {
        faults_elm.innerHTML = "No Faults";
        faults_elm.classList.remove("text-bg-danger");
        faults_elm.classList.add("text-bg-success");
    } else {
        faults_elm.classList.remove("text-bg-success");
        faults_elm.classList.add("text-bg-danger");
        faults_elm.innerHTML = faults[fault_index];
        fault_index = (fault_index + 1) % faults.length;
    }
}, FAULT_UPDATE_INTERVAL);