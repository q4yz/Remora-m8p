component heater "Thermal safety, watchdog, and hardware PWM cutoff";

// --- Inputs ---
pin in float current_temp "Live temperature from Remora";
pin in float requested_temp "Target temperature from GUI";
pin in float pid_pwm_in "PWM output coming from the PID component";
pin in bit machine_enabled "True when machine is ON (not E-stopped)";

// --- User Parameters ---
param rw float min_valid_temp = 5.0 "Failsafe: Tripped if sensor disconnects";
param rw float max_valid_temp = 290.0 "Failsafe: Tripped on thermal runaway";
param rw float min_extrude_temp = 170.0 "Threshold for cold extrusion block";
param rw float heatsink_fan_temp = 50.0 "Threshold to turn on the cold-end fan";
param rw float watchdog_time = 20.0 "Seconds allowed without temperature rising";

// --- Outputs ---
pin out float safe_target_temp "Sent to PID command";
pin out float safe_pwm_out "Sent to Remora hardware heater pin";
pin out bit heatsink_fan "True if temp > heatsink_fan_temp";
pin out bit temp_ready "True if temp >= min_extrude_temp (sent to extruder)";

// Safety Faults (Wire these to an OR2 gate, then to E-stop)
pin out bit fault_range "True if sensor reads < 5 or > 290";
pin out bit fault_watchdog "True if heating command is sent but temp doesn't rise";

// Internal variables for the timer
variable double timer = 0.0;
variable double saved_temp = 0.0;

function _ fp;
;;

FUNCTION(_) {
    // 1. Hardware Failsafe / E-Stop handling
    // If E-stop is active, instantly sever the PWM line and force target to 0
    if (!machine_enabled) {
        safe_target_temp = 0.0;
        safe_pwm_out = 0.0;
        timer = 0.0; // Reset watchdog
    } else {
        safe_target_temp = requested_temp;
        safe_pwm_out = pid_pwm_in;
    }

    // 2. Sensor Range & Thermal Runaway Check
    if (current_temp < min_valid_temp || current_temp > max_valid_temp) {
        fault_range = 1;
        safe_pwm_out = 0.0; // Cut hardware power instantly
    } else {
        fault_range = 0;
    }

    // 3. Heatsink Fan Control
    heatsink_fan = (current_temp > heatsink_fan_temp);

    // 4. Extruder Ready Signal
    temp_ready = (current_temp >= min_extrude_temp);

    // 5. Heating Watchdog (Dead-Heater Detection)
    // If we want it hot, but it's cold, start counting
    if (machine_enabled && (safe_target_temp > current_temp + 3.0)) {
        timer += fperiod; // fperiod is the exact time since the last thread cycle (e.g. 0.001s)

        if (timer >= watchdog_time) {
            // 20 seconds have passed. Did it rise by at least 2 degrees?
            if (current_temp < (saved_temp + 2.0)) {
                fault_watchdog = 1;
                safe_pwm_out = 0.0; // Cut power, something is burning or broken
            }
            // Reset the timer and save the new baseline temp for the next 20s chunk
            timer = 0.0;
            saved_temp = current_temp;
        }
    } else {
        // Not actively trying to heat up, reset the watchdog logic
        timer = 0.0;
        saved_temp = current_temp;
        fault_watchdog = 0;
    }
}