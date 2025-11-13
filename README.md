
#!/usr/bin/env python3
"""
autonomous_wheelchair.py
Raspberry Pi-based autonomous wheelchair (simple waypoint navigation + GPS + IoT)

Features:
- Reads NMEA GPS (via serial)
- Simple waypoint following using distance and bearing
- Ultrasonic obstacle avoidance (HC-SR04)
- Motor control via L298N using GPIO PWM
- MQTT telemetry + remote override (stop / manual / goto / set_speed)
- Safety: emergency stop on obstacle or on MQTT "stop"

IMPORTANT: This is example code. Test in controlled environment only. Add hardware failsafe!
"""

import RPi.GPIO as GPIO
import time
import math
import serial
import pynmea2
import threading
import json
import paho.mqtt.client as mqtt

# -------------------------
# Configuration
# -------------------------
# GPIO pins (BCM numbering)
IN1 = 17
IN2 = 27
IN3 = 22
IN4 = 23
ENA = 24  # PWM left
ENB = 25  # PWM right

TRIG = 5   # HC-SR04 TRIG
ECHO = 6   # HC-SR04 ECHO

# GPS serial port (adjust per your setup)
GPS_PORT = "/dev/serial0"  # or "/dev/ttyUSB0" for USB-serial GPS
GPS_BAUD = 9600

# MQTT
MQTT_BROKER = "broker.hivemq.com"
MQTT_PORT = 1883
CMD_TOPIC = "wheelchair/commands"
TELE_TOPIC = "wheelchair/telemetry"

# Navigation params
WAYPOINTS = [
    # Example waypoints: (lat, lon)
    (12.9715987, 77.594566),  # replace with your coordinates
    (12.9717, 77.5950)
]
REACHED_THRESHOLD_M = 3.0  # meters to consider waypoint reached
MAX_SPEED = 70  # percent PWM (0-100)
TURN_SPEED = 50
OBSTACLE_STOP_CM = 40.0

TELEMETRY_INTERVAL = 2.0  # seconds

# -------------------------
# Setup GPIO
# -------------------------
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

motor_pins = [IN1, IN2, IN3, IN4, ENA, ENB]
GPIO.setup([IN1, IN2, IN3, IN4, TRIG], GPIO.OUT)
GPIO.setup([ECHO], GPIO.IN)
GPIO.setup([ENA, ENB], GPIO.OUT)

# Setup PWM
pwm_left = GPIO.PWM(ENA, 100)   # 100Hz
pwm_right = GPIO.PWM(ENB, 100)
pwm_left.start(0)
pwm_right.start(0)

# -------------------------
# Motor control helpers
# -------------------------
def stop_motors():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)

def set_left(speed_percent):
    # positive -> forward, negative -> backward
    if speed_percent >= 0:
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        pwm_left.ChangeDutyCycle(min(abs(speed_percent), 100))
    else:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        pwm_left.ChangeDutyCycle(min(abs(speed_percent), 100))

def set_right(speed_percent):
    if speed_percent >= 0:
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        pwm_right.ChangeDutyCycle(min(abs(speed_percent), 100))
    else:
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        pwm_right.ChangeDutyCycle(min(abs(speed_percent), 100))

def drive(left_speed, right_speed):
    set_left(left_speed)
    set_right(right_speed)

# -------------------------
# Ultrasonic distance (HC-SR04)
# -------------------------
def read_distance_cm():
    # Trigger
    GPIO.output(TRIG, False)
    time.sleep(0.00005)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    pulse_start = time.time()
    timeout = pulse_start + 0.04

    while GPIO.input(ECHO) == 0 and time.time() < timeout:
        pulse_start = time.time()

    pulse_end = time.time()
    timeout = pulse_end + 0.04
    while GPIO.input(ECHO) == 1 and time.time() < timeout:
        pulse_end = time.time()

    duration = pulse_end - pulse_start
    if duration <= 0:
        return float('inf')
    distance = duration * 17150  # cm (speed of sound)
    return round(distance, 2)

# -------------------------
# GPS reading
# -------------------------
class GPSReader(threading.Thread):
    def __init__(self, port=GPS_PORT, baud=GPS_BAUD, timeout=1.0):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.serial = None
        self.latitude = None
        self.longitude = None
        self.satellites = None
        self.lock = threading.Lock()
        self.running = True
        try:
            self.serial = serial.Serial(self.port, self.baud, timeout=1)
            print(f"Opened GPS on {self.port} @ {self.baud}")
        except Exception as e:
            print("Failed to open GPS serial:", e)
            self.serial = None

    def run(self):
        if not self.serial:
            return
        while self.running:
            try:
                line = self.serial.readline().decode(errors='ignore').strip()
                if not line:
                    continue
                if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                    try:
                        msg = pynmea2.parse(line)
                        with self.lock:
                            if msg.lat and msg.lon:
                                self.latitude = msg.latitude
                                self.longitude = msg.longitude
                            self.satellites = getattr(msg, 'num_sats', None)
                    except Exception:
                        pass
                elif line.startswith('$GPRMC') or line.startswith('$GNRMC'):
                    try:
                        msg = pynmea2.parse(line)
                        with self.lock:
                            if msg.status == 'A' and msg.lat and msg.lon:
                                self.latitude = msg.latitude
                                self.longitude = msg.longitude
                    except Exception:
                        pass
            except Exception:
                time.sleep(0.1)

    def get_position(self):
        with self.lock:
            return (self.latitude, self.longitude, self.satellites)

    def stop(self):
        self.running = False
        if self.serial:
            try:
                self.serial.close()
            except:
                pass

# -------------------------
# Navigation math helpers
# -------------------------
def haversine_distance_m(lat1, lon1, lat2, lon2):
    # all in decimal degrees
    R = 6371000.0  # Earth radius in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2.0)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2.0)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def bearing_deg(lat1, lon1, lat2, lon2):
    # returns bearing from 1 -> 2 in degrees (0..360, 0 = north)
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    x = math.sin(dlambda) * math.cos(phi2)
    y = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(dlambda)
    initial = math.degrees(math.atan2(x, y))
    bearing = (initial + 360.0) % 360.0
    return bearing

def smallest_angle_diff(target, current):
    # returns -180..180 difference (target - current)
    a = (target - current + 180) % 360 - 180
    return a

# -------------------------
# MQTT client and callbacks
# -------------------------
class MQTTClient:
    def __init__(self, broker=MQTT_BROKER, port=MQTT_PORT):
        self.client = mqtt.Client("rpi_autowheelchair")
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(broker, port, 60)
        self.client.loop_start()
        self.latest_cmd = None

    def on_connect(self, client, userdata, flags, rc):
        print("MQTT connected, subscribing to commands")
        client.subscribe(CMD_TOPIC)

    def on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode()
        except:
            payload = ""
        payload = payload.strip().lower()
        print("MQTT message:", payload)
        self.latest_cmd = payload

    def publish(self, topic, payload):
        try:
            self.client.publish(topic, payload)
        except:
            pass

    def stop(self):
        try:
            self.client.loop_stop()
            self.client.disconnect()
        except:
            pass

# -------------------------
# Main autonomous controller
# -------------------------
def autonomous_loop():
    gps = GPSReader()
    gps.start()
    mqttc = MQTTClient()

    waypoints = list(WAYPOINTS)
    current_wp_idx = 0
    mode = "autonomous"  # other modes: 'manual', 'stop'
    speed = MAX_SPEED

    last_telemetry = time.time()

    try:
        while True:
            # Read sensors
            lat, lon, sats = gps.get_position()
            dist_cm = read_distance_cm()
            # handle obstacle
            if dist_cm < OBSTACLE_STOP_CM:
                stop_motors()
                mode = "stop"
                mqttc.publish(TELE_TOPIC, json.dumps({"alert": "obstacle", "distance_cm": dist_cm}))
                print("Obstacle detected (cm):", dist_cm)
                # wait until obstacle cleared
                time.sleep(0.5)
                # continue loop so manual override can occur
                continue

            # check for mqtt commands
            cmd = mqttc.latest_cmd
            if cmd:
                # process commands: stop, manual, forward/back/left/right, goto:lat,lon, set_speed:val
                if cmd == "stop":
                    mode = "stop"
                    stop_motors()
                    print("MQTT STOP command")
                elif cmd == "manual":
                    mode = "manual"
                    stop_motors()
                    print("MQTT switched to MANUAL")
                elif cmd == "autonomous":
                    mode = "autonomous"
                    print("MQTT switched to AUTONOMOUS")
                elif cmd.startswith("goto:"):
                    # goto:lat,lon
                    try:
                        payload = cmd.split(":",1)[1]
                        lat_s, lon_s = payload.split(",")
                        wp = (float(lat_s), float(lon_s))
                        waypoints = [wp]  # overwrite with single target
                        current_wp_idx = 0
                        mode = "autonomous"
                        print("New goto waypoint:", wp)
                    except Exception as e:
                        print("Bad goto payload:", e)
                elif cmd.startswith("set_speed:"):
                    try:
                        v = int(cmd.split(":",1)[1])
                        speed = max(0, min(100, v))
                        print("Speed set to", speed)
                    except:
                        pass
                elif mode == "manual":
                    # manual drive commands allowed when mode is manual
                    if cmd == "forward":
                        drive(speed, speed)
                    elif cmd in ("back", "backward"):
                        drive(-speed, -speed)
                    elif cmd == "left":
                        drive(-TURN_SPEED, TURN_SPEED)
                    elif cmd == "right":
                        drive(TURN_SPEED, -TURN_SPEED)
                    elif cmd == "stop":
                        stop_motors()
                # reset processed cmd
                mqttc.latest_cmd = None

            if mode == "autonomous" and lat and lon and waypoints:
                # get current waypoint
                if current_wp_idx >= len(waypoints):
                    # finished route
                    stop_motors()
                    mqttc.publish(TELE_TOPIC, json.dumps({"status": "route_complete"}))
                    print("All waypoints reached")
                    mode = "stop"
                else:
                    wp_lat, wp_lon = waypoints[current_wp_idx]
                    distance_m = haversine_distance_m(lat, lon, wp_lat, wp_lon)
                    desired_bearing = bearing_deg(lat, lon, wp_lat, wp_lon)
                    # Note: no yaw/compass on Pi: we don't know current heading.
                    # So we approximate by using last movement direction or implement a compass/IMU.
                    # For simplicity we assume wheels produce roughly forward motion; we'll implement a naive steering:
                    # basic strategy: if bearing relative to current heading unknown, perform a simple search:
                    # — Turn slightly right for short period if we think bearing is to right, else left.
                    # However, without current heading measurement this is limited. Best: add IMU/compass.
                    # Here we use a very naive approach: rotate in place toward waypoint using alternating turns
                    if distance_m <= REACHED_THRESHOLD_M:
                        print(f"Waypoint {current_wp_idx} reached ({distance_m:.1f} m).")
                        mqttc.publish(TELE_TOPIC, json.dumps({"event": "waypoint_reached", "index": current_wp_idx, "distance_m": distance_m}))
                        current_wp_idx += 1
                        stop_motors()
                        time.sleep(1.0)
                        continue

                    # Simple turn-then-go approach: spin a small time toward waypoint, then go forward
                    # This needs a compass/IMU for robust steering in real world.
                    # We'll implement: rotate right for small pulse, then drive forward; if distance decreases, keep going.
                    # measure distance before move:
                    before = distance_m
                    # try short rotation right then forward
                    drive(TURN_SPEED, -TURN_SPEED)  # rotate right in place
                    time.sleep(0.35)  # short rotation; tune for your robot geometry
                    drive(speed, speed)  # go forward
                    # allow to drive a bit
                    time.sleep(0.8)
                    stop_motors()
                    after = None
                    # read new gps (may be same for short moves), but best-effort:
                    lat2, lon2, _ = gps.get_position()
                    if lat2 and lon2:
                        after = haversine_distance_m(lat2, lon2, wp_lat, wp_lon)
                    # if after is worse than before, try turning left longer
                    if after and after > before:
                        # try turning left a bit and go forward
                        drive(-TURN_SPEED, TURN_SPEED)  # rotate left
                        time.sleep(0.5)
                        drive(speed, speed)
                        time.sleep(0.8)
                        stop_motors()
                    # continue loop

            # Telemetry publish periodically
            if time.time() - last_telemetry > TELEMETRY_INTERVAL:
                lat2, lon2, sats2 = gps.get_position()
                telemetry = {
                    "mode": mode,
                    "lat": lat2,
                    "lon": lon2,
                    "satellites": sats2,
                    "obstacle_cm": dist_cm,
                    "speed": speed,
                    "current_wp_idx": current_wp_idx,
                }
                mqttc.publish(TELE_TOPIC, json.dumps(telemetry))
                last_telemetry = time.time()

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("KeyboardInterrupt: stopping")
    finally:
        print("Cleaning up")
        gps.stop()
        mqttc.stop()
        stop_motors()
        pwm_left.stop()
        pwm_right.stop()
        GPIO.cleanup()

# -------------------------
# Entry point
# -------------------------
if __name__ == "__main__":
    print("Starting autonomous wheelchair controller")
    autonomous_loop()
