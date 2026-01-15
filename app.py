from flask import Flask, render_template, jsonify
import serial
import threading
import time

app = Flask(__name__)

SERIAL_PORT = 'COM5' # Sesuaikan COM Anda
BAUD_RATE = 115200

sensor_data = {
    'pitch': 0.0, 'roll': 0.0, 'yaw': 0.0,
    'ax': 0.0, 'ay': 0.0, 'az': 0.0
}

def read_serial():
    global sensor_data
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        print(f"Koneksi OK: {SERIAL_PORT}")
        
        while True:
            if ser.in_waiting > 0:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    parts = line.split(',')
                    # Sekarang kita terima 6 data
                    if len(parts) == 6:
                        sensor_data['pitch'] = float(parts[0])
                        sensor_data['roll'] = float(parts[1])
                        sensor_data['yaw'] = float(parts[2])
                        sensor_data['ax'] = float(parts[3])
                        sensor_data['ay'] = float(parts[4])
                        sensor_data['az'] = float(parts[5])
                except ValueError:
                    pass
    except Exception as e:
        print(f"Error: {e}")

thread = threading.Thread(target=read_serial)
thread.daemon = True
thread.start()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/data')
def get_data():
    return jsonify(sensor_data)

if __name__ == '__main__':
    app.run(debug=True, use_reloader=False, host='0.0.0.0', port=5000)