import serial
import serial.tools.list_ports
import datetime
import time
import re

BAUD_RATE = 115200

def parse_line(line):
    """
    Automatically recognises and parses both formats:
    1) BEACON_01,-45,12345 (CSV)
    2) $BEACON,ID=1,RSSI=-65 (NMEA-like)
    """
    line = line.strip()
    if not line:
        return None

    #BEACON_01,-45,12345
    match_csv = re.match(r'^([A-Za-z0-9_]+),\s*(-?\d+),\s*(\d+)$', line)
    if match_csv:
        beacon_id, rssi, device_time_ms = match_csv.groups()
        return {
            "beacon_id": beacon_id,
            "rssi": int(rssi),
            "timestamp_ms": int(device_time_ms),
            "format": "CSV"
        }

    #$BEACON,ID=1,RSSI=-65
    match_keyval = re.match(r'^\$BEACON,ID=([A-Za-z0-9_]+),RSSI=(-?\d+)', line)
    if match_keyval:
        beacon_id, rssi = match_keyval.groups()
        return {
            "beacon_id": f"BEACON_{beacon_id.zfill(2)}",
            "rssi": int(rssi),
            "timestamp_ms": None,
            "format": "NMEA-like"
        }

    return None

def process_and_display(raw_line):
    parsed = parse_line(raw_line)
    laptop_time = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

    if parsed:
        msg_time = f"{parsed['timestamp_ms']} ms" if parsed['timestamp_ms'] else "N/A"
        print(f"[{laptop_time}] | ID: {parsed['beacon_id']:<10} | RSSI: {parsed['rssi']:>3} dBm | Time: {msg_time:<8} | ({parsed['format']})")
    else:
        print(f"[{laptop_time}] RAW: {raw_line.strip()}")

def run_serial_receiver(port):
    """An attempt to read real data from the circuit board"""
    print(f"Підключення до плати на {port} ({BAUD_RATE} baud)")
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        while True:
            if ser.in_waiting > 0:
                raw_line = ser.readline().decode('utf-8', errors='ignore')
                process_and_display(raw_line)
    except KeyboardInterrupt:
        print("\nЗупинено користувачем")
        return True
    except Exception as e:
        print(f"\nПомилка читання з COM-порту: {e}")
        return False

def run_demo_simulation():
    """Emulation of test data if the board is not present"""
    print("Запуск у демо-режимі")
    simulated_messages = [
        "BEACON_01,-45,12345",
        "$BEACON,ID=1,RSSI=-65\n",
        "BEACON_02,-52,12845",
        "$BEACON,ID=2,RSSI=-58\n",
        "BEACON_03,-68,13345",
    ]
    
    try:
        idx = 0
        while True:
            line = simulated_messages[idx % len(simulated_messages)]
            process_and_display(line)
            idx += 1
            time.sleep(0.5) # 2 Гц
    except KeyboardInterrupt:
        print("\nДемо зупинено")

if __name__ == "__main__":
    ports = list(serial.tools.list_ports.comports())
    
    if ports:
        selected_port = ports[0].device
        print(f"Знайдено фізичний пристрій: {selected_port} ({ports[0].description})")
        
        #Пробуємо запустити реальний режим
        success = run_serial_receiver(selected_port)
        
        #Якщо не вдалося - переходимо на демо
        if not success:
            print("\nАвтоматичний перехід у демо-режим...\n")
            run_demo_simulation()
    else:
        print("Фізичну плату (COM-порт) не знайдено")
        print("Автоматичний запуск у демо-режим...\n")
        run_demo_simulation()
