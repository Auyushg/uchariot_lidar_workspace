import serial, struct, csv, math, time
from collections import defaultdict

# === Constants ===
PORT        = "/dev/ttyUSB0"
BAUD        = 230400
TIMEOUT     = 1.0
HEADER      = b'\x54\x2C\xF3\x0D'
PACKET_SIZE = 64
NUM_POINTS  = 28

MAX_PACKETS  = 500
MAX_DURATION = 15.0  # seconds

def parse_ld500_packet(pkt):
    start = struct.unpack_from("<H", pkt, 4)[0] / 100.0
    end   = struct.unpack_from("<H", pkt, 6)[0] / 100.0
    diff  = (end - start + 360.0) % 360.0
    step  = diff / NUM_POINTS
    pts = []
    for i in range(NUM_POINTS):
        angle = (start + i * step) % 360.0
        dist  = struct.unpack_from("<H", pkt, 8 + 2*i)[0] / 1000.0
        pts.append((angle, dist))
    return pts

def main():
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    ser.reset_input_buffer()
    buffer = bytearray()
    bins = defaultdict(list)
    packet_count = 0
    start_time = time.time()

    # read until we’ve seen enough packets or timed out
    try:
        while packet_count < MAX_PACKETS and (time.time() - start_time) < MAX_DURATION:
            chunk = ser.read(ser.in_waiting or 1)
            if not chunk:
                continue
            buffer += chunk

            while True:
                idx = buffer.find(HEADER)
                if idx < 0 or len(buffer) < idx + PACKET_SIZE:
                    break
                pkt = buffer[idx:idx + PACKET_SIZE]
                buffer = buffer[idx + PACKET_SIZE:]
                packet_count += 1

                for angle, dist in parse_ld500_packet(pkt):
                    if dist > 0.0:  # ignore zero returns
                        deg = int(math.floor(angle)) % 360
                        bins[deg].append(dist)
    finally:
        ser.close()

    # build 0–359 list by taking the MIN in each bin
    distances = []
    for deg in range(360):
        readings = bins.get(deg, [])
        if readings:
            distances.append(min(readings))
        else:
            distances.append(float('nan'))

    # optional: interpolate NaNs here if you want
    # write out CSV
    with open('full_scan_360.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['angle_deg','distance_m'])
        for deg, d in enumerate(distances):
            w.writerow([deg, d])

    print(f"Wrote {len(distances)} rows to full_scan_360.csv (min‑distance binning)")

if __name__ == "__main__":
    main()