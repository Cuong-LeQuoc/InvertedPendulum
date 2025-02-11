import serial
import csv
import time

try:
    ser = serial.Serial(port="COM3", baudrate=576000, timeout=1)
    print("Connect Sucessful!")

    csv_filename = input("Fill file's name measurement data: ")


    with open(csv_filename + ".csv", mode='a', newline='') as file:
        writer = csv.writer(file)

        # Ghi tiêu đề nếu file rỗng
        if file.tell() == 0:
            writer.writerow(["Angle", "Velocity", "Raw Velocity"])

        startTime = time.time()

        while True:
            data = ser.readline().decode("utf-8").strip()
            state = data[1:].split()

            if (data[0] != 'S') or (len(state) != 3) or not data:
                continue

            angle = float(state[0])
            velocity = float(state[1])
            rawVel = float(state[2])
            dt = time.time() - startTime

            print(f"Motor's Angle: {angle}")
            print(f"Motor's Velocity: {velocity}")
            print(f"Motor's Raw Velocity: {rawVel}")
            print(f"Time: {dt}")

            writer.writerow([dt, angle, velocity, rawVel])
        

except serial.SerialException as e:
    print(f"Open fail: {e}")

except KeyboardInterrupt:
    print("Interrupt!")

finally:
    if ser.is_open and ('ser' in locals()):
        ser.close()
        print("Serial port Disconnect!")