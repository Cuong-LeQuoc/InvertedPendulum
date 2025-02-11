import serial


try:
    ser = serial.Serial(port="COM3", baudrate=576000, timeout=1)
    print("Connect Sucessful!")

    while open:
        data = ser.readline().decode("utf-8").strip()

        if data:
            print(f"{data}")

except serial.SerialException as e:
    print(f"Open fail: {e}")

except KeyboardInterrupt:
    print("Interrupt!")

finally:
    if ser.is_open and ('ser' in locals()):
        ser.close()
        print("Serial port Disconnect!")