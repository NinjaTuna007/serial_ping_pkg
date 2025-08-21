# TODO: rename file and add sensible comments

import serial
import time
import csv

print("starting")
# time.sleep(3)
#Align the modems to ideally face each other

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

if ser.is_open:
    print("Serial port is open")

command = "$P001"

with open('./data/response_passive.csv', mode='a', newline='') as file:
    writer = csv.writer(file)

    if file.tell() == 0:
        writer.writerow(["Ping Distance", "Time Delta"])

    for _ in range(30):

        print(f"Iteration {_ + 1}")

        ser.write(command.encode())
        time.sleep(1)
        response = ser.read(ser.in_waiting)
        print(f"Ping Response: {response.decode('utf-8')}")

        # split the response at T
        response = response.decode('utf-8').split("T")[1]

        dist = 0.00003125 * 335 * float(response)

        # send a message to the other modem saying: "DO IT"
        
        scream = "$B04DOIT"

        ser.write(scream.encode())
        print(f"Sent: {scream}")

        # Wait for the complete message
        # message = b""
        while ser.in_waiting < 24:
            # print(f"Waiting for message: {ser.in_waiting}")
            pass

        time_received = time.time()

        # read the complete message
        message = ser.read(ser.in_waiting)
    
        
        try:
            message = message.decode('utf-8')
            print(f"Message: {message}")

            message = message.split("#")[1]
            message = message[6:]
            print(f"Message from other modem: {message}")

            time_delta = time_received - float(message)
            writer.writerow([dist, time_delta])


            print(f"Distance: {dist}, Time Delta: {time_delta}")

            # print(f"Raw Message: {message}")
            # print(f"Time received: {time_received}")

        except (IndexError, ValueError) as e:
            print(f"Error parsing message: {e}")
            writer.writerow([dist, "Error"])

        time.sleep(1)  # Adjust sleep time as needed

ser.close()