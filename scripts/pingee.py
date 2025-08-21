import serial
import time

def send_and_get_response(send_cmd, get_response, cmd = "None"):
    if send_cmd == 1:
        command = cmd
        print(f"Sending command : {command}")
        ser.write(command.encode())

    if get_response == 1:
        time.sleep(0.5)
        response = ser.read(ser.in_waiting)
        resp = response.decode('utf-8')
        print(f"Response: {resp}")
        return resp
    else:
        return 1

num_iter = 0
max_iter = 30


print("starting")
time.sleep(3)

#Align the modems to ideally face each other
ser = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)
if ser.is_open:
    print("Serial port is open")

#Enable the timer
resp1 = send_and_get_response(1,1,"$XTE")
time.sleep(0.25)

while True:

    doit_data = send_and_get_response(0,1)
    print(doit_data)
    print(len(doit_data))

    if len(doit_data) == 33:
        print("inside if condition")
        sys_time = int(doit_data[17:])
        tot = str(1500000 + sys_time)
        length_tot = str(len(tot)).zfill(2)
        timed_trans_command = "$B" + length_tot + tot + "T" + "0"*(14 - len(tot)) + tot
        print(timed_trans_command)
        send_and_get_response(1,1,timed_trans_command)
        
        
    
    time.sleep(0.5)
    
ser.close()
