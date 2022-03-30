import serial
import time

# -- SETUP --
arduino_port = 'COM4'
baud = 115200
headers = "Time,Voltage[V],Current[mA],Temp-1[C],Temp-2[C],Temp-3[C],Fan_Speed[RPM],PWM_Duty[%]"


# -- main code --
localtime = time.ctime(time.time())
Test_title = input('Enter title for this test\n')
arduino = serial.Serial(arduino_port, baud, timeout=1)
print('Connected to Arduino on port:' + arduino_port)

log_info = str('Test: ' + Test_title + '\nDate: ' + localtime)
filename = 'Datasets/' + Test_title + '.csv'
data_file = open(filename, 'x')
data_file.write(log_info + '\n' + headers)
print('created file')
data_file.close()
print(log_info)
start = input('Press 1 to begin measurements and ctrl+C to stop\n')

try:
    while start:
        arduino.write(b'b')
        Sample_line = str(arduino.readline())
        Sample_line = Sample_line[2:][:-5]
        if len(Sample_line) > 0:
            timestamp = time.strftime("%H:%M:%S", time.localtime())
            print(timestamp + ',' + Sample_line)
            with open(filename, 'a') as file:
                file.write('\n' + timestamp + ',' + Sample_line)

except:
    arduino.write(b't')
