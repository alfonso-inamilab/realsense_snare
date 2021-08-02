# This sample get the acceleromter data using Arduino/Serial communication

# Runs in a different thred to optimize the communication
import threading
import serial 
import time

class accThread():
    
    # Inits serial communication and init class variables
    def __init__(self, port='COM4', baudrate=115200, timeout=.1 ):
        self.ax = 0.0; self.ay = 0.0; self.az = 0.0
        self.arduino = serial.Serial(port=port, baudrate=baudrate, timeout=timeout )
        self.my_thread=threading.Thread(target=self.run)
        self.stopFlag = False
        pass

    # Thread main loop
    def run(self):
        while (self.stopFlag == False):
            data = self.arduino.readline()
            print (data)
            time.sleep(0.1)

    # start the thread
    def start(self):
        self.my_thread.start()

    # exits main loop using bool flag
    def stop(self):
        self.stopFlag = True
        print ("Stopping serial thread...")


if __name__ == '__main__':
    acc = accThread('COM10', 115200, 0.1)
    acc.start()

    # Press Ctrl+C to finish
    try:
        while True:  
            pass  
    except KeyboardInterrupt:
        acc.stop()
        time.sleep(1)
        print ("Exit")
        
