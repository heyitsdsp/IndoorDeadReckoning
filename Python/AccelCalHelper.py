import serial
import keyboard
import statistics

AccX = []
AccY = []
AccZ = []

StoreList = []

def storevalues(file):

    AccX.clear()
    AccY.clear()
    AccZ.clear()
    StoreList.clear()

    try:            
        for i in range(0, 25):
            line = list(ser.readline().decode('utf-8').strip().split(','))
            if(line[0] == "" or line[1] == "" or line[2] == ""):
                break
            else:
                try:
                    AccX.append((float(line[0])))
                except:
                    break
                try:
                    AccY.append((float(line[1])))
                except:
                    break
                try:
                    AccZ.append((float(line[2])))
                except:
                    break
        
        StoreList.append(statistics.fmean(AccX))
        StoreList.append(statistics.fmean(AccY))
        StoreList.append(statistics.fmean(AccZ))

        print(StoreList)

        for item in StoreList:
            file.write(f"{item}\t")
        file.write(f"\n")

    except KeyboardInterrupt:
        file.close()
        print('Exiting!')


ser = serial.Serial(port = 'COM7', baudrate = 115200, timeout = 1)

with open("F:\\Semester_1\\00_Seminar-Medit\\Python" + "\\AccData.txt", 'w') as file:
    while True:
        try: 
            if keyboard.is_pressed('m'):
                storevalues(file)

        except KeyboardInterrupt:
            file.close()
            print("Exiting!")
            break



