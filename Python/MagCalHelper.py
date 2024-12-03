import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

ser = serial.Serial(port = 'COM7', baudrate = 115200, timeout = 1)
MagData = []

MagX = []
MagY = []
MagZ = []

try:
    with open("F:\\Semester_1\\00_Seminar-Medit\\Python" + "\\MagData.txt", 'w') as file:
        while True:
            line = list(ser.readline().decode('utf-8').strip().split(','))
            if(line[0] == ""):
                continue
            else:
                MagData = list(map(int, line))

                MagX.append(MagData[0])
                MagY.append(MagData[1])
                MagZ.append(MagData[2])

                print(MagData)

                for item in MagData:
                    file.write(f"{item}\t")
                file.write(f"\n")

except KeyboardInterrupt:
    file.close()
    print('Exiting!')

finally:
    fig = plt.figure()
    ax = fig.add_subplot(111, projection = '3d')
    ax.scatter(MagX, MagY, MagZ, color = 'r')
    plt.show()
    ser.close()

