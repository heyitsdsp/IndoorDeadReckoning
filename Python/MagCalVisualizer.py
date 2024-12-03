import matplotlib.pyplot as plt

MagRawX = []
MagRawY = []
MagRawZ = []

MagCalX = []
MagCalY = []
MagCalZ = []

with open("F:\\Semester_1\\00_Seminar-Medit\\Python" + "\\MagData.txt", 'r') as file:
    lines = file.readlines()

for line in lines:
    line = list(line.strip().split('\t'))
    MagRawX.append(int(line[0]))
    MagRawY.append(int(line[1]))
    MagRawZ.append(int(line[2]))
    
hardiron_biases = [-52.741694, -428.279362, -17.080402]
softiron_matrix = [[0.055398, -0.001084, -0.000711],
                   [-0.001084, 0.062740, -0.000171],
                   [-0.000711, -0.000171, 0.090522]]

for i in range(0, len(MagRawX)):
    x = MagRawX[i] - hardiron_biases[0]
    y = MagRawY[i] - hardiron_biases[1]
    z = MagRawZ[i] - hardiron_biases[2]

    x = x * softiron_matrix[0][0] + y * softiron_matrix[0][1] + z * softiron_matrix[0][2]
    y = x * softiron_matrix[1][0] + y * softiron_matrix[1][1] + z * softiron_matrix[1][2]
    z = x * softiron_matrix[2][0] + y * softiron_matrix[2][1] + z * softiron_matrix[2][2]

    MagCalX.append(x)
    MagCalY.append(y)
    MagCalZ.append(z)

fig = plt.figure()
ax = fig.add_subplot(211, projection = '3d')
ax.scatter(MagRawX, MagRawY, MagRawZ, color = 'r')
ay = fig.add_subplot(212, projection = '3d')
ay.scatter(MagCalX, MagCalY, MagCalZ, color = 'b')
plt.show()


file.close()
    