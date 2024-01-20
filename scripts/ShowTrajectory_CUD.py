import matplotlib.pyplot as plt
import numpy as np

def read_trajectory_data(file_path):
    timestamps = []
    displacements = []
    quaternions = []

    with open(file_path, 'r') as file:
        for line in file:
            data = line.split()
            timestamp = float(data[0])
            displacement = np.array(list(map(float, data[1:4])))
            quaternion = np.array(list(map(float, data[4:8])))

            timestamps.append(timestamp)
            displacements.append(displacement)
            quaternions.append(quaternion)

    return timestamps, displacements, quaternions

def plot_trajectory(displacements):
    x = np.array([arr[0] for arr in displacements])
    y = np.array([arr[2] for arr in displacements])
    plt.axis('equal')
    plt.plot(x,y)
    plt.title('X-Y_Trajectory Plot')
    plt.xlabel('X')
    plt.ylabel('Y')
    # plt.legend(['X', 'Y', 'Z'])
    plt.show()

if __name__ == "__main__":
    file_path = 'KeyFrameTrajectory.txt'
    timestamps, displacements, quaternions = read_trajectory_data(file_path)
    # print(timestamps)
    plot_trajectory(displacements)
