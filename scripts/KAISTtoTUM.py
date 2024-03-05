import pandas as pd
import numpy as np
from tqdm import tqdm
from scipy.spatial.transform import Rotation

# 指定CSV文件路径
csv_file_path = '/media/liu/KINGSTON/Complex_Urban_LiDAR_Data_Set/urban26-dongtan/global_pose.csv'

# 读取CSV文件
df = pd.read_csv(csv_file_path, header=None)

# 重命名列
df.columns = ['timestamp', 'm11', 'm12', 'm13', 'tx', 'm21', 'm22', 'm23', 'ty', 'm31', 'm32', 'm33', 'tz']

# 计算四元数
def calculate_quaternion(row):
    rotation_matrix = np.array([[row['m11'], row['m12'], row['m13']],
                                [row['m21'], row['m22'], row['m23']],
                                [row['m31'], row['m32'], row['m33']]])

    quaternion = Rotation.from_matrix(rotation_matrix).as_quat()
    return f"{quaternion[0]}, {quaternion[1]}, {quaternion[2]}, {quaternion[3]}"

# 创建新的DataFrame，包含时间戳、平移向量和四元数
tqdm.pandas(desc='Calculating quaternion')
new_df = pd.DataFrame({
    'timestamp': df['timestamp'] / 10**9,
    'tx': df['tx'],
    'ty': df['ty'],
    'tz': df['tz'],
    'quaternion': df.progress_apply(calculate_quaternion, axis=1)
})
new_df[['qw', 'qx', 'qy','qz']] = new_df['quaternion'].str.split(',', expand=True)
new_df = new_df.drop('quaternion', axis=1)

# 将新的DataFrame写入为TXT文件
txt_file_path = '/home/liu/SLAM_WORK/evo/26/global_poseTUM.txt'
new_df.to_csv(txt_file_path, sep=' ', index=False, header=False)
print('Data has been writen to:' + txt_file_path)
