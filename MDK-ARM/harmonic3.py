import numpy as np

N = 1200
A = 1050
theta = np.linspace(0, 2 * np.pi, N, endpoint=False)
harmonic3 = (A * np.sin(3 * theta)).astype(int)

# 格式化输出为每行16个元素
with open("harmonic3_table.txt", "w") as f:
    for i in range(0, N, 16):
        line = ", ".join(map(str, harmonic3[i:i+16]))
        f.write(line + ",\n")
