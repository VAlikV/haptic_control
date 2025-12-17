import matplotlib.pyplot as plt
import numpy as np

def T(d, thetta, alpha, a):
        return np.array([[np.cos(thetta), -np.sin(thetta)*np.cos(alpha), np.sin(thetta)*np.sin(alpha), a*np.cos(thetta)],
                  [np.sin(thetta), np.cos(thetta)*np.cos(alpha), -np.cos(thetta)*np.sin(alpha), a*np.sin(thetta)], 
                  [0, np.sin(alpha), np.cos(alpha), d], 
                  [0, 0, 0, 1]])

def FK(d, thetta, alpha, a):
        temp_t = np.eye(4)
        N = len(d)

        joint = np.array([[0, 0, 0]])

        for i in range(0,N):
            temp_t = temp_t @ T(d[i], thetta[i], alpha[i], a[i])
            coordinate = np.array([temp_t[0][3], temp_t[1][3], temp_t[2][3]])
            joint = np.vstack([joint, coordinate])

        endefector_pos = [[joint[N]], [joint[N]], [joint[N]]]

        return temp_t, joint, endefector_pos


q = np.array([0, 15.4062, -21.246, 0.0793419, 45.3143, 2.78941])
m = np.array([-1, 1, 1, -1, -1, 1])
q = m * q * np.pi/180

thetta = np.array([np.pi, 0, 0, 0, -np.pi/2, 0],dtype=float)
alpha = np.array([np.pi/2, 0, np.pi/2, np.pi/2, np.pi/2, 0],dtype=float)
d = np.array([2, 0, 0, 2, 0, 1],dtype=float)
a = np.array([0, 2, 0, 0, 0, 0],dtype=float)

rt, data, _ = FK(d,q+thetta,alpha,a)
# print(data)

# data = np.insert(data,0,[0,0,0],axis=1)

# data = np.reshape(data,(3,7))
print(data.T)
print(rt[0:3,0:3])
print("------------------------------------------")

data = data.T

fig = plt.figure()

ax = plt.axes(projection='3d')
plt.axis('equal')

ax.set_xlim3d([-3, 3])
ax.set_ylim3d([-3, 3])
ax.set_zlim3d([0, 3.3])
ax.set_xlabel("X")
ax.set_ylabel("Y")

ax.plot3D(data[0], data[1], data[2], 'red')

ax.scatter3D(data[0], data[1], data[2])


plt.show()