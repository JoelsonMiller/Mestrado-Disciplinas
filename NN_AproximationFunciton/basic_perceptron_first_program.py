
import matplotlib.pyplot as plt
import numpy as np
import math

#sign function
def sinal(num):
    if num >= 0:
        return 1
    else:
        return 0

#Training Dataset

x = np.array([[2.78, 1.46, 3.39, 1.38, 3.06, 7.06, 5.33, 6.92, 8.67, 7.67]
    ,[2.55, 2.36, 4.4, 1.85, 3.00, 2.75, 2.08, 1.77, -0.24, 3.51]])

t = [0, 0, 0, 0, 0, 1, 1, 1, 1, 1]

r_learning = 0.6

epoca = 0

w = [0.05, 1.0, 0.5]

flag = 1

while(flag != 0):
    flag = 0
    for i in range(len(t)):
        a = x[0:,i]
        a = np.append(-1,a)
        sum = np.dot(w,a)
        #print(sum)
        y = sinal(sum)
        e = t[i]-y
        if e != 0.0:
            w = np.add(w,r_learning*e*a)
            #print(w)
            flag = 1;
    epoca += 1

print("Os pesos na saída foram: ")
print(w)

print("A quantidade de epocas utilizadas para o treinamento foram: {}".format(epoca))

xpoints1 = x[0,:4]
ypoints1 = x[1,:4]
plt.plot(xpoints1,ypoints1, 'ro')


xpoints2 = x[0,5:]
ypoints2 = x[1,5:]
plt.plot(xpoints2,ypoints2, 'bo')

#Reta que divide os dois grupos

x_line = np.linspace(-1,10,100)
y_line = -(w[1]/w[2])*x_line + w[0]/w[2]
plt.plot(x_line, y_line, 'k-')
plt.show()