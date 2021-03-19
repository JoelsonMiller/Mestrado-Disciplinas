import multilayerperceptron as neural_NN
import numpy as np
import matplotlib.pyplot as plt

inputs = np.arange(-1,1.1,0.1)
#targets = 5*inputs
targets = [-0.96,-0.577, -0.073, 0.377, 0.641, 0.660, 0.461, 0.134, -0.201,
           -0.434, -0.5, -0.393, -0.165, 0.099, 0.307, 0.396, 0.345, 0.182,
           -0.031, -0.219, -0.32]

trainning_set = np.array([inputs,targets]).T

#trainning_set = [[1,1,0],[0,0,0],[0,1,1],[1,0,1]]
neural = neural_NN.NeuralNetwork(1,10,1)

iteractions = 20000

print("I am here!")
error = neural.training(trainning_set, iteractions, 0.01)
print("ok! ")

#print("Erro final da rede: {}".format(error))


#Teste do XOR
#guess, _ = neural.feedforward([1,1])
#print(guess)

#guess, _ = neural.feedforward([0,0])
#print(guess)

#guess, _ = neural.feedforward([1,0])
#print(guess)

#guess, _ = neural.feedforward([0,1])
#print(guess)

y = np.zeros(inputs.shape[0])
i =0

for x in inputs:
    y[i], _ = neural.feedforward([x])
    i+=1

#print(error)
plt.plot(np.arange(1,iteractions, 1), error[1:])
plt.show()
#print(y)

plt.plot(inputs, targets,'o')
plt.plot(inputs, y)
plt.show()
