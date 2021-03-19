import numpy as np
from math import exp as e

class NeuralNetwork:

    def __init__(self, num_inputs, num_hidden_layers, num_outputs):
        self.num_inputs = num_inputs
        self.num_hidden_layers = num_hidden_layers
        self.num_outputs = num_outputs
        self.weights_ih = np.random.rand(num_hidden_layers, num_inputs)
        self.weights_ho = np.random.rand(num_outputs, num_hidden_layers)
        self.teste = self.weights_ho
        self.bias_ho = np.zeros(num_outputs)
        self.teste_bias = self.bias_ho
        self.bias_ih = np.zeros(num_hidden_layers)
        self.momentum = 0.95

        #print('Matriz de pesos original IH')
        #print(self.weights_ih)
        #print()
        #print("Matriz de pesos origina HO")
        #print(self.weights_ho)
        #print()
        #print('Bias original H')
        #print(self.bias_ih)
        #print()
        #print("Bias original O")
        #print(self.bias_ho)
        #print()

    def __toArrayNumpy(self, array):
        arr = np.array(array)
        return arr

    def __err(self, target, output):
        error = np.subtract(target,output)

        return error

    def __activation_function(self, sum):
        #y_neuron = 1/(1+np.exp(-sum))-1
        y_neuron = (e(sum)-e(-sum))/(e(sum)+e(-sum))
        return y_neuron

#    def __activation_function_linear(self, sum):
#        sum = 2*sum
#        if sum <= -1:
#            y_neuron = -1
#        elif sum >= 1:
#            y_neuron = 1
#        else:
#            return sum
#        return y_neuron

   # def __adap_learning_r(self, i):
   #     lr = 0.01*(1/np.linalg.norm(i))
        #print(lr)
    #    return lr

    def transpose(self, matrix):
        matrix = self.__toArrayNumpy(matrix)
        if len(matrix.shape) == 1:
            matrix_trans = np.atleast_2d(matrix).T
        else:
            matrix_trans = matrix.T
        return matrix_trans


    def feedforward(self, inputs_user):
        inputs = self.__toArrayNumpy(inputs_user)
        h = np.zeros(self.num_hidden_layers)

        for l in range(self.num_hidden_layers):
            sum = 0
            for j in range(self.num_inputs):
                sum += self.weights_ih[l][j]*inputs[j]
            h[l] = self.__activation_function(sum + self.bias_ih[l])


        output = np.zeros(self.num_outputs)
        for i in range(self.num_outputs):
            sum = 0
            for l in range(self.num_hidden_layers):
                sum += self.teste[i][l]*h[l]
            output[i] = np.add(sum,self.teste_bias[i])
            #output[i] = sum
        return output, h

    def training(self, train_set, number_iteractions, learning_rate):
        set = self.__toArrayNumpy(train_set)
        x= 0
        error_sum = 0
        error_sum_array = np.zeros(number_iteractions)

        for epoca in range(number_iteractions):

            error_sum_array[x] = error_sum
            x += 1
            error_sum = 0

            for set_element in set:
                inputs = np.array(set_element[:self.num_inputs])
                targets = np.array(set_element[-self.num_outputs:])

                y,h = self.feedforward(inputs)
                #error = self.__err(targets, y)

                delta_o = np.zeros(self.num_outputs)
                for i in range(self.num_outputs):
                    error = targets[i]-y[i]
                    #delta_o[i] = y[i]*(1-y[i])*error
                    delta_o[i] = (1 - np.tanh(y[i])**2) * error

                error_sum += error ** 2

                delta_l = np.zeros(self.num_hidden_layers)
                for l in range(self.num_hidden_layers):
                    sum = 0
                    for i in range(self.num_outputs):
                        sum += self.weights_ho[i][l]*delta_o[i]
                    #delta_l[l] = h[l]*(1-h[l])*sum
                    delta_l[l] = (1 - np.tanh(h[l])**2) * sum


                #for l in range(self.num_hidden_layers):
                #    for i in range(self.num_outputs):
                #        self.weights_ho[i][l] = self.weights_ho[i][l] + learning_rate*h[l]*delta_o[i]

                for j in range(self.num_inputs):
                    for l in range(self.num_hidden_layers):
                        self.weights_ih[l][j] = self.weights_ih[l][j] + learning_rate*inputs[j]*delta_l[l]

                for i in range(self.num_outputs):
                    self.bias_ho[i] = self.bias_ho[i]+learning_rate*delta_o[i]

                for l in range(self.num_hidden_layers):
                    self.bias_ih[l] = self.bias_ih[l]+learning_rate*delta_l[l]


        return error_sum_array
