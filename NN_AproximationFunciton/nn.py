
import numpy as np

class NeuralNetwork:

    def __init__(self, num_inputs, num_hidden_layers, num_outputs):
        self.num_inputs = num_inputs
        self.num_hidden_layers = num_hidden_layers
        self.num_outputs = num_outputs
        self.weights_ih = np.random.rand(num_hidden_layers, num_inputs)
        self.weights_ho = np.random.rand(num_outputs, num_hidden_layers)

    def __toArrayNumpy(self, array):
        arr = np.array(array)
        return arr

    def __err(self, target, output):
        error = np.subtract(target,output)

        return error

    def __activation_function(self, sum):
        y_neuron = 1/(1+np.exp(-sum))
        return y_neuron

    def __dev_act_func(self, x):
        aux = self.__activation_function(x)
        result = aux*(1-aux)
        return result

    def feedforward(self, inputs_user):
        inputs = self.__toArrayNumpy(inputs_user)
        weighted_sum_out = self.weights_ih @ inputs
        h = self.__activation_function(weighted_sum_out)
        weighted_sum_hidden = self.weights_ho @ h
        output = self.__activation_function(weighted_sum_hidden)
        return output, h

    def training(self, train_set, number_iteractions, learning_rate):
        set = self.__toArrayNumpy(train_set)

        set_inputs = np.array(set[:,:-1])
        set_desired_outputs = set[:,-1]

        for epoca in range(number_iteractions):
            for inputs in set_inputs:
                y,h = self.feedforward(inputs)
                error = self.__err(set_desired_outputs,y)

                grad_w_ho = np.zeros([self.num_outputs, self.num_hidden_layers])
                grad_w_ih = np.zeros([self.num_hidden_layers, self.num_inputs])

                for k in range(self.num_hidden_layers):
                    for j in range(self.num_outputs):
                        grad_w_ho[j][k] = -self.__dev_act_func(self.weights_ho[j,:] @ h)*h[k]*error[j]

                for i in range(self.num_inputs):
                    for k in range(self.num_hidden_layers):
                        sum_error = 0
                        for m in range(self.num_outputs):
                            sum_error += self.weights_ho[m][k]*self.__activation_function(self.weights_ho[m,:] @ h)*error[m]

                        grad_w_ih[k][i] = -inputs[i]*self.__activation_function(self.weights_ih[k,:]@inputs)*sum_error

                self.weights_ho = np.subtract(self.weights_ho,-learning_rate*grad_w_ho)
                self.weights_ih = np.subtract(self.weights_ih,-learning_rate*grad_w_ih)

        return 0