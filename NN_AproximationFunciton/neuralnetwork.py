import numpy as np

class NeuralNetwork:

    def __init__(self, num_inputs, num_hidden_layers, num_outputs):
        self.num_inputs = num_inputs
        self.num_hidden_layers = num_hidden_layers
        self.num_outputs = num_outputs
        self.weights_ih = np.random.rand(num_hidden_layers, num_inputs)
        self.weights_ho = np.random.rand(num_outputs, num_hidden_layers)
        self.bias_ho = np.random.rand(num_outputs)
        self.bias_ih = np.random.rand(num_hidden_layers)

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
        y_neuron = 1/(1+np.exp(-sum))
        return y_neuron

    def transpose(self, matrix):
        matrix = self.__toArrayNumpy(matrix)
        if len(matrix.shape) == 1:
            matrix_trans = np.atleast_2d(matrix).T
        else:
            matrix_trans = matrix.T
        return matrix_trans

    def feedforward(self, inputs_user):
        inputs = self.__toArrayNumpy(inputs_user)
        weighted_sum_hidden = np.add(np.dot(self.weights_ih, self.transpose(inputs)), self.transpose(self.bias_ih))
        h = self.__activation_function(weighted_sum_hidden)

        print(h)

        weighted_sum_out = np.add(np.dot(self.weights_ho, h), self.transpose(self.bias_ho))
        output = self.__activation_function(weighted_sum_out)
        return output, h

    def training(self, train_set, number_iteractions, learning_rate):
        set = self.__toArrayNumpy(train_set)

        i= 0
        error_sum = 0
        error_sum_array = np.zeros(number_iteractions)
        for epoca in range(number_iteractions):
            error_sum_array[i] = error_sum
            i += 1
            error_sum = 0
            for set_element in set:
                inputs = np.array(set_element[:self.num_inputs])
                targets = np.array(set_element[-self.num_outputs:])

                y,h = self.feedforward(inputs)
                error = self.__err(targets, y)
                error_sum += error**2

                subtract_IY = np.subtract(np.identity(len(y)),np.diag(y))
                multiply_matrix_1 = np.dot(np.diag(y),subtract_IY)
                delta_ho = np.dot(multiply_matrix_1, error)

                subtract_IH = np.subtract(np.identity(len(h)),np.diag(self.transpose(h)[0,:]))
                multiply_matrix_2 = np.dot(np.diag(self.transpose(h)[0,:]), subtract_IH)
                delta_ih = np.dot(np.dot(multiply_matrix_2, self.transpose(self.weights_ho)), delta_ho)

                weights_ho_trans = self.transpose(self.weights_ho)
                weights_ih_trans = self.transpose(self.weights_ih)

                weights_ho_trans = np.add(weights_ho_trans, learning_rate*(np.dot(h,self.transpose(delta_ho))))
                weights_ih_trans = np.add(weights_ih_trans, learning_rate*(np.dot(self.transpose(inputs), np.atleast_2d(delta_ih))))

                self.weights_ih = self.transpose(weights_ih_trans)
                self.weights_ho = self.transpose(weights_ho_trans)

                self.bias_ho = np.add(self.bias_ho,np.dot(learning_rate,delta_ho))
                self.bias_ih = np.add(self.bias_ih, np.dot(learning_rate, delta_ih))

        #print("Matriz de pesos da IH")
        #print(self.weights_ih)
        #print()
        #print("Matriz de pesos para HO")
        #print(self.weights_ho)
        #print()
        #print('Bias original H')
        #print(self.bias_ih)
        #print()
        #print("Bias original O")
        #print(self.bias_ho)
        #print()
            #print(error_sum)
            #print()

        return error_sum_array
