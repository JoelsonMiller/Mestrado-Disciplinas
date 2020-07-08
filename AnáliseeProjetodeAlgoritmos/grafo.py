import numpy as np #Biblioteca matemática

from networkx import Graph, DiGraph, draw_networkx, spring_layout #Biblioteca de grafo
from matplotlib.pyplot import show #Biblioteca de plot
from colorama import init, deinit #Biblioteca para estética

def processFileData(file):
    firstline = file.readline().rstrip().upper()  # Ler a primeira linha do arquivo
    if firstline == 'D':
        tipoGrafo = True
    else:
        tipoGrafo = False
    arestas = []
    vertices = []

    # Extração do conteúdo do arquivo

    for l in file:

        rawData = l.split(",")
        for i in range(2):
            rawData[i] = rawData[i].rstrip().lstrip()
        if len(vertices) != 0:
            for i in range(2):
                if not rawData[i] in vertices:
                    vertices.append(rawData[i])
        else:
            vertices.append(rawData[0])
            if not rawData[1] in vertices:
                vertices.append(rawData[1])
        arestas.append(rawData)
    return vertices, arestas, tipoGrafo

class Grafo:
    def __init__(self, vertices, arestas, tipoGrafo):
        self.vertices = vertices
        self.arestas = arestas
        self.tipoGrafo = tipoGrafo  # True para Digrafo; False para Grafo não dirigido
        self.grafoMA = np.zeros((len(vertices), len(vertices)))

    def criar_grafo(self):
        if self.tipoGrafo:
            for aresta in self.arestas:
                i, j = self.vertices.index(aresta[0]), self.vertices.index(aresta[1])
                self.grafoMA[i, j] = 1
        else:
            for aresta in self.arestas:
                i, j = self.vertices.index(aresta[0]), self.vertices.index(aresta[1])
                self.grafoMA[i, j] = 1
                self.grafoMA[j, i] = 1
        return self.grafoMA

    def testa_aresta(self, v1, v2):
        i, j = self.vertices.index(v1), self.vertices.index(v2)
        if self.grafoMA[i, j]:
            return True
        else:
            return False

    def grau_vertice(self, v):
        index = self.vertices.index(v)
        if self.tipoGrafo:
            linha_v = self.grafoMA[index, :]
            coluna_v = self.grafoMA[:,index]
            grau_saida = np.sum(linha_v)
            grau_entrada = np.sum(coluna_v)
            return grau_entrada, grau_saida                #Grau de entrada, Grau de Saída
        else:
            linha_v = self.grafoMA[index,:]
            grau_v = np.sum(linha_v)
            return grau_v, -1

    def adj_vertice(self, v):
        vertices_adj = []
        vertices_suc = []
        vertices_ant = []
        i = self.vertices.index(v)
        if self.tipoGrafo:
            linha_v =  self.grafoMA[i, :]
            coluna_v = self.grafoMA[:,i]
            index_suc = np.nonzero(linha_v)
            index_ant = np.nonzero(coluna_v)
            for x in index_suc[0]:
                vertices_suc.append(self.vertices[x])
            for y in index_ant[0]:
                vertices_ant.append(self.vertices[y])
            return vertices_ant, vertices_suc
        else:
            linha_v = self.grafoMA[i, :]
            index_adj = np.nonzero(linha_v)
            for x in index_adj[0]:
                vertices_adj.append(self.vertices[x])
            return vertices_adj, vertices_suc

    def testa_vertice(self, v):
        for x in self.vertices:
            if x == v:
                return True
        return False

filePath = input("Informe o caminho para o arquivo com dados do grafo: ")

file = open(filePath, "r") # Abre o arquivo para ler o seu conteúdo

arestas = []
vertices = []

vertices, arestas, tipoGrafo = processFileData(file)

'''print("\nExistem {} arestas\n".format(len(arestas)))
print(arestas)
print("\nExistem {} vértices\n".format(len(vertices)))
print(vertices)'''
file.close()

grafo = Grafo(vertices, arestas, tipoGrafo)
'''print("\nMatriz de adjacência: \n")
print()'''
grafoMA = grafo.criar_grafo()
#print(grafoMA)

while True:
    init() #Colorama init
    print("\033[1;37m"+"\nPor favor, informe a tarefa que você deseja realizar: ")
    ans = input("1 -> Saber se dois vértices são adjacentes ou não.\n"
                "2 -> Saber o grau de um vértice do grafo\n"
                "3 -> Buscar os vizinhos de um vértice qualquer do grafo\n"
                "4 -> Visitar todas as aretas do grafo.\n"
                "5 -> Plotar um grafo\n"
                "0 -> Encerrar o programa.\n"
                "Solicitação: ")
    if ans == '1':
        print("\nInforme os vértices que você gostaria de saber a relação.")
        while True:
            v1 = input("\nInforme o primeiro vértice : ")
            if grafo.testa_vertice(v1):
                break
            print("\033[1;31m"+"\nO Vértice {} informado não pertence ao grafo.".format(v1))
            print("Informe um vértice válido"+"\033[1;37m")

        while True:
            v2 = input("\nInforme o segundo vértice: ")
            if grafo.testa_vertice(v2):
                break
            print("\033[1;31m"+"\nO Vértice {} informado não pertence ao grafo.".format(v2))
            print("Informe um vértice válido."+"\033[1;37m")

        if grafo.testa_aresta(v1, v2):
            print("\033[1;31m"+"\nOs vertices {} e {} são adjacentes.".format(v1, v2))
        else:
            print("\033[1;31m"+"\nOs vertices {} e {} não são adjacentes.".format(v1, v2))

    elif ans == '2':

        while True:
            v = input("\nInforme qual vértice você gostaria de saber o grau: ")
            if grafo.testa_vertice(v):
                break
            print("\033[1;31m"+"\nO Vértice {} informado não pertence ao grafo.".format(v))
            print("Informe um vértice válido"+"\033[1;37m")

        index = vertices.index(v)
        grau_entrada, grau_saida = grafo.grau_vertice(v)

        if grau_saida == -1:
            print("\033[1;31m"+"\nO grau do {} é {}.".format(v, grau_entrada))
        else:
            print("\033[1;31m"+"\nGrau de entrada {} é {}".format(v, grau_entrada))
            print("Grau de saída {} é {}".format(v, grau_saida))

    elif ans == '3':

        while True:
            v = input("\nInforme qual vértice você gostaria de saber quais são os vizinhos: ")
            if grafo.testa_vertice(v):
                break
            print("\033[1;31m"+"\nO Vértice {} informado não pertence ao grafo.".format(v))
            print("Informe um vértice válido"+"\033[1;37m")

        adj, adj_ = grafo.adj_vertice(v)

        if not grafo.tipoGrafo:
            print("\033[1;31m" + "\nOs vértices adjacentes de {} são: ".format(v))
            for a in adj:
                print(a)
        else:
            if len(adj_):
                print("\033[1;31m"+"\nOs vértices adjacentes sucessores de {} são: ".format(v))
                for x in adj_:
                    print(x)
            if len(adj):
                print("\033[1;31m"+"\nOs vértices adjacentes antecessores de {} são: ".format(v))
                for y in adj:
                    print(y)

    elif ans == '4':
        print("\033[1;31m"+"Percorrendo as arestas: ")
        if tipoGrafo:
            for i in range(len(grafoMA)):
                for j in range(len(grafoMA)):
                    if grafoMA[i][j] == 1:
                        print("{} --> {}".format(grafo.vertices[i], grafo.vertices[j]))
        else:
            for i in range(len(grafoMA)):
                for j in range(len(grafoMA)-i):
                    if grafoMA[i][j+i] == 1:
                        print("{} <--> {}".format(grafo.vertices[i],grafo.vertices[j+i]))
    elif ans == '5':
        filePathPlot = input("Informe o caminho para o arquivo: ")

        filePlot = open(filePathPlot, 'r')
        verticesPlot = []
        arestasPlot = []

        verticesPlot, arestasPlot, tipoGPlot = processFileData(filePlot)

        if tipoGPlot:
            G = DiGraph() #Cria um grafo dirigido
        else:
            G = Graph() #Cria um grafo não dirigido
        G.add_nodes_from(verticesPlot) #Insere os nós no grafo
        G.add_edges_from(arestasPlot) #Insere as arestas no grafo
        draw_networkx(G, pos=spring_layout(G)) #Cria graficamente o grafo
        print('Para continuar feche a figura...')
        show() #Plota o grafo
        filePlot.close()

    elif ans == '0':
        print("\033[1;31m"+'\nO programa sera encerrado!')
        deinit()
        break
    else:
        print("\033[1;31m"+"\nOpcao Invalida!")
        print("Tente novamente!")
