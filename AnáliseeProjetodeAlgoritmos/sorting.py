from time import time, sleep
from datetime import datetime
from random import shuffle
import csv
import os
import psutil

_pid = os.getpid()
_ps = psutil.Process(_pid)

_memory_usage_bubble = 0
_memory_usage_selection = 0
_sum_memory_usage_merge = 0
_index = 0

def getUsageMemoryBubble():
    return _memory_usage_bubble

def getUsageMemorySelection():
    return _memory_usage_selection

def getUsageMemoryMerge():
    if _index == 0:
        return None
    return _sum_memory_usage_merge/_index

def reset():
    global _index
    global _sum_memory_usage_merge
    global _memory_usage_selection
    global _memory_usage_bubble

    _index = 0
    _sum_memory_usage_merge = 0
    _memory_usage_selection = 0
    _memory_usage_bubble = 0
    _memory_usage_merge = 0

def mergeSort(list):
    if len(list) > 1:
        mid = len(list) // 2  # Finding the mid of the array
        L = list[:mid]  # Dividing the array elements
        R = list[mid:]  # into 2 halves
        mergeSort(L)  # Sorting the first half
        mergeSort(R)  # Sorting the second half

        left_index = right_index = result_index = 0
        while left_index < len(L) and right_index < len(R):
            if L[left_index] < R[right_index]:
                list[result_index] = L[left_index]
                left_index += 1
            else:
                list[result_index] = R[right_index]
                right_index += 1
            result_index += 1
        while left_index < len(L):
            list[result_index] = L[left_index]
            left_index += 1
            result_index += 1
        while right_index < len(R):
            list[result_index] = R[right_index]
            right_index += 1
            result_index += 1
    global _sum_memory_usage_merge
    global _index
    _sum_memory_usage_merge += _ps.memory_info().vms #medição da memória
    _index += 1


def bubbleSort(lista):
    n = len(lista)
    trocado = True
    while trocado:
        trocado = False
        for i in range(n-1):
            if lista[i] > lista[i+1]:
                trocado = True
                lista[i], lista[i+1] = lista[i+1], lista[i]   # troca de elementos nas posições i e i+1
    global _memory_usage_bubble
    _memory_usage_bubble = _ps.memory_info().vms #medida da memoria

def selection_sort(lista):
    n = len(lista)
    for j in range(n-1):
        min_index = j
        for i in range(j, n):
            if lista[i] < lista[min_index]:
                min_index = i
        if lista[j] > lista[min_index]:
            lista[j], lista[min_index] = lista[min_index], lista[j]
    global _memory_usage_selection
    _memory_usage_selection = _ps.memory_info().vms # medida da memoria

# Code to print the list
def printList(arr):
    for i in range(len(arr)):
        print(arr[i], end=" ")
    print()

def getBubbleSortDuration(list):
    start_time = time()
    bubbleSort(list)
    duration = time() - start_time
    return duration

def getMergeSortDuration(list):
    start_time = time()
    mergeSort(list)
    duration = time() - start_time
    return duration

def getSelectionSortDuration(list):
    start_time = time()
    selection_sort(list)
    duration = time() - start_time
    return duration

def saveDataCSV(alg, n, case, duration, memory):
    data = [datetime.now().strftime("%Y-%m-%d %H:%M:%S"), alg, n, case, duration, memory]
    with open('sort_alg_data.csv', 'a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(data)
    file.close()

# driver code to test the above code
if __name__ == '__main__':

    filePath = input("Informe o caminho para o arquivo com os dados: ")
    file = open(filePath, "r")  # Abre o arquivo para ler o seu conteúdo
    n = int(file.readline().rstrip())
    alg = file.readline().rstrip()

    print("O algoritmo a ser utilizado é: " + alg)
    print("O tamanho da lista é {}".format(n))

    bettercase = []
    worstcase = []
    random_list = []

    for i in range(n + 1):  # Montando o melhor caso
        bettercase.append(i)

    for i in range(n + 1):  # Montando o pior caso
        worstcase.append(n - i)

    random_list = list(bettercase) #Preparando o caso médio

    if alg in "bubblesort":

        #Realizando o processo para o melhor caso
        bettercase_duration = getBubbleSortDuration(bettercase)
        print("O melhor caso foi encerrado em: ", bettercase_duration)
        saveDataCSV(alg, n, "melhor", bettercase_duration, getUsageMemoryBubble()/(1024 * 1024))
        reset()

        #Realizando o processo para o pior caso
        worstcase_duration = getBubbleSortDuration(worstcase)
        print("O pior caso foi encerrado em: ", worstcase_duration)
        saveDataCSV(alg, n, "pior", worstcase_duration, getUsageMemoryBubble()/(1024 * 1024))
        reset()

        #Realizando o caso randômico
        quant = 5 #Número de casos em que o processo será repetido
        duration_sum = 0
        sum_memory = 0

        for i in range(quant):
            shuffle(random_list)
            duration_sum += getBubbleSortDuration(random_list)
            sum_memory += getUsageMemoryBubble()
            reset()
        median_duration = duration_sum/quant
        print("O caso aleatorio foi encerrado em: ", median_duration)
        saveDataCSV(alg, n, "medio", median_duration, sum_memory/(quant*1024*1024))

    elif alg in "mergesort":

        #Realizando o processo para o melhor caso
        bettercase_duration = getMergeSortDuration(bettercase)
        print("O melhor caso foi encerrado em: ", bettercase_duration)
        saveDataCSV(alg, n, "melhor", bettercase_duration, getUsageMemoryMerge()/(1024 * 1024))
        reset()

        #Realizando o processo para o pior caso
        worstcase_duration = getMergeSortDuration(worstcase)
        print("O pior caso foi encerrado em: ", worstcase_duration)
        saveDataCSV(alg, n, "pior", worstcase_duration, getUsageMemoryMerge()/(1024 * 1024))
        reset()

        #Realizando o caso randômico
        quant = 5 #Número de casos em que o processo será repetido
        duration_sum = 0
        sum_memory = 0
        for i in range(quant):
            shuffle(random_list)
            duration_sum += getMergeSortDuration(random_list)
            sum_memory += getUsageMemoryMerge()
            reset()
        median_duration = duration_sum/quant
        print("O caso aleatorio foi encerrado em: ", median_duration)
        saveDataCSV(alg, n, "medio", median_duration, sum_memory/(quant*1024*1024))

    elif alg in "selectionsort":

        #Realizando o processo para o melhor caso
        bettercase_duration = getSelectionSortDuration(bettercase)
        print("O melhor caso foi encerrado em: ", bettercase_duration)
        saveDataCSV(alg, n, "melhor", bettercase_duration, getUsageMemorySelection()/(1024 * 1024))
        reset()

        #Realizando o processo para o pior caso
        worstcase_duration = getSelectionSortDuration(worstcase)
        print("O pior caso foi encerrado em: ", worstcase_duration)
        saveDataCSV(alg, n, "pior", worstcase_duration, getUsageMemorySelection()/(1024 * 1024))
        reset()

        #Realizando o caso randômico
        quant = 5 #Número de casos em que o processo será repetido
        duration_sum = 0
        sum_memory = 0

        for i in range(quant):
            shuffle(random_list)
            duration_sum += getSelectionSortDuration(random_list)
            sum_memory += getUsageMemorySelection()
            reset()
        median_duration = duration_sum/quant
        print("O caso aleatorio foi encerrado em: ", median_duration)
        saveDataCSV(alg, n, "medio", median_duration, sum_memory/(quant*1024*1024))
    else:
        print("Algoritmo de ordenação inválido. Tente: bubblesort, selectionsort ou mergesort")
        sleep(2)

# This code is contributed by Mayank Khanna (Merge Sort)
#This code is contribuited by Hallison Paz (Programação Dinâmica) (Bubble sort and Selection Sort)
