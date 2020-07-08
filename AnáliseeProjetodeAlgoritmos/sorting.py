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
    global _sum_memory_usage_merge
    global _memory_usage_merge
    global _index
    if len(list) > 1:
        mid = len(list) // 2  # Finding the mid of the array
        L = list[:mid]  # Dividing the array elements
        R = list[mid:]  # into 2 halves
        mergeSort(L)  # Sorting the first half
        mergeSort(R)  # Sorting the second half

        i = j = k = 0

        # Copy data to temp arrays L[] and R[]
        while i < len(L) and j < len(R):
            if L[i] < R[j]:
                list[k] = L[i]
                i += 1
            else:
                list[k] = R[j]
                j += 1
            k += 1

        # Checking if any element was left
        while i < len(L):
            list[k] = L[i]
            i += 1
            k += 1

        while j < len(R):
            list[k] = R[j]
            j += 1
            k += 1
    _sum_memory_usage_merge += _ps.memory_info().vms
    _index += 1

def bubbleSort(lista):
    global _memory_usage_bubble
    n = len(lista)
    i = 0
    troca = True
    while troca:
        troca = False
        for i in range(n-1):
            if lista[i] > lista[i+1]:
                troca = True
                # troca de elementos nas posições i e i+1
                lista[i], lista[i+1] = lista[i+1], lista[i]
    _memory_usage_bubble = _ps.memory_info().vms

def selection_sort(lista):
    global _memory_usage_selection
    n = len(lista)
    for j in range(n-1):
        min_index = j
        for i in range(j, n):
            if lista[i] < lista[min_index]:
                min_index = i
        if lista[j] > lista[min_index]:
            aux = lista[j]
            lista[j] = lista[min_index]
            lista[min_index] = aux
    _memory_usage_selection = _ps.memory_info().vms

# This code is contributed by Mayank Khanna (Merge Sort)
#This code is contribuited by Hallison Paz (Programação Dinâmica) (Bubble sort and Selection Sort)
