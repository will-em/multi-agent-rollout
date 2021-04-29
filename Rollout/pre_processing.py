import pickle
import os
import numpy as np
#import tensorflow

infile = open("./Dataset/data438", 'rb')
data = pickle.load(infile)
infile.close()

input_vec = []
output_vec = []

for mat, targets in data[0]:
    new_mat = mat.flatten()
    for target in targets:
        new_mat = np.append(new_mat, target[0])
        new_mat = np.append(new_mat, target[1])
    new_mat = new_mat/19
    input_vec.append(new_mat)

for vec in data[1]:
    output = [0]*40
    for i, element in enumerate(vec):
        output[5*i+element] = 1
    output_vec.append(output)
outfile = open("./pre_processed_data", 'wb')
pickle.dump((input_vec, output_vec), outfile)
outfile.close()
