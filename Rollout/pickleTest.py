import pickle
import os

infile = open("./Dataset/data1", 'rb')
test = pickle.load(infile)
print(len(test[1]))
infile.close()
