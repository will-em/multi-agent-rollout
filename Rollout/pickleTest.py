import pickle
import os

infile = open("./Dataset/data2", 'rb')
test = pickle.load(infile)
print(len(test[0]))
infile.close()
