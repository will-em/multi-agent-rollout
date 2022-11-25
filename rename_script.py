import os


folder = "./frames"
for count, filename in enumerate(os.listdir(folder)):
    name = filename.split("_")
    while len(name[1]) < 8:
        name[1] = "0" + name[1]

    # rename() function will
    # rename all the files
    os.rename("./frames/"+filename, "./frames/" + "".join(name))
