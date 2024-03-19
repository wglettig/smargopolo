#!/usr/bin/env python3
wcnt=0
with open("SMARGON.RED_ALLCATPART.stl") as fin, open("RED.stl",'w') as fout:
    line = fin.readline()
    rcnt = 1
    while line:
        #print("Line {}: {}".format(cnt, line.strip())
        words = line.split()

        if (words[0] == "vertex"):
           words[1] = "%.9f" % ((float(words[1])-443.399)*0.001)
           words[2] = "%.9f" % ((float(words[2])-469.241)*0.001)
           words[3] = "%.9f" % ((float(words[3])-147.22)*0.001)
           line_new = '      vertex %s %s %s \n' % (words[1],words[2],words[3])
           #print (line_new)
           fout.writelines(line_new)
           wcnt+=1
        elif (len(words)>1 and words[1] == "normal"):
           #words[2] = "%.9f" % (-float(words[2]))
           line_new = '%s %s %s %s %s\n' % (words[0], words[1],words[2],words[3],words[4])
           #print (line_new)
           fout.writelines(line_new)
           wcnt+=1
        else:
           fout.writelines(line)
        line = fin.readline()
        rcnt+=1
        
print("lines read: %d, lines modified: %d" % (rcnt, wcnt));
