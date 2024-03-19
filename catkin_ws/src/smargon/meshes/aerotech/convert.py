#!/usr/bin/env python3
import vtk

def reformat (infile, outfile):    
    wcnt=0
    with open(infile) as fin, open(outfile,'w') as fout:
        line = fin.readline()
        rcnt = 1
        while line:
            shiftX = 0. #-331.099
            shiftY = 0. #-469.19
            shiftZ = 0. #-144.333
            #print("Line {}: {}".format(cnt, line.strip())
            words = line.split()
            if (words[0] == "vertex"):
               words[1] = "%.9f" % ((float(words[1])+shiftX)*0.001)
               words[2] = "%.9f" % ((float(words[2])+shiftY)*0.001)
               words[3] = "%.9f" % ((float(words[3])+shiftZ)*0.001)
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


def ascii2bin (infile, outfile):
    reader = vtk.vtkSTLReader()
    # path to binary or ascii stl file to be converted
    reader.SetFileName(infile)
    reader.Update()
    
    write = vtk.vtkSTLWriter()
    #uncomment unnecessary 2Ascii or 2Binary
    #write.SetFileTypeToASCII()
    write.SetFileTypeToBinary()
    write.SetInputData(reader.GetOutput())
    # path to
    write.SetFileName(outfile)
    write.Write()
    print("Converted %s to Binary %s" % (infile, outfile))


reformat ('00_BASE.stl',   '00_BASEfrm.stl')
ascii2bin('00_BASEfrm.stl','00_BASEbin.stl')
reformat ('01_XTRANS.stl',   '01_XTRANSfrm.stl')
ascii2bin('01_XTRANSfrm.stl','01_XTRANSbin.stl')
reformat ('02_ZTRANS.stl',   '02_ZTRANSfrm.stl')
ascii2bin('02_ZTRANSfrm.stl','02_ZTRANSbin.stl')
reformat ('03_YTRANS.stl',   '03_YTRANSfrm.stl')
ascii2bin('03_YTRANSfrm.stl','03_YTRANSbin.stl')
reformat ('04_OMEGAROT.stl',   '04_OMEGAROTfrm.stl')
ascii2bin('04_OMEGAROTfrm.stl','04_OMEGAROTbin.stl')



    #reader.SetFileName("01_XTRANS.stl")
    #reader.SetFileName("02_ZTRANS.stl")
    #reader.SetFileName("03_YTRANS.stl")
    #reader.SetFileName("04_OMEGAROT.stl")
