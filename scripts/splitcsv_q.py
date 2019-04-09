# python splitcsv.py /Users/benjamin/ros/data/csv/hast_040.csv

import csv
import sys
import itertools

PATH = sys.argv[1]
DATE = sys.argv[2]
RUN = sys.argv[3]
OUTPATH = "/Users/benjamin/ros/data/" + DATE + "/csv/"
# print PATH
# print DATE
# print RUN

# source_path = datafile[0:len(datafile)-12]
# experiment_suffix = '_' + DATE + datafile[len(datafile)-8: len(datafile)]
# print experiment_suffix

datafile = PATH + "hast_" + DATE + '_' + RUN + ".csv"
print(datafile)
f = open(datafile, 'rt')
datalist = []
i = 0;
try:
	reader = csv.reader(f)
	for row in reader:
		i += 1
		if i == 3:
			objects = row
		if i > 5:
			datalist.append(row)
finally:
	f.close()

stripped_objects = filter(None, objects)
filenames=[]
for x in stripped_objects:
	y = x.split(":")
	# print y[1]
	filenames.append(OUTPATH+y[1]+ '_' + RUN + ".csv")

# print filenames
for x in range(0, len(filenames)):
	f = open(filenames[x], 'wt')
	try:
		writer = csv.writer(f)
		writer.writerow( ('Frame','RX', 'RY', 'RZ', 'RW', 'TX', 'TY', 'TZ') )
		for i in range(len(datalist)-1):
			writer.writerow( [datalist[i][0]] + datalist[i][(2+7*x):(9+7*x)])
	finally:
		f.close()

