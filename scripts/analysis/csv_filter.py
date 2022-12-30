import csv

file_path = '/home/kazuki/Downloads/use_dl_output/'
success_list = []
failure_list = []
list = []
flag = False
num = 0

with open(file_path + 'trajectory.csv', 'r') as a:
    for row in csv.reader(a):
        if row[4] == 'True':
            list.append(int(row[3])-1)

with open(file_path + 'trajectory.csv', 'r') as a:
    for row in csv.reader(a):
        if int(row[3]) in list:
            failure_list.append(row)
        else:
            success_list.append(row)

with open(file_path + 'success_trajectory.csv', 'a') as b:
    writer = csv.writer(b)
    writer.writerows(success_list)

with open(file_path + 'failure_trajectory.csv', 'a') as c:
    writer = csv.writer(c)
    writer.writerows(failure_list)