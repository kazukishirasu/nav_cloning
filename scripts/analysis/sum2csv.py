#!/usr/bin/env python3
import csv

model1 = []
model2 = []
count1 = 0
count2 = 0
threshold = 'threshold_0.31'
output_path = '/home/kazuki/catkin_ws/src/nav_cloning/data/result_image/selected_training/thesis/'

for i in range(1, 21):
    if i % 2 == 0:
        dirnum = int(i / 2)
        csv_num = 2
    else:
        dirnum = int(i - int(i / 2))
        csv_num = 1
    file_path = '/home/kazuki/catkin_ws/src/nav_cloning/data/result_image/selected_training/thesis/'+str(dirnum)+'/'+str(threshold)+'/result/traceable'+str(csv_num)+'.csv'
    try:
        with open(file_path, 'r') as f:
            for row in csv.reader(f):
                if csv_num == 1:
                    model1.append(row)
                    if row[0] == 'Failure':
                        count1 += 1
                else:
                    model2.append(row)
                    if row[0] == 'Failure':
                        count2 += 1
    except FileNotFoundError:
        pass
print('model1')
print(model1)
print(str(count1) + '/' + str(len(model1)))
print('model2')
print(model2)
print(str(count2) + '/' + str(len(model2)))

with open(output_path + str(threshold) + 'model1.csv', 'a') as a:
    writer = csv.writer(a)
    writer.writerows(model1)
    writer.writerow([str(count1) + '/' + str(len(model1))])

with open(output_path + str(threshold) + 'model2.csv', 'a') as b:
    writer = csv.writer(b)
    writer.writerows(model2)
    writer.writerow([str(count2) + '/' + str(len(model2))])