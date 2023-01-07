#!/usr/bin/env python3
import csv

model1 = []
model2 = []
model3 = []
model4 = []
count1 = 0
count2 = 0
count3 = 0
count4 = 0
threshold = 'threshold_0.25'
output_path = '/home/kazuki/catkin_ws/src/nav_cloning/data/result_image/selected_training/thesis/'

for i in range(1, 41):
    if i % 2 == 0:
        if i % 4 == 0:
            csv_num = 4
        else:
            csv_num = 2
    else:
        csv_num = i - (int(i / 4) * 4)
    dirnum = int((i + 4 - 1) / 4)
#     if i % 2 == 0:
#         dirnum = int(i / 2)
#         csv_num = 2
#     else:
#         dirnum = int(i - int(i / 2))
#         csv_num = 1
    file_path = '/home/kazuki/catkin_ws/src/nav_cloning/data/result_image/selected_training/thesis/'+str(dirnum)+'/'+str(threshold)+'/result/traceable'+str(csv_num)+'.csv'
    try:
        with open(file_path, 'r') as f:
            for row in csv.reader(f):
                if csv_num == 1:
                    model1.append(row)
                    if row[0] == 'Failure':
                        count1 += 1
                elif csv_num == 2:
                    model2.append(row)
                    if row[0] == 'Failure':
                        count2 += 1
                elif csv_num == 3:
                    model3.append(row)
                    if row[0] == 'Failure':
                        count3 += 1
                else:
                    model4.append(row)
                    if row[0] == 'Failure':
                        count4 += 1
    except FileNotFoundError:
        pass
print('model1')
# print(model1)
print(str(count1) + '/' + str(len(model1)))
print('model2')
# print(model2)
print(str(count2) + '/' + str(len(model2)))
print('model3')
# print(model2)
print(str(count3) + '/' + str(len(model3)))
print('model4')
# print(model2)
print(str(count4) + '/' + str(len(model4)))

with open(output_path + str(threshold) + 'model1.csv', 'a') as a:
    writer = csv.writer(a)
    writer.writerows(model1)
    writer.writerow([str(count1) + '/' + str(len(model1))])

with open(output_path + str(threshold) + 'model2.csv', 'a') as b:
    writer = csv.writer(b)
    writer.writerows(model2)
    writer.writerow([str(count2) + '/' + str(len(model2))])

with open(output_path + str(threshold) + 'model3.csv', 'a') as c:
    writer = csv.writer(c)
    writer.writerows(model3)
    writer.writerow([str(count3) + '/' + str(len(model3))])

with open(output_path + str(threshold) + 'model4.csv', 'a') as d:
    writer = csv.writer(d)
    writer.writerows(model4)
    writer.writerow([str(count4) + '/' + str(len(model4))])