import numpy as np

def mark_hit_timestamp(temp_list):
    # input: temp_list 
    #    temp_list must be filtered data
    # output: index_list 
    diff_list = []
    index_list = []
    for i in range(1, len(temp_list)):
        print('temp_list[i]', temp_list[i])
        pre_sensor_val = temp_list[i-1]
        cur_sensor_val = temp_list[i]
        diff = cur_sensor_val - pre_sensor_val
        diff_list.append(diff)
        
    for i in range(1, len(diff_list)):
        if((diff_list[i-1] > 0) and (diff_list[i] < 0)):
            # it is a peak
            index_list.append(i) 
    print("index list:", index_list)

test_list = [4,5,6,5,4,2,4,6,7,8,9,8,6,5,4,3,2,1,2,3,4,5,6,7,8,9,8,7,6,5,4,2,1,3,4,5,6,8,9,10,11,9,7,6,5,4,3,2,1,0,1,2,3,4,5]
mark_hit_timestamp(test_list)
mean = np.mean(test_list)
print(mean)