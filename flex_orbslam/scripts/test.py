import json
from datetime import datetime
from collections import OrderedDict
# Opening JSON file
f = open('/home/vivekw964/vivek_ws/src/flex_orbslam/logs/BufferData.json')
  
# returns JSON object as 
# a dictionary
data = json.load(f)
failure_track = {
    "went_back_in_time": [],
    "repeated_frames": []
}
c = 0
i = 1
prev = data[0]
img_count = {
    data[0]["frameID"]: 1
}
while i <len(data):
    img_count[data[i]["frameID"]] = 1 + img_count.get(data[i]["frameID"], 0)
    #converting to datetime from string format
    prev1, prev2 = datetime.strptime(prev["timestamp_before_image_processed"].split(".")[0], "%m/%d/%Y, %H:%M:%S"), int(prev["timestamp_before_image_processed"].split(".")[1])
    next1, next2 = datetime.strptime(data[i]["timestamp_before_image_processed"].split(".")[0], "%m/%d/%Y, %H:%M:%S"), int(data[i]["timestamp_before_image_processed"].split(".")[1])
    # if data[i]["frameID"] == "10":
    #     print(prev2, next2, data[i]["frameID"]<=prev["frameID"])
    if int(data[i]["frameID"])<int(prev["frameID"]):
        failure_track["went_back_in_time"].append(OrderedDict({
            "prev_frameID": prev["frameID"],
            "prev_timestamp": prev["timestamp_before_image_processed"],
            "next_frameID": data[i]["frameID"],
            "next_timestamp": data[i]["timestamp_before_image_processed"]
        }))
    elif (int(data[i]["frameID"])==int(prev["frameID"])):
        failure_track["repeated_frames"].append(OrderedDict({
            "prev_frameID": prev["frameID"],
            "prev_timestamp": prev["timestamp_before_image_processed"],
            "next_frameID": data[i]["frameID"],
            "next_timestamp": data[i]["timestamp_before_image_processed"]
        }))
    prev = data[i]
    i+=1
print("Number of times went back in time ", len(failure_track["went_back_in_time"]))
print("Number of frames republished to ORBSLAM:", len(failure_track["repeated_frames"]))
# for frameID,count in img_count.items():
#     if count > 1:
#         print(frameID, count)
#         c+=1
# print("number of repeated images:", c)

with open('/home/vivekw964/vivek_ws/src/flex_orbslam/logs/failure_track.json', 'wb') as myfile:
    myfile.seek(0)
    json.dump(dict(failure_track), myfile, indent=4)
# or prev1>next1 or (prev1==next1 and prev2>next2)
