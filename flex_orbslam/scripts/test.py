import json
from datetime import datetime
# Opening JSON file
f = open('/home/vivekw964/vivek_ws/src/flex_orbslam/logs/BufferData.json')
  
# returns JSON object as 
# a dictionary
data = json.load(f)
failure_track = []
i = 1
prev = data[0]
while i <len(data):
    #converting to datetime from string format
    prev1, prev2 = datetime.strptime(prev["timestamp_before_image_processed"].split(".")[0], "%m/%d/%Y, %H:%M:%S"), int(prev["timestamp_before_image_processed"].split(".")[1])
    next1, next2 = datetime.strptime(data[i]["timestamp_before_image_processed"].split(".")[0], "%m/%d/%Y, %H:%M:%S"), int(data[i]["timestamp_before_image_processed"].split(".")[1])
    # if data[i]["frameID"] == "10":
    #     print(prev2, next2, data[i]["frameID"]<=prev["frameID"])
    if int(data[i]["frameID"])<=int(prev["frameID"]) or prev1>next1 or (prev1==next1 and prev2>next2):
        failure_track.append({
            "prev_frameID": prev["frameID"],
            "prev_timestamp": prev["timestamp_before_image_processed"],
            "next_frameID": data[i]["frameID"],
            "next_timestamp": data[i]["timestamp_before_image_processed"]
        })
    prev = data[i]
    i+=1
print("Number of times went back in time ", len(failure_track))
with open('/home/vivekw964/vivek_ws/src/flex_orbslam/logs/failure_track.json', 'wb') as myfile:
    myfile.seek(0)
    json.dump(failure_track, myfile, indent=4)
