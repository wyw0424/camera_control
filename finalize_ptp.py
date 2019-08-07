import os
import exifread
import time
from datetime import datetime

out = open("./image_info.txt", "w");
all_pic={}

for (root, dirs, files) in os.walk("./"):
    for filename in files:
        f = open(filename, "rb");
        tags = exifread.process_file(f);
        for tag in tags.keys():
            if tag in ('Image DateTime'):
                pic_time = tags[tag]
                
                #print str(pic_time)
                tmp = str(pic_time).split(" ")
                data = str(tmp).split(":")
                year = int(data[0].split("'")[1])
                
                month = int(data[1])
                day = int(data[2].split("'")[0])
                hour = int(data[2].split("'")[2])
                minute = int(data[3])
                second = int(data[4].split("'")[0])

                stamp = datetime(year, month, day, hour, minute, second)
                
                epoch_time = time.mktime(stamp.timetuple())
                #print epoch_time
                dateTime = tags[tag].values.replace(':', '').replace(' ', 'T');
                
                index = int(filename.split(".jpg")[0])
                
                all_pic.update({index:epoch_time})
                
for data in all_pic:
    print str(data) + ": " + str(all_pic[data])
    out.write("%s %s\n" % (data, all_pic[data]));
