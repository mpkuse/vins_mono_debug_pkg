#!/usr/bin/env python

import numpy as np
import code
import cv2

import json


BASE = '/Bulk_Data/_tmp'


LOG_FILE_NAME = BASE+'/log.json'
print 'Open file: ', LOG_FILE_NAME
with open(LOG_FILE_NAME) as data_file:
    data = json.load(data_file)


for i in range( len(data['DataNodes']) ):
    if data['DataNodes'][i]['isUnVnAvailable']!= 1:
        continue

    cv2.imshow( 'im', cv2.imread( BASE+'/%d.jpg' %(i) ) )

    print i, data['DataNodes'][i]["unvn"]["cols"]
    cv2.waitKey(0)
