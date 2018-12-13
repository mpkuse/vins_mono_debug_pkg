## Load the log.json. Compute descriptors for each image and store them

import caffe
import numpy as np
import cv2
import code
import json
import time

from simplistic_experiment import load_alex_net, get_alexnet_descriptor
from simplistic_experiment import load_calc_net, get_calc_descriptor



#------------ Load log.json
BASE = '/Bulk_Data/_tmp_cerebro/mynt_coffee-shop/'
LOG_FILE_NAME = BASE+'/log.json'
print 'Open file: ', LOG_FILE_NAME
with open(LOG_FILE_NAME) as data_file:
    data = json.load(data_file)


#------------ Init AlexNet/calc
# model_type = 'caffemodel_calc_descriptor'
# net = load_calc_net()
# FN = get_calc_descriptor

# model_type = 'caffemodel_alexnet_descriptor'
# net = load_alex_net()
# FN = get_alexnet_descriptor


#----- Gaussian Random Matrix
model_type = 'caffemodel_alexnet_descriptor_GRM'
net = load_alex_net()
FN = get_alexnet_descriptor
M = np.random.normal( 0,1, (4096,64896) )

netvlad_desc = []
netvlad_at_i = []
for i in range( len(data['DataNodes']) ):
    a = data['DataNodes'][i]['isKeyFrame']
    b = data['DataNodes'][i]['isImageAvailable']
    c = data['DataNodes'][i]['isPoseAvailable']
    d = data['DataNodes'][i]['isPtCldAvailable']


    if not ( a==1 and b==1 and c==1 and d==1 ): #only process keyframes which have pose and ptcld info
        continue

    im = cv2.imread( BASE+'%d.jpg' %(i) )

    start_time = time.time()
    print '---', i , ' of ', len(data['DataNodes']), '\n'
    print 'im.shape ', im.shape

    _descriptor = FN( net , im ) #.shape : 1064,
    _descriptor = np.array(_descriptor)



    if 'M' in vars(): # Do Gaussian Random Normalization only if M is defined. skip otherwise
        u = np.matmul( M, _descriptor )
        u = u / np.linalg.norm( u )
        netvlad_desc.append( u )
    else:
        netvlad_desc.append( _descriptor )


    netvlad_at_i.append( i )
    print 'Done in %4.4fms' %( 1000. * (time.time() - start_time ) ),
    print '\tdesc.shape=', str( _descriptor.shape )

    continue

    cv2.imshow( 'im', im )
    key = cv2.waitKey(10)
    if key == ord( 'q' ):
        break


netvlad_desc = np.array( netvlad_desc ) # N x 4096. 4096 is the length of netvlad vector.

fname = BASE+'/'+model_type+'.npz'
print 'Save `netvlad_desc` (shape=%s) and `netvlad_at_i` in ' %( str(netvlad_desc.shape) ), fname
np.savez_compressed( fname, netvlad_desc=netvlad_desc, netvlad_at_i=netvlad_at_i)
