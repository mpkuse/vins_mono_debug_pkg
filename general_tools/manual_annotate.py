#!/usr/bin/env python


# In this script a human labels the seuquence.
# The video is played 1 frame at a time.
# Step-1:
# A bunch of consecutive images are grouped as 1 scene.
# Step-2:
# Every pair of scenes are shown and asked if they are the same physical places.
#
# Write this data to disk

# 22nd Nov, 2018, ==> TODO: This is yet not compatible with cerebro.
#       code up!

# Also have a look at `rosrun cv2_cam kitti_plot.py`

import cv2
import numpy as np
import collections
import json
import code
import os.path



# BASE = '/Bulk_Data/_tmp_cerebro/mynt_drone_fly_area_loopy/'
# BASE = '/Bulk_Data/_tmp_cerebro/mynt_pinhole_1loop_in_lab/'
# BASE = '/Bulk_Data/_tmp_cerebro/mynt_coffee-shop/'
BASE = '/Bulk_Data/_tmp_cerebro/ptgrey_floorg_lsk/'

# Step-0:
LOG_FILE_NAME = BASE+'/log.json'
print 'Open file: ', LOG_FILE_NAME
with open(LOG_FILE_NAME) as data_file:
    data = json.load(data_file)

# Collect all w_T_c, ie. VIO poses for plotting
VIO__w_T_i = []
idx_at = [] #this is global index of every image used.
image_at = []
print 'Loading VIO__w_t_i'
for i in range( len(data['DataNodes']) ):
    a = data['DataNodes'][i]['isKeyFrame']
    b = data['DataNodes'][i]['isImageAvailable']
    c = data['DataNodes'][i]['isPoseAvailable']
    d = data['DataNodes'][i]['isPtCldAvailable']


    if not ( a==1 and b==1 and c==1 and d==1 ): #only process keyframes which have pose and ptcld info
        continue

    w_T_c = np.array( data['DataNodes'][i]['w_T_c']['data']).reshape( (4,4) )
    VIO__w_T_i.append( w_T_c )

    idx_at.append( i )

    im = cv2.imread( BASE+'%d.jpg' %(i) )
    image_at.append( im )







# Step-1:

print '============Step-1: Annotate each frame of the sequence with SegmentIds'
if not os.path.isfile(BASE+'/loopcandidates_manually_marked_detailed.json'): #make this to false if you want to load file with seq
    scene_id = 0
    JSON_OBH = {}
    JSON_OBH['scene_seq_id'] = [] # scene ids for each frame
    for i in range( len( idx_at ) ):
        # fname = KITTI_BASE+'/%06d.jpg' %(i)
        # print 'READ: ', fname
        # im = cv2.imread( fname )
        im = image_at[ i ]
        idx = idx_at[ i ]
        print 'idx=', idx, '::: ', i, ' of ', len( idx_at ), '  current_scene_id=', scene_id
        cv2.imshow( 'im', im )
        key = cv2.waitKey(0)
        print 'press <space> to continue. press n to start new scene at this location'
        if key == ord( 'n' ):
            scene_id += 1

        JSON_OBH['scene_seq_id'].append( scene_id )

    print 'Writing ', BASE+'/loopcandidates_manually_marked_detailed.json'
    with open(BASE+'/loopcandidates_manually_marked_detailed.json', 'w') as outfile:
        # json.dumps(JSON_OBH, outfile, indent=4 )
        json.dump(JSON_OBH, outfile, indent=4)
    cv2.destroyWindow('im')

else:
    print 'File with manual sequence annotations exisits, so will proceed to step-2'
    print 'Reading ', BASE+'/loopcandidates_manually_marked_detailed.json'
    with open(BASE+'/loopcandidates_manually_marked_detailed.json') as data_file:
        JSON_OBH = json.load(data_file)

S = JSON_OBH['scene_seq_id']

# Make the inverted index
inv_S = collections.OrderedDict()
# inv_S = {}
inv_S[0] = {}
inv_S[0]['start'] = 0
for i in range(0, len( idx_at )-1 ):
    if S[i] != S[i+1]:
        inv_S[ S[i] ]['end'] = i
        inv_S[ S[i+1] ] = {}
        inv_S[ S[i+1] ]['start'] = i+1
inv_S[ inv_S.keys()[-1] ]['end'] =  len( idx_at )


# Review the segments
if False:
    for key in inv_S:
        val = inv_S[key]
        print 'key=',key, '\tval=', val
        __start = image_at[ val['start'] ]# cv2.imread(  KITTI_BASE+'/%06d.jpg' %(val['start'])   )
        __end = image_at[ val['end']-1 ]# cv2.imread(  KITTI_BASE+'/%06d.jpg' %(val['end'])   )
        mid_pt = int(  (val['start'] + val['end'] )/2 )
        __mid = image_at[ mid_pt  ]# cv2.imread(  KITTI_BASE+'/%06d.jpg' %(   (val['start'] + val['end'] )/2   ) )

        cv2.imshow( '__start', __start )
        cv2.imshow( '__end', __end )
        cv2.imshow( '__mid', __mid )
        cv2.waitKey(0)
    cv2.destroyWindow('__start')
    cv2.destroyWindow('__end')
    cv2.destroyWindow('__mid')




# Step-2:
# Show every pair. filter by data from poses.txt
print '====================Step-2:'
print 'You will be shown 2 smartly picked images which you are suppose to identify as being the same physical place or not'
matching_pairs = []
matching_pairs_T = [] # in a similar format of T.
for key0 in inv_S:
    for key1 in inv_S:
        val0 = inv_S[key0]
        val1 = inv_S[key1]

        position_key0 = VIO__w_T_i[ val0['start'] ][0:3,3] #poses[ val0['start']  ].reshape( (3,4) )[0:3,3]
        position_key1 = VIO__w_T_i[ val1['start'] ][0:3,3] #poses[ val1['start']  ].reshape( (3,4) )[0:3,3]
        if key0 >= key1 or abs(key0 - key1)<2 or np.linalg.norm(position_key0-position_key1) > 6.  or abs(val1['start'] - val0['start']) < 40:
            continue

        print '%d [%d,%d] <---> %d [%d,%d] match?(y,n). Total segs=%d' %(key0, val0['start'], val0['end'],  key1, val1['start'], val1['end'], len(inv_S.keys()) )

        mid_pt0 = int(  (val0['start'] + val0['end'] )/2 )
        mid_pt1 = int(  (val1['start'] + val1['end'] )/2 )
        __mid0 = image_at[ mid_pt0  ] # cv2.imread(  KITTI_BASE+'/%06d.jpg' %(   (val0['start'] + val0['end'] )/2   ) )
        __mid1 = image_at[ mid_pt1  ] #cv2.imread(  KITTI_BASE+'/%06d.jpg' %(   (val1['start'] + val1['end'] )/2   ) )

        cv2.imshow( '0', __mid0  )
        cv2.imshow( '1', __mid1  )

        key = cv2.waitKey(0)
        if key == ord('y'):
            matching_pairs.append( (key0, key1) )
            print 'MATCH ', key0, key1

            # mid -- mid,    mid -- start,    mid -- end
            # start -- mid,    start -- start,    start -- end
            # end -- mid,    end -- start,    end -- end
            matching_pairs_T.append( ((val0['start'] + val0['end'] )/2,   (val1['start'] + val1['end'] )/2, np.random.random() ) )
            # matching_pairs_T.append( ((val0['start'] + val0['end'] )/2,    val1['start'] ,                  np.random.random() ) )
            # matching_pairs_T.append( ((val0['start'] + val0['end'] )/2,    val1['end'] , np.random.random() ) )
            #
            # matching_pairs_T.append( (val0['start']  ,   (val1['start'] + val1['end'] )/2, np.random.random() ) )
            # matching_pairs_T.append( (val0['start'],    val1['start'] ,                  np.random.random() ) )
            # matching_pairs_T.append( (val0['start'],    val1['end'] , np.random.random() ) )
            #
            # matching_pairs_T.append( ( val0['end'] ,   (val1['start'] + val1['end'] )/2, np.random.random() ) )
            # matching_pairs_T.append( ( val0['end'] ,    val1['start'] ,                  np.random.random() ) )
            # matching_pairs_T.append( ( val0['end'] ,    val1['end'] , np.random.random() ) )


# to json
JSON_OBH['candidates'] = []
JSON_OBH['meta'] = "manually marked"
for match in matching_pairs_T:
    u = {}
    u['global_a'] = idx_at[ match[1] ]
    u['global_b'] = idx_at[ match[0] ]
    u['score'] = match[2]
    JSON_OBH['candidates'].append( u )

print 'Writing ', BASE+'/loopcandidates_manually_marked_detailed.json'
with open(BASE+'/loopcandidates_manually_marked_detailed.json', 'w') as outfile:
    # json.dumps(JSON_OBH, outfile, indent=4 )
    json.dump(JSON_OBH, outfile, indent=4)


print 'Writing ', BASE+'/loopcandidates_manually_marked.json'
JSON_OBH['candidates'][0]["meta"] = "These are all manually marked. The scores in this file have no meaning, they are just random numbers. They are here to maintain consist file format"
with open(BASE+'/loopcandidates_manually_marked.json', 'w') as outfile:
    # json.dumps(JSON_OBH, outfile, indent=4 )
    json.dump(JSON_OBH['candidates'], outfile, indent=4)
