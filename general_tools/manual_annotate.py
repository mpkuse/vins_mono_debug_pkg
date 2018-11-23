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
def _Param():
    PARAMS = []
    KITTI_BASE='/home/mpkuse/Downloads/loop_closure_evaluation/__seq__/KITTI/KITTI00/'
    start_ = 0
    end_ = 648
    PARAMS.append( (KITTI_BASE, start_, end_) ) #N=0

    KITTI_BASE='/home/mpkuse/Downloads/loop_closure_evaluation/__seq__/KITTI/KITTI05/'
    start_ = 0
    end_ = 394
    PARAMS.append( (KITTI_BASE, start_, end_) ) #N=1

    KITTI_BASE='/home/mpkuse/Downloads/loop_closure_evaluation/__seq__/KITTI/tpt-park/'
    start_ = 0
    end_ = 421
    PARAMS.append( (KITTI_BASE, start_, end_) ) #N=2

    KITTI_BASE='/home/mpkuse/Downloads/loop_closure_evaluation/__seq__/KITTI/coffee-shop/'
    start_ = 0
    end_ = 940
    PARAMS.append( (KITTI_BASE, start_, end_) ) #N=3

    KITTI_BASE='/home/mpkuse/Downloads/loop_closure_evaluation/__seq__/KITTI/seng-base/'
    start_ = 0
    end_ = 704
    PARAMS.append( (KITTI_BASE, start_, end_) ) #N=4

    KITTI_BASE='/home/mpkuse/Downloads/loop_closure_evaluation/__seq__/KITTI/lsk-1/'
    start_ = 0
    end_ = 740
    PARAMS.append( (KITTI_BASE, start_, end_) ) #N=5

    KITTI_BASE='/home/mpkuse/Downloads/loop_closure_evaluation/__seq__/KITTI/base-2/'
    start_ = 0
    end_ = 1058
    PARAMS.append( (KITTI_BASE, start_, end_) ) #N=6
    return PARAMS


N = 6 # Choose dataset. [0,5]. 0: KITTI00;   1: KITTI05;   2: tpt-park;   3: coffee-shop;  4: seng-base;  5: lsk-1

PARAMS = _Param()
KITTI_BASE = PARAMS[N][0]
start_ = PARAMS[N][1]
end_   = PARAMS[N][2]
poses = np.loadtxt( KITTI_BASE+'/poses.txt' )


# Step-1:
print 'Step-1: Annotate each frame of the sequence with SegmentIds'
if False:
    scene_id = 0
    S = [] # scene ids for each frame
    for i in range(start_,end_+1):
        fname = KITTI_BASE+'/%06d.jpg' %(i)
        print 'READ: ', fname
        im = cv2.imread( fname )
        cv2.imshow( 'im', im )
        key = cv2.waitKey(0)
        print 'press <space> to continue. press n to start new scene at this location'
        if key == ord( 'n' ):
            scene_id += 1

        S.append( scene_id )

    print 'Writing ', KITTI_BASE+'/human_marked_per_frame_seg_id.txt'
    np.savetxt( KITTI_BASE+'/human_marked_per_frame_seg_id.txt', np.array(S).astype('int32'), fmt='%d', header='segId of each frame. Length of this file equals the number of images in this seq. This is marked by a human and is subject to annotators-bias' )
else:
    S = np.loadtxt( KITTI_BASE+'/human_marked_per_frame_seg_id.txt' )
    # import code
    # code.interact( local=locals() )



# Make the inverted index
inv_S = collections.OrderedDict()
# inv_S = {}
inv_S[0] = {}
inv_S[0]['start'] = 0
for i in range(start_,end_):
    if S[i] != S[i+1]:
        inv_S[ S[i] ]['end'] = i
        inv_S[ S[i+1] ] = {}
        inv_S[ S[i+1] ]['start'] = i+1
inv_S[ inv_S.keys()[-1] ]['end'] = end_


# Review the segments
if False:
    for key in inv_S:
        print key
        val = inv_S[key]
        __start = cv2.imread(  KITTI_BASE+'/%06d.jpg' %(val['start'])   )
        __end = cv2.imread(  KITTI_BASE+'/%06d.jpg' %(val['end'])   )
        __mid = cv2.imread(  KITTI_BASE+'/%06d.jpg' %(   (val['start'] + val['end'] )/2   ) )

        cv2.imshow( '__start', __start )
        cv2.imshow( '__end', __end )
        cv2.imshow( '__mid', __mid )
        cv2.waitKey(0)


# Step-2:
# Show every pair. filter by data from poses.txt
print 'You will be shown 2 smartly picked images which you are suppose to identify as being the same physical place or not'
matching_pairs = []
matching_pairs_T = [] # in a similar format of T.
for key0 in inv_S:
    for key1 in inv_S:
        val0 = inv_S[key0]
        val1 = inv_S[key1]

        position_key0 = poses[ val0['start']  ].reshape( (3,4) )[0:3,3]
        position_key1 = poses[ val1['start']  ].reshape( (3,4) )[0:3,3]
        if key0 >= key1 or abs(key0 - key1)<2 or np.linalg.norm(position_key0-position_key1) > 6.  or abs(val1['start'] - val0['start']) < 40:
            continue

        print '%d [%d,%d] <---> %d [%d,%d] match?(y,n)' %(key0, val0['start'], val0['end'],  key1, val1['start'], val1['end'] )

        __mid0 = cv2.imread(  KITTI_BASE+'/%06d.jpg' %(   (val0['start'] + val0['end'] )/2   ) )
        __mid1 = cv2.imread(  KITTI_BASE+'/%06d.jpg' %(   (val1['start'] + val1['end'] )/2   ) )

        cv2.imshow( '0', __mid0  )
        cv2.imshow( '1', __mid1  )

        key = cv2.waitKey(0)
        if key == ord('y'):
            matching_pairs.append( (key0, key1) )

            # mid -- mid,    mid -- start,    mid -- end
            # start -- mid,    start -- start,    start -- end
            # end -- mid,    end -- start,    end -- end
            matching_pairs_T.append( ((val0['start'] + val0['end'] )/2,   (val1['start'] + val1['end'] )/2, np.random.random() ) )
            matching_pairs_T.append( ((val0['start'] + val0['end'] )/2,    val1['start'] ,                  np.random.random() ) )
            matching_pairs_T.append( ((val0['start'] + val0['end'] )/2,    val1['end'] , np.random.random() ) )

            matching_pairs_T.append( (val0['start']  ,   (val1['start'] + val1['end'] )/2, np.random.random() ) )
            matching_pairs_T.append( (val0['start'],    val1['start'] ,                  np.random.random() ) )
            matching_pairs_T.append( (val0['start'],    val1['end'] , np.random.random() ) )

            matching_pairs_T.append( ( val0['end'] ,   (val1['start'] + val1['end'] )/2, np.random.random() ) )
            matching_pairs_T.append( ( val0['end'] ,    val1['start'] ,                  np.random.random() ) )
            matching_pairs_T.append( ( val0['end'] ,    val1['end'] , np.random.random() ) )


np.savetxt( KITTI_BASE + '/human_marked_matching_segments.txt', np.array(matching_pairs), fmt='%d %d', header='list of matching segIds. These 2 segments represent the same places as marked by a human. This is marked by a human and is subject to annotators-bias')
np.savetxt( KITTI_BASE + '/human_marked_frame_match.txt', np.array(matching_pairs_T), fmt='%d %d %f', header='list of matching frames as marked by a human. scores (3rd column) are random.' )

print 'Writing ', KITTI_BASE+'/per_frame_seg_id.txt'
print 'Writing ', KITTI_BASE + '/matching_segments.txt'
print 'Writing ', KITTI_BASE + '/human_marked_frame_match.txt'
