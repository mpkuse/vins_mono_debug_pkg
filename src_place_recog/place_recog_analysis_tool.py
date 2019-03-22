#!/usr/bin/env python

## This tool reads the json files, file.npz (containing the whole image descriptors)
## It produces visualization_msgs::Marker, Image-current, Image-looppair to be
## viewed with rviz.

import numpy as np
import code
import cv2
import time
import datetime

import json

# Needed to make ros-service call.
import rospy
from cv_bridge import CvBridge, CvBridgeError
from cerebro.srv import *


# publishing
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


from unit_tools import publish_marker, imshow_loopcandidates, play_trajectory_with_loopcandidates


# returns selected loop candidates by locality heuristic and the threshold.
# T = [ (p0, p1, score), .... ]
# S = [ (p0, p1, score), .... ] ==> a subset of T
def filter_candidates( T, TH=0.92, locality=8 ):
    S = []
    for i in range( 0,len(T)-3, 3 ):
        p0 = int(T[i][0])
        p1 = int(T[i][1])
        score = T[i][2]

        if score > TH and abs(T[i+1][1] - p1) < locality and abs(T[i+2][1] - p1) < locality: # and T[i+1][2] > TH and T[i+2][2] > TH:
            S.append( (p0, p1, score) )
    return S


def filter_candidates_gt_thresh( T, TH ):
    S = [ tt for tt in T if tt[2] > TH ]
    return S

def filter_candidates_lt_thresh( T, TH ):
    S = [ tt for tt in T if tt[2] < TH ]
    return S


# import keras
# class SampleGPUComputer:
#     def __init__(self):
#         self.model = keras.applications.vgg16.VGG16( weights=None)
#         self.model.summary()
#
#     def compute( self, im ):
#         im = cv2.resize( im, (224,224) )
#         preds = self.model.predict( np.expand_dims(im,0) )
#         return preds

if __name__ == "__main__":
    BASE = '/Bulk_Data/_tmp/'
    # BASE = '/Bulk_Data/_tmp_cerebro/bb4_multiple_loops_in_lab/'
    # BASE = '/Bulk_Data/_tmp_cerebro/bb4_loopy_drone_fly_area/'
    # BASE = '/Bulk_Data/_tmp_cerebro/bb4_long_lab_traj/'
    # BASE = '/Bulk_Data/_tmp_cerebro/bb4_floor2_cyt/'

    # BASE = '/Bulk_Data/_tmp_cerebro/euroc_MH_01_easy/'
    # BASE = '/Bulk_Data/_tmp_cerebro/euroc_MH_02_easy/'

    # BASE = '/Bulk_Data/_tmp_cerebro/ptgrey_floorg_lsk/'
    # BASE = '/Bulk_Data/_tmp_cerebro/ptgrey_tpt_park/'

    # BASE = '/Bulk_Data/_tmp_cerebro/tum_magistrale2/'
    # BASE = '/Bulk_Data/_tmp_cerebro/tum_magistrale4/'

    # BASE = '/Bulk_Data/_tmp_cerebro/mynt_multi_loops_in_lab/'
    # BASE = '/Bulk_Data/_tmp_cerebro/mynt_drone_fly_area_loopy/'
    # BASE = '/Bulk_Data/_tmp_cerebro/mynt_pinhole_1loop_in_lab/'
    # BASE = '/Bulk_Data/_tmp_cerebro/mynt_coffee-shop/'

    # BASE = '/Bulk_Data/_tmp_cerebro/ptgrey_floorg_lsk/'
    # BASE = '/Bulk_Data/_tmp_cerebro/mynt_seng3/'
    # BASE = '/Bulk_Data/_tmp_cerebro/bb4_long_lab_traj/'




    #
    # Open Log File and load VIO poses.
    #
    LOG_FILE_NAME = BASE+'/log.json'
    print 'Open file: ', LOG_FILE_NAME
    with open(LOG_FILE_NAME) as data_file:
        data = json.load(data_file)

    # Collect all w_T_c, ie. VIO poses for plotting
    VIO__w_t_i = []
    w_t_c = np.array( [0,0,0] )
    print 'Loading VIO__w_t_i'
    for i in range( len(data['DataNodes']) ):
        if data['DataNodes'][i]['isPoseAvailable'] == 0:
            # w_t_c = np.array( [0,0,0] )
            pass # use the previously known pose
        else:
            w_T_c = np.array( data['DataNodes'][i]['w_T_c']['data']).reshape( (4,4) )
            w_t_c = w_T_c[0:3,3]

        VIO__w_t_i.append( w_t_c )



    #
    # ROS Node
    #
    rospy.init_node('talker', anonymous=True)
    rospy.myargv(argv=sys.argv)
    print 'Publish Topic : Marker::chatter, Image::frames, Image::loopcandidates'
    pub = rospy.Publisher('chatter', Marker, queue_size=100)
    pub_frames = rospy.Publisher('frames', Image, queue_size=50)
    pub_loopcandidates = rospy.Publisher('loopcandidates', Image, queue_size=50)

    DESCRIPTOR_STR = ""

    # Create Loop Candidates or Load file loopcandidates_?_.json
    if True:
        #
        # Loops over all images and precomputes their netvlad vector (or read the .npz file)
        #
        if True: #making this to false will load npz files which contain the pre-computes descriptors.
            #
            # Init Keras Model - NetVLAD / Enable Service
            #
            # gpu_s = SampleGPUComputer()
            print 'waiting for ros-service : whole_image_descriptor_compute'
            rospy.wait_for_service( 'whole_image_descriptor_compute' )
            try:
                service_proxy = rospy.ServiceProxy( 'whole_image_descriptor_compute', WholeImageDescriptorCompute )
            except rospy.ServiceException, e:
                print 'failed', e
            print 'connected to ros-service'

            #try the 0th image
            im = cv2.imread( BASE+'%d.jpg' %(0) )
            image_msg = CvBridge().cv2_to_imgmsg( im )
            rcvd_ = service_proxy( image_msg, 24 )

            netvlad_desc = []
            netvlad_at_i = []
            print ' len(data[\'DataNodes\'])=',  len(data['DataNodes'])
            for i in range( len(data['DataNodes']) ):
                a = data['DataNodes'][i]['isKeyFrame']
                b = data['DataNodes'][i]['isImageAvailable']
                c = data['DataNodes'][i]['isPoseAvailable']
                d = data['DataNodes'][i]['isPtCldAvailable']


                if not ( a==1 and b==1 and c==1  ): #only process keyframes which have pose and ptcld info
                    continue

                im = cv2.imread( BASE+'%d.jpg' %(i) )


                start_time = time.time()
                print '---', i , ' of ', len(data['DataNodes']), '\n'
                image_msg = CvBridge().cv2_to_imgmsg( im )
                rcvd_ = service_proxy( image_msg, 24 )
                netvlad_desc.append( rcvd_.desc )
                netvlad_at_i.append( i )
                print 'Done in %4.4fms' %( 1000. * (time.time() - start_time ) ),
                print '\tdesc.shape=', str( np.array(rcvd_.desc).shape )
                print '\tdesc.model_type=', rcvd_.model_type

                cv2.imshow( 'im', im )
                key = cv2.waitKey(10)
                if key == ord( 'q' ):
                    break
            netvlad_desc = np.array( netvlad_desc ) # N x 4096. 4096 is the length of netvlad vector.

            # fname = BASE+'/file.npz' #TODO, will read the model filename from server-response.
            fname = BASE+'/'+rcvd_.model_type+'.npz'
            print 'Save `netvlad_desc` (shape=%s) and `netvlad_at_i` in ' %( str(netvlad_desc.shape) ), fname
            np.savez_compressed( fname, netvlad_desc=netvlad_desc, netvlad_at_i=netvlad_at_i)
            DESCRIPTOR_STR = 'from server model_type='+rcvd_.model_type
        else:
            # fname = BASE+'/mobilenet_conv7_allpairloss.npz'
            # fname = BASE + '/mobilenet_conv7_quash_chnls_tripletloss2.npz'
            # fname = BASE + '/mobilenet_conv7_quash_chnls_allpairloss.npz'
            # fname = BASE+'/block5_pool_k16_allpairloss.npz'
            # fname = BASE+'/relja_matlab_model.npz'
            # fname = BASE+'/caffemodel_calc_descriptor.npz'
            # fname = BASE+'/caffemodel_alexnet_descriptor_GRM.npz'
            # fname = BASE+'/caffemodel_alexnet_descriptor.npz'
            DESCRIPTOR_STR = 'from '+fname

            print 'Load ', fname
            loaded = np.load(fname)
            netvlad_desc = loaded['netvlad_desc']
            netvlad_at_i = loaded['netvlad_at_i']
            print 'netvlad_desc.shape=', netvlad_desc.shape , '\tnetvlad_at_i.shape', netvlad_at_i.shape



        #
        # Find candidate matches for each i amongst (0 to i-1). Accept candidate
        #   only if i-1, i-2 and i go to a similar neighbourhood (filter_candidates).
        #       T = [ {a<-->b, score}, {a<-->b, score},...  ] #list of raw loop candidates with their scores
        #
        from Plot2Mat import Plot2Mat
        plot_handle = Plot2Mat()

        D = netvlad_desc
        T = []
        flag_show_image = True
        for i in range( netvlad_desc.shape[0] ):
                if i < 150: #don't lookup for first few frames
                    continue

                if i %500 == 0:
                    print ' < D[0:%d], D[i] > of %d' %(i, netvlad_desc.shape[0])

                DOT = np.dot( D[0:i-145,:], D[i,:] ) # compare D_live[i] will all the memory
                score  = np.max(DOT)
                argmax = np.argmax( DOT )
                #print 'Nearest neighobour of %d of live in db is %d (dotprodt = %4.4f)' %( netvlad_at_i[i], netvlad_at_i[argmax], score )

                T.append( (netvlad_at_i[i], netvlad_at_i[argmax], score) )

                if flag_show_image:
                    plot_handle.create()
                    cv2.imshow( 'plot', plot_handle.plot( DOT ).astype('uint8') )

                    __im = cv2.imread( BASE+'%d.jpg' %(netvlad_at_i[i]) )
                    cv2.imshow( '__im', cv2.resize(__im, (0,0), fx=0.5, fy=0.5 ) )


                    key = cv2.waitKey(20)
                    if key == ord('q'):
                        print 'Will do the do products but not display images; doing the dot products now, have patience.'
                        flag_show_image = False
                        cv2.destroyWindow( '__im' )
                        cv2.destroyWindow( 'plot')
        # cv2.destroyWindow( 'plot' )
        # This is a locality and threshold filtering.
        S = filter_candidates( T, TH=0., locality=8 )

    else:
        # Load the candidates from json file
        loopcandidate_json_fname = BASE+'/loopcandidates_ibow_lcd.json'
        # loopcandidate_json_fname = BASE+'/loopcandidates_dbow.json'
        # loopcandidate_json_fname = BASE+'/loopcandidates_liverun.json'
        # loopcandidate_json_fname = BASE+'/loopcandidates_manually_marked.json'
        DESCRIPTOR_STR = 'loopcandidate_json_fname='+'/'.join(loopcandidate_json_fname.split('/')[-2:])

        print 'LOAD file: ', loopcandidate_json_fname
        with open(loopcandidate_json_fname) as f:
            loopcandidate__data = json.load(f)

        T = []
        for l in loopcandidate__data:
            T.append(  ( l['global_a'], l['global_b'], l['inliers'] ) )
            # T.append(  ( l['global_a'], l['global_b'], l['score'] ) )
        # code.interact( local=locals() )
        # quit()






    #
    # Publish VIO__w_t_i and loop candidates on trajectory
    #   while not rospy.is_shutdown():
    #     publish_marker( pub, VIO__w_t_i, T, TH=0.92 )


    rate = rospy.Rate(10) # 10hz
    TH=0.92
    TH_step = 0.005
    list_of_scores = [ g[2] for g in T ]
    do_filter_candidates = 0
    do_filter_candidates_msg = [ 'filter by locality and if score greater than thresh', 'retain if greater than thresh', 'retain if less than thresh', 'retain all. <a>, <z> will have no effect' ]
    do_filter_candidates_len = len(do_filter_candidates_msg)
    print_msg = True
    while not rospy.is_shutdown():
        if do_filter_candidates%do_filter_candidates_len == 0:
            S = filter_candidates( T, TH=TH, locality=25 )
        if do_filter_candidates%do_filter_candidates_len == 1:
            S = filter_candidates_gt_thresh( T, TH=TH )
        if do_filter_candidates%do_filter_candidates_len == 2:
            S = filter_candidates_lt_thresh( T, TH=TH )
        if do_filter_candidates%do_filter_candidates_len == 3:
            S = T

        publish_marker( pub, VIO__w_t_i, S, TH=TH )
        if print_msg:
            print '---\n(higher score ==> higher confidence of match)'
            print 'list_of_scores: min=', min(list_of_scores), 'max=',max( list_of_scores)
            print 'press\n\
                    <a> to increment threshold by %4.6f.<z> to decrement.\n\
                    <s> to view current loop-candidates and write list as csv file.\n\
                    <p> play like a video\n\
                    <e> compare current candidates with manual annotations\n\
                    <t> compare current candidates with manual annotations\n\
                    <f> next filtering mode\n\
                    <j> double the TH_step\n\
                    <k> half the TH_step\n\
                    <r> TH_step=0.005\n\
                    <q> to quit.' %(TH_step)
            print_msg = False

        IM = np.zeros( (190,450)).astype('uint8')
        cv2.putText(IM,'DATASET=%s' %( BASE.split('/')[-2] ), (10,15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)
        cv2.putText(IM,'nAccepted=%d' %( len(S) ), (10,35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)
        cv2.putText(IM,'TH=%4.6f' %(TH), (10,65), cv2.FONT_HERSHEY_SIMPLEX, 1, 255)
        cv2.putText(IM,'max=%4.6f' %(max(list_of_scores)), (10,85), cv2.FONT_HERSHEY_SIMPLEX, .5, 255)
        cv2.putText(IM,'min=%4.6f' %(min(list_of_scores)), (10,105), cv2.FONT_HERSHEY_SIMPLEX, .5, 255)
        cv2.putText(IM,'do_filter_candidates=%d' %(do_filter_candidates), (10,125), cv2.FONT_HERSHEY_SIMPLEX, .5, 255)
        cv2.putText(IM,'%s' %(do_filter_candidates_msg[do_filter_candidates]), (10,145), cv2.FONT_HERSHEY_SIMPLEX, .5, 255)
        cv2.putText(IM,'=%s=' %( DESCRIPTOR_STR ), (10,165), cv2.FONT_HERSHEY_SIMPLEX, 0.3, 255)


        cv2.imshow( 'im', IM )
        key = cv2.waitKey(10)
        if key == ord( 'a' ):
            TH += TH_step
            print_msg = True
        if key == ord( 'z' ):
            TH -= TH_step
            print_msg = True
        if key == ord( 'q' ):
            break
        if key == ord( 's' ):
            imshow_loopcandidates( S, BASE=BASE, VIO__w_t_i=VIO__w_t_i, pub=pub )
            print_msg = True
        if key == ord( 'p' ):
            play_trajectory_with_loopcandidates( VIO__w_t_i, S, BASE=BASE, pub=pub, pub_frames=pub_frames, pub_loopcandidates=pub_loopcandidates )
            print_msg = True
        if key == ord( 'f' ):
            do_filter_candidates += 1
            if do_filter_candidates >= do_filter_candidates_len:
                do_filter_candidates = 0
            print_msg = True
        if key == ord( 'e' ): #Evaluate with manual annotations
            # This intended to be called when raw descriptors are available.
            # ie. not just the candidates but the descriptors were also available.
            from unit_tools import compare_with_manual
            # compare_with_manual( T, BASE+'/loopcandidates_manually_marked.json' )
            compare_with_manual( S, BASE+'/loopcandidates_manually_marked.json', BASE=None ) # will not imshow
        if key == ord( 't' ): #Evaluate with manual annotations
            # This intended to be called when raw descriptors are available.
            # ie. not just the candidates but the descriptors were also available.
            from unit_tools import compare_with_manual
            # compare_with_manual( T, BASE+'/loopcandidates_manually_marked.json' )
            compare_with_manual( S, BASE+'/loopcandidates_manually_marked.json', BASE=BASE )


        if key == ord( 'r' ):
            TH_step = 0.005
        if key == ord( 'j' ):
            TH_step /= 2.0
        if key == ord( 'k' ):
            TH_step *= 2.0
        # if key == ord( 'c' ):
            # compare_with_manual( T, dump_file_ptr=pr_file_ptr )
        # if key == ord( 'v'):
            # for g in np.linspace( list_of_scores[0], list_of_scores[-1], 50 ):
                # TH = g
                # compare_with_manual( T, dump_file_ptr=pr_file_ptr )
        rate.sleep()


    # Write the current candidates to file:
    print 'Write File containing loop candidates: ', BASE+'loop_candidates.txt'
    print 'nCandidates : ', len(T), '\tFiltered Candidates : ', len(S)
    with open(BASE+"loop_candidates.txt", "w") as text_file:
        text_file.write( '#curr,prev,score\n#TH=%f\n#generated with place_recog_analysis_tool on %s\n' %(TH, str(datetime.datetime.now())))
        for s in S:
            text_file.write( '%d, %d, %f\n' %(s[0], s[1], s[2]) )

    quit()
