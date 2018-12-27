
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2
import code
import sys



def make_marker():
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "my_namespace"
    marker.id = 0

    marker.type = marker.SPHERE
    marker.action = marker.ADD

    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.2
    marker.scale.y = 0.5
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    return marker

def make_line_marker(w_t_p0, w_t_p1):
    marker = make_marker()
    marker.ns = 'hlt_loop_candidates_raw'#+offsettxt
    marker.type = marker.LINE_LIST
    marker.id = 0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.scale.x = 0.17
    marker.points = []
    marker.points.append( Point( w_t_p0[0], w_t_p0[1], w_t_p0[2] ) )
    marker.points.append( Point( w_t_p1[0], w_t_p1[1], w_t_p1[2] ) )
    return marker

# T : info on each pair like:  [  (i,j, score) , ... ]
# VIO__w_t_i: position of each node.
# max_n (optional): If this is None means publish all nodes and all candidates. If this is a number will publish upto that nodes. In this case will return image of last pair
def publish_marker( pub, VIO__w_t_i, T, TH=0.92, max_n=None, BASE=None  ):

    if max_n is None:
        up_to = len(VIO__w_t_i)
    else:
        up_to = max_n

    # Publish VIO
    m = make_marker()
    m.ns = 'vio'#+offsettxt
    m.type = m.LINE_STRIP
    m.scale.x = 0.05
    #print 'len(VIO__w_t_i) ', len(VIO__w_t_i), '   len(T) ', len(T), 'max_n ', str(up_to)
    for i, w_t_i in enumerate(VIO__w_t_i[:up_to] ):
        m.points.append( Point( w_t_i[0], w_t_i[1], w_t_i[2] ) )  #plot nodes at x,y,z
        # m.points.append( Point( w_t_i[0], w_t_i[2], i ) )           #plot nodes at x,z,frame#
    pub.publish( m )


    if True:
        # Publish VIO text
        m = make_marker()
        m.ns = 'vio_text'#+offsettxt
        m.type = m.TEXT_VIEW_FACING
        m.scale.z = 0.07
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        for i, w_t_i in enumerate(VIO__w_t_i[:up_to]):
            m.text = str(i)
            m.id = i
            m.pose.position.x =  w_t_i[0]
            m.pose.position.y =  w_t_i[1]
            m.pose.position.z =  w_t_i[2]
            pub.publish( m )


    # Plot loop candidates
    c = 0
    m = make_marker()
    m.ns = 'loop_candidates_raw'#+offsettxt
    m.type = m.LINE_LIST
    m.color.r = 1.0
    m.color.g = 0.0
    m.color.b = 0.0
    m.scale.x = 0.07
    selected_loop_candidates = []
    for i in range( len(T) ):
        p0 = int(T[i][0])
        p1 = int(T[i][1])
        score = T[i][2]

        # if score > TH and abs(T[i+1][1] - p1) < 8 and abs(T[i+2][1] - p1) < 8: # and T[i+1][2] > TH and T[i+2][2] > TH:
        if max_n is not None and (p0 >= up_to or p1 >= up_to):
            continue

        if max_n is None:
            # print '%d<--%4.2f-->%d' %( p0, score, p1 ), '\tAccept'
            pass

        w_t_p0 = VIO__w_t_i[ p0 ]
        w_t_p1 = VIO__w_t_i[ p1 ]
        m.points.append( Point( w_t_p0[0], w_t_p0[1], w_t_p0[2] ) )
        m.points.append( Point( w_t_p1[0], w_t_p1[1], w_t_p1[2] ) )

        # m.points.append( Point( w_t_p0[0], w_t_p0[2], p0) )
        # m.points.append( Point( w_t_p1[0], w_t_p1[2], p1 ) )

        selected_loop_candidates.append( (p0,p1) )

        c = c+1

    pub.publish( m )
    # print 'publish n_loop_candidates: ', c, 'TH=', TH


    # Display Images of the Last Candidate
    if (max_n is not None) and len(selected_loop_candidates) > 0:
        assert( BASE is not None )
        p0 = selected_loop_candidates[ -1 ][0]
        p1 = selected_loop_candidates[ -1 ][1]

        im0 = cv2.imread( BASE+'/%d.jpg' %(p0) )
        im1 = cv2.imread( BASE+'/%d.jpg' %(p1) )
        blank_space = np.ones( (im0.shape[0], 50, 3) )*255

        im = np.concatenate( (im0, blank_space.astype('uint8'), im1,), axis=1 )
        status = np.zeros( (120,im.shape[1],3)).astype('uint8')
        cv2.putText(status, '%d<   (%4.6f)   >%d' %( p0, score, p1 ), (10,65), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 2 )
        im = np.concatenate( (im,status), axis=0 )
        # cv2.imshow( 'candidate', im  )
        # cv2.waitKey(10)
        return im
    else:
        return None




# uses the global variable TH and KITTI_BASE.
# T : info on each pair like:  [  (i,j, score) , ... ]
# VIO__w_t_i: position of each node. list of 3d pts.
#   Show call the candidates in T. Hopefully they are filtered already, using the call to `filter_candidates`
def imshow_loopcandidates( T, BASE=None, VIO__w_t_i=None, pub=None ):
    assert( BASE is not None )



    # for i in range( 0,len(T)-3, 3 ):
    for i in range( 0, len(T) ):
        p0 = int(T[i][0])
        p1 = int(T[i][1])
        score = T[i][2]
        print '[imshow_loopcandidates:%d of %d]' %(i, len(T)), '%d<--(%4.2f)-->%d' %( p0, score, p1 )


        im0 = cv2.imread( BASE+'/%d.jpg' %(p0) )
        im1 = cv2.imread( BASE+'/%d.jpg' %(p1) )
        blank_space = np.ones( (im0.shape[0], 50, 3) )*255

        im = np.concatenate( (im0, blank_space.astype('uint8'), im1,), axis=1 )
        status = np.zeros( (120,im.shape[1],3)).astype('uint8')
        cv2.putText(status, '%d<   (%4.6f)   >%d' %( p0, score, p1 ), (10,65), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255) )
        im = np.concatenate( (im,status), axis=0 )

        # TODO Try Feature Matching and score computation on these 2 images

        # Publish marker to highlight this pair on trajectory
        if VIO__w_t_i is not None and pub is not None:
            marker = make_line_marker( VIO__w_t_i[p0], VIO__w_t_i[p1] )
            pub.publish( marker )


        # cv2.imshow( 'im', im )
        cv2.imshow( 'im_resized', cv2.resize( im, (0,0), fx=0.5, fy=0.5 ) ) # resize for better display
        print 'Press <space> for next pair, <q> to exit this loop. <w> to save this image to BASE (%s)' %(BASE)
        key = cv2.waitKey(0)
        if key == ord('q'):
            break
        if key == ord('w'):
            print 'Writing Image: ', BASE+'/loopcandidate_%d_%d.jpg' %(p0,p1)
            cv2.imwrite( BASE+'/loopcandidate_%d_%d.jpg' %(p0,p1), im )

    cv2.destroyWindow('im')
    cv2.destroyWindow('im_resized')



# Play. Increment 1 note at a time. 20Hz loop
# w_t_all: list of 4x4 matrix. Each 4x4 matrix is the pose.
# T      : loop candidates
def play_trajectory_with_loopcandidates( VIO__w_t_i, T, BASE=None, pub=None, pub_frames=None, pub_loopcandidates=None ):
    assert( BASE is not None and pub is not None and pub_frames is not None and pub_loopcandidates is not None )

    print 'play_trajectory_with_loopcandidates'

    m = make_marker()
    m.ns = 'vio'#+offsettxt
    m.type = m.LINE_STRIP
    m.scale.x = 0.05
    m.points = []
    m.id = 0


    rate = rospy.Rate(200)
    N = len(VIO__w_t_i)
    n = -1
    last_candidate_indx = 0
    while not rospy.is_shutdown():
        n += 1
        if n >= N:
            break
        print 'Play %d of %d' %(n,N)

        im_frame = cv2.imread( BASE+'/%d.jpg' %(n) )
        # cv2.imshow( 'frame-', im_frame )
        image_msg = CvBridge().cv2_to_imgmsg( im_frame )
        pub_frames.publish( image_msg )

        # im_loopcandidate = publish_marker( pub, VIO__w_t_i, T, max_n=n, BASE=BASE )
        m.points.append( Point( VIO__w_t_i[n][0], VIO__w_t_i[n][1], VIO__w_t_i[n][2] ) )
        pub.publish( m )


        for candidate_i  in range( last_candidate_indx, len(T) ):
            candidate = T[ candidate_i ]
            if candidate[0] > n:
                last_candidate_indx = candidate_i
                break
            p0 = int(candidate[0])
            p1 = int(candidate[1])
            score = candidate[2]

            # Make and publish Line Marker
            candidate_marker = make_line_marker( VIO__w_t_i[p0], VIO__w_t_i[p1] )
            candidate_marker.ns = 'loop_candidates_raw'
            candidate_marker.color.r = 1.0
            candidate_marker.color.g = 0.0
            candidate_marker.color.b = 0.0
            candidate_marker.scale.x = 0.07
            candidate_marker.id = candidate_i
            pub.publish(candidate_marker)


            # Load Those 2 image pair and publish
            im0 = cv2.imread( BASE+'/%d.jpg' %(p0) )
            im1 = cv2.imread( BASE+'/%d.jpg' %(p1) )
            blank_space = np.ones( (im0.shape[0], 50, 3) )*255

            im = np.concatenate( (im0, blank_space.astype('uint8'), im1,), axis=1 )
            status = np.zeros( (120,im.shape[1],3)).astype('uint8')
            cv2.putText(status, '%d<   (%4.6f)   >%d' %( p0, score, p1 ), (10,65), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 2 )
            im_loopcandidate = np.concatenate( (im,status), axis=0 )

            image_loopcandidate_msg = CvBridge().cv2_to_imgmsg( im_loopcandidate )
            pub_loopcandidates.publish( image_loopcandidate_msg )


        # if im_loopcandidate is not None:
            # image_loopcandidate_msg = CvBridge().cv2_to_imgmsg( im_loopcandidate )
            # pub_loopcandidates.publish( image_loopcandidate_msg )

        cv2.waitKey(1)
        rate.sleep()




# See if s exists in S
def find_candidate_in_set( s, S, lthresh=6, rthresh=6 ):
    for s_i in S:
        if ( abs(s[0] - s_i[0]) < lthresh and abs( s[1] - s_i[1]) < rthresh ) or ( abs(s[0] - s_i[1]) < rthresh and abs( s[1] - s_i[0]) < lthresh ):
            return True, s_i
    return False, None

# TODO: not in use removal
# See if s exists in S. Returns noccurences
# def find_candidate_hit_in_set( s, S, lthresh=6, rthresh=6 ):
#     n = 0
#     n_list = []
#     for s_i in S:
#         if ( abs(s[0] - s_i[0]) < lthresh and abs( s[1] - s_i[1]) < rthresh ) or ( abs(s[0] - s_i[1]) < rthresh and abs( s[1] - s_i[0]) < lthresh ):
#             #return True, s_i
#             n += 1
#             n_list.append( s_i )
#     return n, n_list

# Given the loop candidates, compare these with manual annotations
def compare_with_manual( T , manual_loop_candidates_json_fname, dump_file_ptr=None, BASE=None ):
    selected_loop_candidates = []



    import json
    manual_loop_candidates = []
    print 'Load Manual Annotations json: ', manual_loop_candidates_json_fname
    with open(manual_loop_candidates_json_fname) as f:
        loopcandidate_manual_data = json.load(f)

    manual_loop_candidates = []
    for u in loopcandidate_manual_data:
        print u
        manual_loop_candidates.append( ( u['global_a'], u['global_b'] ) )



    print 'input loop candidates = ', len(T)

    if False:
        TH = 0.92
        for i in range( 0,len(T)-3, 3 ):
            p0 = int(T[i][0])
            p1 = int(T[i][1])
            score = T[i][2]

            if score > TH and abs(T[i+1][1] - p1) < 12 and abs(T[i+2][1] - p1) < 12: # and T[i+1][2] > TH and T[i+2][2] > TH:
                # print '%d<--(%4.2f)-->%d' %( p0, score, p1 ), '\tAccept'
                selected_loop_candidates.append( (p0, p1) )

    else:
        selected_loop_candidates = T


    print 'len(selected_loop_candidates)=', len(selected_loop_candidates)
    # code.interact( local=locals() )
    # return

    # print 'manual_loop_candidates\n'
    # for m in manual_loop_candidates:
    #     print m
    #
    #
    # print 'selected_loop_candidates\n'
    # for m in selected_loop_candidates:
    #     print m
    # code.interact( local=locals() )


    # Step - 1
    # Loop thru `selected_loop_candidates`
    BASE1 = BASE
    print 'Step-1: Loop thru `selected_loop_candidates'
    a = 0
    for enum_s, s in enumerate(selected_loop_candidates):
        decision, found = find_candidate_in_set( s,  manual_loop_candidates, 20, 20 )
        status_str = 'find '+ str(s)+ 'in set {manual_candidates}? FOUND:'+ str( decision )+ str(found)
        print '[%d of %d]' %(enum_s, len(selected_loop_candidates)), status_str
        if decision > 0 :
            a += decision

        if BASE1 is not None: #imshow this pair
            __im1 = cv2.imread( BASE+'/%d.jpg' %(s[0]) )
            __im2 = cv2.imread( BASE+'/%d.jpg' %(s[1]) )

            __im1_im2 = cv2.resize( np.hstack((__im1,__im2)), (0,0), fx=0.5, fy=0.5 )
            __status_im = np.zeros( (50, __im1_im2.shape[1], 3), dtype='uint8' )
            if found:
                txt_color = (0,255,0)
            else:
                txt_color = (0,0,255)
            cv2.putText(__status_im,status_str, (10,15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, txt_color )
            cv2.putText(__status_im, "       ^^^^^of set {selected_loop_candidates}", (10,35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, txt_color )

            if found: # if this was found in also show the `found` im pair
                x__im1 = cv2.imread( BASE+'/%d.jpg' %(found[0]) )
                cv2.putText(x__im1, str(found[0]), (10,35), cv2.FONT_HERSHEY_SIMPLEX, 1.5, txt_color )
                x__im2 = cv2.imread( BASE+'/%d.jpg' %(found[1]) )
                cv2.putText(x__im2, str(found[1]), (10,35), cv2.FONT_HERSHEY_SIMPLEX, 1.5, txt_color )
                x__im1_im2 = cv2.resize( np.hstack((x__im1,x__im2)), (0,0), fx=0.5, fy=0.5 )
                cv2.putText(x__im1_im2, "step-1:this pair was marked by human", (10,65), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,255,255) )
                __status_im = np.vstack( (__status_im, x__im1_im2 ) )



            # cv2.imshow( '__im1_im2', __im1_im2 )
            cv2.imshow( 'selected_loop_candidates__im1_im2', np.vstack( (__im1_im2, __status_im) ) )
            print '[in compare with manual/step1] any key to continue'
            key = cv2.waitKey(0)
            if key == ord( 'q' ):
                cv2.destroyWindow( 'selected_loop_candidates__im1_im2')
                BASE1=None

    if BASE1 is not None:
        cv2.destroyWindow( 'selected_loop_candidates__im1_im2')



    # Step - 2
    # Loop thru `manual_loop_candidates`
    BASE2 = BASE
    print 'Step-2: Loop thru `manual_loop_candidates`'
    b = 0
    for enum_s,s in enumerate( manual_loop_candidates ):
        decision, found = find_candidate_in_set( s,  selected_loop_candidates, 120, 120 )
        status_str = 'find '+ str(s) + 'in set {selected_loop_candidates}? FOUND: '+ str(decision)+ str(found)
        print '[%d of %d]' %( enum_s, len( manual_loop_candidates) ), status_str
        if decision > 0:
            b += decision

        if BASE2 is not None: #imshow this pair
            __im1 = cv2.imread( BASE+'/%d.jpg' %(s[0]) )
            __im2 = cv2.imread( BASE+'/%d.jpg' %(s[1]) )

            __im1_im2 = cv2.resize( np.hstack((__im1,__im2)), (0,0), fx=0.5, fy=0.5 )
            cv2.putText(__im1_im2, "step-2:this pair was marked by human", (10,65), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,255,255) )
            __status_im = np.zeros( (50, __im1_im2.shape[1], 3), dtype='uint8' )
            if found:
                txt_color = (0,255,0)
            else:
                txt_color = (0,0,255)
            cv2.putText(__status_im,status_str, (10,15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, txt_color )
            cv2.putText(__status_im, "       ^^^^^of set {manual_loop_candidates}", (10,35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, txt_color )

            if found: # if this was found in also show the `found` im pair
                x__im1 = cv2.imread( BASE+'/%d.jpg' %(found[0]) )
                cv2.putText(x__im1, str(found[0]), (10,35), cv2.FONT_HERSHEY_SIMPLEX, 1.5, txt_color )
                x__im2 = cv2.imread( BASE+'/%d.jpg' %(found[1]) )
                cv2.putText(x__im2, str(found[1]), (10,35), cv2.FONT_HERSHEY_SIMPLEX, 1.5, txt_color )
                x__im1_im2 = cv2.resize( np.hstack((x__im1,x__im2)), (0,0), fx=0.5, fy=0.5 )
                __status_im = np.vstack( (__status_im, x__im1_im2 ) )

            # cv2.imshow( '__im1_im2', __im1_im2 )
            cv2.imshow( 'manual_loop_candidates__im1_im2', np.vstack( (__im1_im2, __status_im) ) )
            print '[in compare with manual/step2] any key to continue'
            key = cv2.waitKey(0)
            if key == ord('q'):
                cv2.destroyWindow( 'manual_loop_candidates__im1_im2')
                BASE2=None

    if BASE2 is not None:
        cv2.destroyWindow( 'manual_loop_candidates__im1_im2')


    print '# of selected found in manual = %d' %( a )
    print '# of manual found in selected = %d' %( b )
    print '# of selected = %d' %( len( selected_loop_candidates ) )
    print '# of manual = %d' %( len(manual_loop_candidates) )

    try:
        print '\nprecision = %4.4f' %( float(a) /  len( selected_loop_candidates ) )
        print 'recall    = %4.4f' %( float(b)/ len(manual_loop_candidates))
    except:
        pass

    IM = np.zeros( (150,450)).astype('uint8')
    cv2.putText(IM,'# of selected found in manual = %d' %(a) , (10,15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)
    cv2.putText(IM, '# of manual found in selected = %d' %( b ), (10,35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)
    cv2.putText(IM,'# of selected = %d' %( len( selected_loop_candidates ) ), (10,65), cv2.FONT_HERSHEY_SIMPLEX, .5, 255)
    cv2.putText(IM,'# of manual = %d' %( len(manual_loop_candidates) ), (10,85), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)
    try:
        cv2.putText(IM,'precision = %4.4f' %( float(a) / len( selected_loop_candidates ) ), (10,115), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)
        cv2.putText(IM,'recall    = %4.4f' %( float(b)/ len(manual_loop_candidates) ), (10,135), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)
    except:
        pass
    cv2.imshow( 'comaprison with manual', IM )


    # print 'press any key to end comparuson'
    # cv2.waitKey(0)
    # cv2.destroyWindow( 'comaprison with manual' )

    # if dump_file_ptr is not None:
        # dump_file_ptr.write( '%4.2f,%d,%d,%d,%d\n' %(TH, a,b,len( selected_loop_candidates ),len(manual_loop_candidates) ) )
    # code.interact( local=locals() )
