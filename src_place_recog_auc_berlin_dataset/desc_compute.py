import numpy as np
import cv2
import os.path
import code
import scipy.io
import random

# Needed to make ros-service call.
import rospy
from cv_bridge import CvBridge, CvBridgeError
from cerebro.srv import *
from Plot2Mat import Plot2Mat

DB_BASE = '/Bulk_Data/cv_datasets/mapillary_berlin_seq/'
# SEQ = 'berlin_A100'
# SEQ = 'berlin_halenseestrasse'
SEQ = 'berlin_kudamm'

#------------------- IO ----------------------#

def read_all_images() :
    # Read all
    imshow = False
    setA = []
    for i in range(1000):
        fname = DB_BASE+'/'+SEQ+'/'+'_1/'+'%03d.jpg' %(i)
        if not os.path.isfile(fname):
            break

        print 'READ ', fname
        im = cv2.imread( fname )
        setA.append( im )
        if imshow:
            print im.shape
            cv2.imshow( 'im', im )
            cv2.waitKey(0)

    setB = []
    for i in range(1000):
        fname = DB_BASE+'/'+SEQ+'/'+'_2/'+'%03d.jpg' %(i)
        if not os.path.isfile(fname):
            break

        print 'READ ', fname
        im = cv2.imread( fname )
        setB.append( im )
        if imshow:
            print im.shape
            cv2.imshow( 'im', im )
            cv2.waitKey(0)

    return setA, setB

def init_service_proxy():
    print 'waiting for ros-service : whole_image_descriptor_compute'
    rospy.wait_for_service( 'whole_image_descriptor_compute' )
    try:
        service_proxy = rospy.ServiceProxy( 'whole_image_descriptor_compute', WholeImageDescriptorCompute )
    except rospy.ServiceException, e:
        print 'failed', e
    print 'connected to ros-service'
    return service_proxy

def compute_whole_image_desc( service_proxy, imSet ):
    print 'compute_whole_image_desc. imSet.shape=', len(imSet)
    netvlad_desc = []
    for i in range( len(imSet) ):
        if i%40 == 0:
            print 'Done with ', i, ' of ', len(imSet)
        im = imSet[i]
        image_msg = CvBridge().cv2_to_imgmsg( im )
        rcvd_ = service_proxy( image_msg, 22 )
        netvlad_desc.append( rcvd_.desc )
    return np.array(netvlad_desc), rcvd_.model_type

def load_ground_truth_candidates():
    gt_ = scipy.io.loadmat( DB_BASE+'/'+SEQ+'/ground_truth.mat' )#['ground_truth']
    ground_truth_np = np.zeros( gt_['ground_truth'].shape , dtype='int32')
    ground_truth_np[:,0] = gt_['ground_truth'][:,0]- gt_['test_start']
    ground_truth_np[:,1] = gt_['ground_truth'][:,1]
    ground_truth = []
    for i in range( ground_truth_np.shape[0] ):
        ground_truth.append( ( ground_truth_np[i,0], ground_truth_np[i,1] ) )

    return ground_truth

#------------------ VIZ ------------------------#

def make_viz_image( n_setA, n_setB, candidates=None ):
    """
        candidates = [ (indx_A, indx_B),  ... ]
    """
    m = max( n_setA, n_setB )
    im = np.zeros( (150, m*13+40, 3 ), dtype='uint8' )
    cv2.putText(im,'n_setA=%d' %(n_setA), (5,90), cv2.FONT_HERSHEY_SIMPLEX, .5, (255,255,255) )
    cv2.putText(im,'n_setB=%d' %(n_setB), (5,110), cv2.FONT_HERSHEY_SIMPLEX, .5, (255,255,255) )



    for i in range( n_setA ):
        center = (20+i*13, 20)
        cv2.circle(im, center, 5, (0,0,255), thickness=1, lineType=8, shift=0)

        if i%5 == 0:
            if i>0:
                cv2.putText(im,str(i), center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255) )
            else:
                cv2.putText(im,'A', center, cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255) )


    for j in range( n_setB ):
        center = (20+j*13, 60)
        cv2.circle(im, center, 5, (0,255,0), thickness=1, lineType=8, shift=0)

        if j%5 == 0:
            if j>0:
                cv2.putText(im,str(j), center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255) )
            else:
                cv2.putText(im, 'B', center, cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255) )

    if candidates is None:
        return im

    cv2.putText(im,'candidates=%d' %(len(candidates)), (5,130), cv2.FONT_HERSHEY_SIMPLEX, .5, (255,255,255) )
    for candidate in candidates:
        i = candidate[0]
        j = candidate[1]

        center_i = (20+i*13, 20)
        center_j = (20+j*13, 60)


        cv2.line(im, center_i, center_j, (0,255,0), 2)



    return im


#------------------ EVAL ------------------------#
# See if s exists in S
def find_candidate_in_set( s, S, lthresh=6, rthresh=6 ):
    for s_i in S:
        if abs(s[0] - s_i[0]) < lthresh and abs(s[1] - s_i[1]) < rthresh:
        # if ( abs(s[0] - s_i[0]) < lthresh and abs( s[1] - s_i[1]) < rthresh ) or ( abs(s[0] - s_i[1]) < rthresh and abs( s[1] - s_i[0]) < lthresh ):
            return True, s_i
    return False, None


def evaluate_precision_recall( candidate_method, candidates_gt ):
    """ Given two sets of candidates get precision and recall """

    print 'see if candidates[i] \\in {candidates_gt}'
    a = 0
    for s in candidate_method: # `a` will be number of true positives
        verdict, _  = find_candidate_in_set( s, candidates_gt )
        print 'find', s, ' in candidates_gt', verdict
        if verdict:
            a+=1
    print '#True Positives = ', a
    print '#False Positives = ', len(candidate_method) - a
    print '|candidates|=', len(candidate_method)
    print '|candidates_gt|=', len(candidates_gt)

    # print '---'
    # print 'see if candidates_gt[i] \\in {candidate}'
    # b = 0
    # for s in candidates_gt:
    #     verdict, _ = find_candidate_in_set( s, candidate_method )
    #     print 'find', s, ' in candidate', verdict
    #     if verdict:
    #         b+=1
    # print '#True verdicts = ', b

    print 'Precision = %4.4f' %( float(a)/len(candidate_method) ) #Ok! verified
    # print 'Recall    = %4.4f' %( float(b)/len(candidates_gt))
    print 'Recall    = %4.4f' %( float(a)/len(candidates_gt)) # since there are no true negatives, |candidates_gt| == relavant samples


plot_handle = Plot2Mat()


if __name__ == '__1main__':
    setA, setB = read_all_images()
    service_proxy = init_service_proxy()

    netvlad_desc_A, model_typeA = compute_whole_image_desc( service_proxy, setA )
    netvlad_desc_B, model_typeB = compute_whole_image_desc( service_proxy, setB )

    print 'netvlad_desc_A.shape=', netvlad_desc_A.shape, '\n', 'netvlad_desc_B.shape=', netvlad_desc_B.shape
    npz_fnameA = DB_BASE+'/'+SEQ+'/'+ model_typeA+'_1.npz'
    npz_fnameB = DB_BASE+'/'+SEQ+'/'+ model_typeB+'_2.npz'
    print 'save ', npz_fnameA, '\t', npz_fnameB
    np.savez_compressed( npz_fnameA, netvlad_desc=netvlad_desc_A )
    np.savez_compressed( npz_fnameB, netvlad_desc=netvlad_desc_B )


def conditions( num, DOT, DOT_p1, DOT_m1 ):
    if num == 0:
        return DOT.max() > 0.0
    if num == 1:
        return DOT.max() > 0.1
    if num == 2:
        return DOT.max() > 0.6
    if num == 3:
        return DOT.max() > 0.64
    if num == 4:
        return DOT.max() > 0.67

    #---
    if num == 10:
        return DOT.max() > 0.0 and abs( DOT.argmax() - DOT_p1.argmax() )<8
    if num == 11:
        return DOT.max() > 0.1 and abs( DOT.argmax() - DOT_p1.argmax() )<8
    if num == 12:
        return DOT.max() > 0.6 and abs( DOT.argmax() - DOT_p1.argmax() )<8
    if num == 13:
        return DOT.max() > 0.64 and abs( DOT.argmax() - DOT_p1.argmax() )<8
    if num == 14:
        return DOT.max() > 0.67 and abs( DOT.argmax() - DOT_p1.argmax() )<8
    if num == 15:
        return DOT.max() > 0.0 and abs( DOT.argmax() - DOT_p1.argmax() )<6
    if num == 16:
        return DOT.max() > 0.1 and abs( DOT.argmax() - DOT_p1.argmax() )<6
    if num == 17:
        return DOT.max() > 0.6 and abs( DOT.argmax() - DOT_p1.argmax() )<6
    if num == 18:
        return DOT.max() > 0.64 and abs( DOT.argmax() - DOT_p1.argmax() )<6
    if num == 19:
        return DOT.max() > 0.67 and abs( DOT.argmax() - DOT_p1.argmax() )<6
    if num == 8:
        return DOT.max() > 0.1 and abs( DOT.argmax() - DOT_p1.argmax() )<3
    if num == 5:
        return DOT.max() > 0.6 and abs( DOT.argmax() - DOT_p1.argmax() )<3
    if num == 6:
        return DOT.max() > 0.65 and abs( DOT.argmax() - DOT_p1.argmax() )<3
    if num == 7:
        return DOT.max() > 0.70 and abs( DOT.argmax() - DOT_p1.argmax() )<3

    #---
    if num == 20:
        return DOT.max() > .0 and abs( DOT.argmax() - DOT_p1.argmax() )<3 and abs( DOT.argmax() - DOT_m1.argmax() )<3
    if num == 21:
        return DOT.max() > .1 and abs( DOT.argmax() - DOT_p1.argmax() )<3 and abs( DOT.argmax() - DOT_m1.argmax() )<3
    if num == 22:
        return DOT.max() > .6 and abs( DOT.argmax() - DOT_p1.argmax() )<3 and abs( DOT.argmax() - DOT_m1.argmax() )<3
    if num == 23:
        return DOT.max() > .64 and abs( DOT.argmax() - DOT_p1.argmax() )<3 and abs( DOT.argmax() - DOT_m1.argmax() )<3
    if num == 24:
        return DOT.max() > .67 and abs( DOT.argmax() - DOT_p1.argmax() )<3 and abs( DOT.argmax() - DOT_m1.argmax() )<3


    if num == 25:
        return DOT.max() > .0 and abs( DOT.argmax() - DOT_p1.argmax() )<6 and abs( DOT.argmax() - DOT_m1.argmax() )<6
    if num == 26:
        return DOT.max() > .1 and abs( DOT.argmax() - DOT_p1.argmax() )<6 and abs( DOT.argmax() - DOT_m1.argmax() )<6
    if num == 27:
        return DOT.max() > .6 and abs( DOT.argmax() - DOT_p1.argmax() )<6 and abs( DOT.argmax() - DOT_m1.argmax() )<6
    if num == 28:
        return DOT.max() > .64 and abs( DOT.argmax() - DOT_p1.argmax() )<6 and abs( DOT.argmax() - DOT_m1.argmax() )<6
    if num == 29:
        return DOT.max() > .67 and abs( DOT.argmax() - DOT_p1.argmax() )<6 and abs( DOT.argmax() - DOT_m1.argmax() )<6


    if num == 30:
        return DOT.max() > .0 and abs( DOT.argmax() - DOT_p1.argmax() )<8 and abs( DOT.argmax() - DOT_m1.argmax() )<8
    if num == 31:
        return DOT.max() > .1 and abs( DOT.argmax() - DOT_p1.argmax() )<8 and abs( DOT.argmax() - DOT_m1.argmax() )<8
    if num == 32:
        return DOT.max() > .6 and abs( DOT.argmax() - DOT_p1.argmax() )<8 and abs( DOT.argmax() - DOT_m1.argmax() )<8
    if num == 33:
        return DOT.max() > .64 and abs( DOT.argmax() - DOT_p1.argmax() )<8 and abs( DOT.argmax() - DOT_m1.argmax() )<8
    if num == 9:
        return DOT.max() > .67 and abs( DOT.argmax() - DOT_p1.argmax() )<8 and abs( DOT.argmax() - DOT_m1.argmax() )<8


    print 'you asked for non existant condition ', num
    quit()

if __name__ == '__main__':

    model_type = 'mobilenet_conv7_allpairloss'
    # model_type = 'relja_matlab_model'
    # model_type = 'block5_pool_k16_allpairloss'

    npz_fnameA = DB_BASE+'/'+SEQ+'/'+ model_type+'_1.npz'
    npz_fnameB = DB_BASE+'/'+SEQ+'/'+ model_type+'_2.npz'
    print 'Load ', npz_fnameA
    print 'Load ', npz_fnameB
    netvlad_desc_A = np.load( npz_fnameA )['netvlad_desc' ]
    netvlad_desc_B = np.load( npz_fnameB )['netvlad_desc' ]

    ground_truth = load_ground_truth_candidates()

    cond_idx = random.randint(0,33)

    if True:
        im = make_viz_image( netvlad_desc_A.shape[0], netvlad_desc_B.shape[0], ground_truth )
        cv2.imshow( 'viz_groundtruth', im )
        # cv2.waitKey(0)
        # quit()

    # loop thru B and find matches forall i, B[i],
    candidates = []
    for i in range( 1, netvlad_desc_B.shape[0]-1 ):
        DOT = np.matmul( netvlad_desc_A, netvlad_desc_B[i] )
        DOT_p1 = np.matmul( netvlad_desc_A, netvlad_desc_B[i+1] )
        DOT_m1 = np.matmul( netvlad_desc_A, netvlad_desc_B[i-1] )


        maxi = DOT.max()
        argmaxi = DOT.argmax()


        # if DOT.max() > 0.6:
        # if DOT.max() > 0.0 and abs( DOT.argmax() - DOT_p1.argmax() )<8:
        # if DOT.max() > .497 and abs( DOT.argmax() - DOT_p1.argmax() )<3 and abs( DOT.argmax() - DOT_m1.argmax() )<3 :

        if conditions( cond_idx, DOT, DOT_p1, DOT_m1 ):
            print 'candidate : ', i, argmaxi , 'score = ', maxi
            candidates.append( (argmaxi, i) )


        if False:
            plot_handle.create()
            plotted_im = plot_handle.plot( DOT ).astype('uint8')
            plotted_im = plot_handle.mark( [argmaxi] )
            cv2.imshow( 'plot', plotted_im.astype('uint8') )
            cv2.waitKey(20)



    im = make_viz_image( netvlad_desc_A.shape[0], netvlad_desc_B.shape[0], candidates )
    cv2.imshow( 'viz', im )

    evaluate_precision_recall( candidates, ground_truth )
    print 'cond_idx', cond_idx
    cv2.waitKey(200)
