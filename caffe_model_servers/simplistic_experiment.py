""" Reads all images in live and memory and computes the calc descriptor for
each image. Additionally will also read conv3 of AlexNet
"""

import caffe
import math
import numpy as np
# from matplotlib import pyplot as plt, rcParams
# from matplotlib.font_manager import FontProperties
import cv2
# from sklearn.metrics import precision_recall_curve, auc
from time import time
# import re
# from os import path, getcwd, listdir, makedirs
# import sys
# from matplotlib import rcParams

import code

def load_LA_net():
    lanet_def_path='../../LA/model/deploy.prototxt'
    lanet_model_path='../../LA/model/ft1-small_37750.caffemodel'

    # caffe.set_mode_cpu()
    caffe.set_mode_gpu()
    caffe.set_device(0)
    lanet = caffe.Net(lanet_def_path,1,weights=lanet_model_path)
    return lanet

def load_calc_net(  ):
    ##
    ## Setting up caffe
    ##
    net_def_path='proto/deploy.prototxt'
    net_model_path='proto/calc.caffemodel'

    # caffe.set_mode_cpu()
    caffe.set_mode_gpu()
    caffe.set_device(0)
    net = caffe.Net(net_def_path,1,weights= net_model_path)
    return net

def load_alex_net():
    net_def_path='proto/bvlc_alexnet.prototxt'
    net_model_path='proto/bvlc_alexnet.caffemodel'

    # caffe.set_mode_cpu()
    caffe.set_mode_gpu()
    caffe.set_device(0)
    net = caffe.Net(net_def_path,1,weights=net_model_path)
    return net

def get_calc_descriptor( net, im1_fl, display_visual=False ):
    ##
    ## Hist Eq
    ##
    img_yuv = cv2.cvtColor(im1_fl, cv2.COLOR_BGR2YUV )
    img_yuv[:,:,0] = cv2.equalizeHist(img_yuv[:,:,0])
    im1_fl = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)

    ##
    ## Forward Pass
    ##
    im1 = cv2.resize(cv2.cvtColor(im1_fl, cv2.COLOR_BGR2GRAY), (160,120))

    transformer = caffe.io.Transformer({'X1':(1,1,120,160)})
    transformer.set_raw_scale('X1',1./255)
    net.blobs['X1'].data[...] = transformer.preprocess('X1', im1)
    net.forward()
    # relu13 = np.copy(net.blobs['relu3'].data[0,0,:,:])
    calc_descriptor1 = np.copy( net.blobs['descriptor'].data[0,:] ).astype('float64')
    calc_descriptor1 /= np.linalg.norm(calc_descriptor1)


    # if display_visual:
    #     plt.axis('off')
    #     plt.imshow(relu13)
    #     plt.show()

    return calc_descriptor1

def get_alexnet_descriptor( alexnet, im1_fl ):

    ##
    ## Forward Pass
    ##
    im1 = cv2.resize(im1_fl, (227,227))

    transformer = caffe.io.Transformer({'data':(1,3,227,227)})
    transformer.set_raw_scale('data',1./255)
    transformer.set_transpose('data',(2,0,1))
    alexnet.blobs['data'].data[...] = transformer.preprocess('data', im1)
    alexnet.forward()
    rraw = np.copy( alexnet.blobs['conv3'].data[0,:] )
    desc = rraw.astype('float64').flatten()
    desc /= np.linalg.norm(desc) #normalize
    return desc

def get_lanet_descriptor( lanet, im1_fl ):


    im1 = cv2.resize( im1_fl, (160,120) )

    transformer = caffe.io.Transformer({'QueryImage':(1,3,120,160)})
    transformer.set_raw_scale('QueryImage',1./255)
    transformer.set_transpose('QueryImage',(2,0,1))

    lanet.blobs['QueryImage'].data[...] = transformer.preprocess('QueryImage', im1)
    lanet.forward()

    rraw = np.copy( lanet.blobs['outi'].data[0,:] )

    desc = rraw.astype('float64').flatten()
    desc_normalized = desc / np.linalg.norm(desc) #normalize
    return desc, desc_normalized



# BASE = '/home/mpkuse/Downloads/loop_closure_evaluation/__seq__/CampusLoopDataset/'
# start_memory = 1
# end_memory = 101
# start_live = 1
# end_live = 101


# BASE = '/home/mpkuse/Downloads/loop_closure_evaluation/__seq__/GardensPointDataset/'
# start_memory = 0
# end_memory = 200
# start_live = 0
# end_live = 200


BASE = '/home/mpkuse/Downloads/loop_closure_evaluation/__seq__/UniConcourseDense/'
start_memory = 0
end_memory = 300
start_live = 0
end_live = 300




# LA
# if __name__ == "__main__":
def do_LA():
    la_net = load_LA_net()

    # Sample usage
    # im1_fl = cv2.imread( BASE+'/memory/'+'Image%03d.jpg' %(10) )
    # desc, desc_normalized = get_lanet_descriptor( la_net, im1_fl )



    # loop on memory
    desc_memory = []
    desc_normalized_memory = []
    for i in range(start_memory,end_memory):
        fname = BASE+'/memory/'+'Image%03d.jpg' %(i)
        print i, 'READ : ', fname
        im_fl = cv2.imread( fname )
        la_desc, la_desc_normalized = get_lanet_descriptor( la_net, im_fl )
        desc_memory.append( la_desc )
        desc_normalized_memory.append( la_desc_normalized )

        cv2.imshow( 'im', im_fl )
        cv2.waitKey(10)
    desc_memory = np.array( desc_memory ) # 100x128
    desc_normalized_memory = np.array( desc_normalized_memory ) # 100x128

    print 'Writing ', BASE+'/memory/lanet_descriptor.out'
    np.savetxt( BASE+'/memory/lanet_descriptor.out' , desc_memory )
    np.savetxt( BASE+'/memory/lanet_normalized_descriptor.out' , desc_normalized_memory )



    # loop on live
    desc_live = []
    desc_normalized_live = []
    for i in range(start_live,end_live):
        fname = BASE+'/live/'+'Image%03d.jpg' %(i)
        print i, 'READ : ', fname
        im_fl = cv2.imread( fname )
        la_desc, la_desc_normalized = get_lanet_descriptor( la_net, im_fl )
        desc_live.append( la_desc )
        desc_normalized_live.append( la_desc_normalized )

        cv2.imshow( 'im', im_fl )
        cv2.waitKey(10)
    desc_live = np.array( desc_live )
    desc_normalized_live = np.array( desc_normalized_live )

    print 'Writing ', BASE+'/live/lanet_descriptor.out'
    np.savetxt( BASE+'/live/lanet_descriptor.out' , desc_live )
    np.savetxt( BASE+'/live/lanet_normalized_descriptor.out' , desc_normalized_live )





# # Basic Usage
# if __name__=='__1main__':
#
#     net = load_calc_net()
#
#     im1_fl = cv2.imread( BASE+'/memory/'+'Image%03d.jpg' %(10) )
#     im2_fl = cv2.imread( BASE+'/live/'+'Image%03d.jpg' %(10) )
#
#     calc_descriptor1 = get_calc_descriptor( net, im1_fl )
#     calc_descriptor2 = get_calc_descriptor( net, im2_fl )
#
#     cv2.imshow( 'im1_fl', im1_fl )
#     cv2.imshow( 'im2_fl', im2_fl )
#     cv2.waitKey(0)
#
#
#     # Conv3
#     alexnet = load_alex_net()
#     im1_fl = cv2.imread( BASE+'/memory/'+'Image%03d.jpg' %(10) )
#     alexnet_descriptor = get_alexnet_descriptor( alexnet , im1_fl )


# On KITTI
# if __name__ == '__1main__':
def do_KITTI():
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


    N = 6
    KITTI_BASE = PARAMS[N][0]
    start_ = PARAMS[N][1]
    end_   = PARAMS[N][2]

    net = load_calc_net()
    alexnet = load_alex_net()
    la_net = load_LA_net()


    np.random.seed(0)
    M = np.random.normal( size=(1064, 64896) ) * 1064 / 64864 # for GRP for alexnet

    A = []
    B = []
    C = []
    for i in range( start_, end_+1 ):
        print 'READ ', KITTI_BASE+'/%06d.jpg' %(i)

        im1_fl = cv2.imread( KITTI_BASE+'/%06d.jpg' %(i) )
        cv2.imshow( 'im', im1_fl )
        cv2.waitKey(10)

        calc_descriptor = get_calc_descriptor( net, im1_fl )
        alexnet_descriptor = get_alexnet_descriptor( alexnet , im1_fl )
        la_desc, la_desc_normalized = get_lanet_descriptor( la_net, im1_fl )


        A.append( calc_descriptor )
        B.append( np.dot( M, alexnet_descriptor) )
        C.append( la_desc_normalized )


    print 'Writing ', KITTI_BASE+'/calc_descriptor.out'
    np.savetxt( KITTI_BASE+'/calc_descriptor.out' , np.array(A) )

    print 'Writing ', KITTI_BASE+'/alexnet_conv3_descriptor.out'
    np.savetxt( KITTI_BASE+'/alexnet_conv3_descriptor.out' , np.array(B) )

    print 'Writing ', KITTI_BASE+'/lanet_normalized_descriptor.out'
    np.savetxt( KITTI_BASE+'/lanet_normalized_descriptor.out' , np.array(C) )






# Calc descriptors
# if __name__=='__1main__':
def do_calc():
    net = load_calc_net()

    # loop on memory
    desc_memory = []
    for i in range(start_memory,end_memory):
        fname = BASE+'/memory/'+'Image%03d.jpg' %(i)
        print 'READ : ', fname
        im_fl = cv2.imread( fname )
        calc_descriptor = get_calc_descriptor( net, im_fl )
        desc_memory.append( calc_descriptor )

        cv2.imshow( 'im', im_fl )
        cv2.waitKey(10)
    desc_memory = np.array( desc_memory ) # 100x1064
    # desc_memory = desc_memory[:,0,:]
    np.savetxt( BASE+'/memory/calc_descriptor.out' , desc_memory )



    # loop on live
    desc_live = []
    for i in range(start_live,end_live):
        fname = BASE+'/live/'+'Image%03d.jpg' %(i)
        print 'READ : ', fname
        im_fl = cv2.imread( fname )
        calc_descriptor = get_calc_descriptor( net, im_fl )
        desc_live.append( calc_descriptor )

        cv2.imshow( 'im', im_fl )
        cv2.waitKey(10)
    desc_live = np.array( desc_live )
    # desc_live = desc_live[:,0,:]
    np.savetxt( BASE+'/live/calc_descriptor.out' , desc_live )
    print 'Writing ', BASE+'/live/calc_descriptor.out'


    for i in range(100):
        DOT = np.dot( desc_memory, desc_live[i,:] )
        print 'Nearest neighobour of %d of live in db is %d (dotprodt = %4.4f)' %( i, np.argmax( DOT ), np.max(DOT))



# Alex Net descriptors
# if __name__=='__1main__':
def do_alexnet():
    alexnet = load_alex_net()

    # loop on memory
    alexnet_desc_memory = []
    for i in range(start_memory,end_memory):
        fname = BASE+'/memory/'+'Image%03d.jpg' %(i)
        print 'READ : ', fname
        im_fl = cv2.imread( fname )
        alexnet_descriptor = get_alexnet_descriptor( alexnet , im_fl )
        alexnet_desc_memory.append( alexnet_descriptor )

        cv2.imshow( 'im', im_fl )
        cv2.waitKey(10)
    alexnet_desc_memory = np.array( alexnet_desc_memory ) # 100x1064
    # desc_memory = desc_memory[:,0,:]
    np.savetxt( BASE+'/memory/alexnet_conv3_descriptor.out' , alexnet_desc_memory )


    # loop on live
    alexnet_desc_live = []
    for i in range(start_live,end_live):
        fname = BASE+'/live/'+'Image%03d.jpg' %(i)
        print 'READ : ', fname
        im_fl = cv2.imread( fname )
        alexnet_descriptor = get_alexnet_descriptor( alexnet, im_fl )
        alexnet_desc_live.append( alexnet_descriptor )

        cv2.imshow( 'im', im_fl )
        cv2.waitKey(10)
    alexnet_desc_live = np.array( alexnet_desc_live )
    # desc_live = desc_live[:,0,:]
    np.savetxt( BASE+'/live/alexnet_conv3_descriptor.out' , alexnet_desc_live )
    print 'Writing ', BASE+'/live/alexnet_conv3_descriptor.out'


    for i, k in enumerate( range(start_live, end_live) ):
        DOT = np.dot( alexnet_desc_memory, alexnet_desc_live[i,:] )
        print 'Nearest neighobour of %d of live in db is %d (dotprodt = %4.4f)' %( k, np.argmax( DOT ), np.max(DOT) )



if __name__ == '__main__':
    do_KITTI()
