# This script loads the whole image descriptors (computed by `place_recog_analysis_tool`).
# Then try using faiss into it. Use my docker image: mpkuse/kusevisionkit:ros-kinetic-vins-tf-faiss
#
# Author  : Manohar Kuse <mpkuse@connect.ust.hk>
# 29th Apr, 209
#

import numpy as np
from TerminalColors import bcolors
tcol = bcolors()
import faiss
import time

#--------------------------------------------------
if False:
    # load `pinhole_1loop_in_lab` to train an index
    BASE = '/Bulk_Data/_tmp_saved_seq/mynt_pinhole_1loop_in_lab/'
    fname = BASE+'/gray_conv6_K16__centeredinput.npz'
    DESCRIPTOR_STR = 'from '+fname

    print tcol.OKGREEN, 'Load ', fname, tcol.ENDC
    loaded = np.load(fname)
    netvlad_desc = loaded['netvlad_desc'].astype('float32')
    netvlad_at_i = loaded['netvlad_at_i']
    print 'netvlad_desc.shape=', netvlad_desc.shape , '\tnetvlad_at_i.shape', netvlad_at_i.shape


#--------------------------------------------------


BASE = '/Bulk_Data/_tmp/'
BASE = '/Bulk_Data/_tmp_saved_seq/mynt_mall0/'

fname = BASE+'/gray_conv6_K16__centeredinput.npz'
DESCRIPTOR_STR = 'from '+fname

print tcol.OKGREEN, 'Load ', fname, tcol.ENDC
loaded = np.load(fname)
netvlad_desc = loaded['netvlad_desc'].astype('float32')
netvlad_at_i = loaded['netvlad_at_i']
print 'netvlad_desc.shape=', netvlad_desc.shape , '\tnetvlad_at_i.shape', netvlad_at_i.shape

index = faiss.IndexFlatIP( 4096 )
# index.add( netvlad_desc[0:-200] )

for i in range(0, len(netvlad_desc) ):
# for i in range(0, 500 ):
    print tcol.HEADER, '---', i , tcol.ENDC

    if i-150 >= 0:
        start_t = time.time()
        print 'add netvlad_desc[', i-150, ']', '\t done in %4.4fms' %(1000.*(time.time() - start_t))
        index.add( np.expand_dims( netvlad_desc[i-150], 0 ) )

    if i<200:
        print 'i less than 200. not seen enough'
        continue




    start_t = time.time()
    print 'search(', i, ')'
    # D, I = index.search( np.expand_dims(netvlad_desc[i],0), 4 )
    D, I = index.search( netvlad_desc[i-3:i], 4 )
    print 'Elapsed time: %4.4fms' %(1000.* (time.time() - start_t) )
    print 'i=', i, '::::>\n', I
    print D
