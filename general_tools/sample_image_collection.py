#!/usr/bin/env python

import glob
import os
from random import randint


# Look at a folder which has stereo images and sample the set. Will copy
# cam0 and the corresponding cam1

INPUT_BASE = '/Bulk_Data/ros_bags/mynteye/calib/calib2' #don't put / at the end. ]

print 'mkdir ', INPUT_BASE+'_sampled'
os.makedirs( INPUT_BASE+'_sampled')

EVERY = 30
KEEP_PERCENT = 5

for i,fname in enumerate(glob.glob( INPUT_BASE+'/cam0_*' )):
	#print i, fname
	fname_split = fname.split( '/' )
	# if i>250 and i%EVERY == 0:
	if i>250 and randint(0,100) < 5:

		cmd = 'cp '+ fname+ ' '+ INPUT_BASE+'_sampled/'+fname_split[-1]
		print cmd
		os.system( cmd )
		u = fname_split[-1].split('_')
		cmd = 'cp '+ '/'.join(fname_split[0:-1])+'/cam1_'+u[1]+ ' '+ INPUT_BASE+'_sampled/cam1_'+u[1]
		# import code
		# code.interact( local=locals() )
		print cmd
		os.system( cmd )
