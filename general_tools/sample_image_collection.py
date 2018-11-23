#!/usr/bin/env python

import glob
import os
from random import randint


# Look at a folder which has stereo images and sample the set. Will copy
# cam0 and the corresponding cam1

# INPUT_BASE = '/Bulk_Data/ros_bags/mynteye/calib/calib2' #don't put / at the end. ]
# INPUT_BASE = '/Bulk_Data/ros_bags/bluefox_stereo/calib/leveled_cam'
INPUT_BASE = '/Bulk_Data/ros_bags/bluefox_stereo/calib/right_titled'


OUTPUT_BASE = INPUT_BASE+'_sampled'
print 'mkdir ', OUTPUT_BASE
os.makedirs(  OUTPUT_BASE  )

EVERY = 30
KEEP_PERCENT = 10

e=0
cmd_list = []

for i,fname in enumerate(glob.glob( INPUT_BASE+'/cam0_*.png' )):
	#print i, fname
	fname_split = fname.split( '/' )
	# if i>250 and i%EVERY == 0:
	if i>250 and randint(0,100) < KEEP_PERCENT:

		cmd = 'cp '+ fname+ ' '+ INPUT_BASE+'_sampled/'+fname_split[-1]
		cmd = 'cp '+ fname+ ' '+ INPUT_BASE+'_sampled/'+'cam0_'+str(e)+'.png'
		print cmd
		cmd_list.append( cmd )
		os.system( cmd )
		u = fname_split[-1].split('_')
		cmd = 'cp '+ '/'.join(fname_split[0:-1])+'/cam1_'+u[1]+ ' '+ INPUT_BASE+'_sampled/cam1_'+u[1]
		cmd = 'cp '+ '/'.join(fname_split[0:-1])+'/cam1_'+u[1]+ ' '+ INPUT_BASE+'_sampled/cam1_'+str(e)+'.png'
		e+=1
		# import code
		# code.interact( local=locals() )
		print cmd
		cmd_list.append( cmd )
		os.system( cmd )

print 'Write Log to: ', OUTPUT_BASE+'/cmd_list.txt'
with open(OUTPUT_BASE+'/cmd_list.txt', 'w') as f:
    for item in cmd_list:
        f.write("%s\n" % item)
