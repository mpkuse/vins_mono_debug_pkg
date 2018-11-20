import glob
import os

# Look at a folder which has stereo images and sample the set. Will copy
# cam0 and the corresponding cam1

for i,fname in enumerate(glob.glob( 'calib1/cam0_*' )):
	#print i, fname
	fname_split = fname.split( '/' )
	if i%15 == 0:
		cmd = 'cp '+ fname+ ' '+ 'calib1_sampled/'+fname_split[1]
		print cmd
		os.system( cmd )
		u = fname_split[1].split('_')
		cmd = 'cp '+ fname_split[0]+'/cam1_'+u[1]+ ' '+ 'calib1_sampled/cam1_'+u[1]
		print cmd
		os.system( cmd )
