import caffe
#import math
import numpy as np
#from matplotlib import pyplot as plt, rcParams
#from matplotlib.font_manager import FontProperties
import cv2
#from sklearn.metrics import precision_recall_curve, auc
#from time import time
#import re
#from os import path, getcwd, listdir, makedirs
#import sys
#from matplotlib import rcParams

import code

from simplistic_experiment import load_alex_net, get_alexnet_descriptor
from simplistic_experiment import load_calc_net, get_calc_descriptor



alexnet = load_alex_net()
im_fl = cv2.imread( 'lena_color.png' )
alexnet_descriptor = get_alexnet_descriptor( alexnet , im_fl ) # shape=64K


# calcnet = load_calc_net()
# im_fl = cv2.imread( 'lena_color.png' )
# calc_descriptor = get_calc_descriptor( calcnet , im_fl ) #.shape : 1064,
