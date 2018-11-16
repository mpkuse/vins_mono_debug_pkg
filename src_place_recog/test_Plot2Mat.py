import numpy as np
import cv2

from Plot2Mat import Plot2Mat

y = np.random.random( 100 )
# y = np.zeros( 20 ) + .2
print y
obj = Plot2Mat()
im = obj.plot( y )
im = obj.mark( np.array( [10,20] ) )
cv2.imshow( 'im', im.astype('uint8') )
cv2.waitKey(0)
