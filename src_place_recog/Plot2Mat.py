""" A pure python implementation to plot to image. Basic implementation """

import cv2
import numpy as np

import matplotlib
import code

class Plot2Mat:
    def __init__(self):
        self.width = 640
        self.height = 480
        self.im = None
        self.create()

    def xprint( self, msg  ):
        print '[Plot2Mat]', msg

    def create( self ):
        self.im = np.zeros( ( int(self.height*1.2), int(self.width*1.2), 3) ) + 80

    def mark( self, xm_array, color=(0,0,255) ):
        """ Give 1 iteration number (x-axis) to mark. This can be called after plot """
        # self.xprint( 'mark called'+str(xm_array.shape) )
        if self.im is None:
            return self.im
        # self.xprint( str(xm_array.shape) )
        for xm in xm_array:
            # self.xprint( str(xm) )
            xd = xm / self.L * self.width
            c = (  int(xd), int(self.height)  )
            self.im = cv2.circle( self.im, c, 7, color, -1  )

        cv2.putText(self.im, 'loop_items = %d' %(len(xm_array)), ( 10,40 ), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255,255,255), 3)
        return self.im



    def plot( self, y, line_color=(0,255,0) ):
        """ y is a 1-d array. Plot with indices. """
        im = self.im


        x = np.array( range( len(y) ) ) # [0 to N] ---> [ 0 to self.width ]
        L = float( len(x) )
        self.L = L

        # y [ min(y) to max(y) ] ---> [ 0 to self.height ]

        # code.interact( local=locals() )
        xd = x / ( L ) * (self.width)

        ymin = 0.0#y.min()
        ymax = 1.0#y.max()
        yd = (y - ymin) / (ymax - ymin) * (self.height)

        # Plot each point as circle
        for i in range( int(L) ):
            # print 'draw circle at ', xd[i], yd[i]
            c = (  int(xd[i]), int(-yd[i] + self.height)  ) # circle center
            # cv2.circle(im, c, 2, (0,0,255) )

            c_xlabel = (  int(xd[i]), int(0.0 + self.height)  )
            if i % int(L/5) == 0 :
                cv2.circle(im, c_xlabel, 4, (255,255,255), -1 )
                cv2.putText(im,str(i), (c_xlabel[0],c_xlabel[1]+50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255,255,255), 3)

                cv2.line( im, c_xlabel, (c_xlabel[0],0), (255,255,255), 2 )


        for i in range( 1, int(L) ):
            p1 = (  int(xd[i]), int(-yd[i] + self.height)  ) # circle center
            p2 = (  int(xd[i-1]), int(-yd[i-1] + self.height)  ) # circle center
            cv2.line( im, p1, p2, line_color, 3 )

            # cv2.line(  )


        for _yd in np.linspace( 0, 1, 11 ):
            #line (in image cord space): (0, _y) --- (self.width, _y)
            _y = (_yd) / (ymax - ymin) * self.height   + ymin
            _y = int(-_y+self.height)
            # print '(%d,%d) <---> (%d,%d)' %(0,_y, self.width, _y)
            cv2.line( im, (0,_y), (self.width, _y), (255,255,255),3)

            cv2.putText(im,str(_yd),(self.width+40, _y+10), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255), 3)

        self.im = im
        return im
