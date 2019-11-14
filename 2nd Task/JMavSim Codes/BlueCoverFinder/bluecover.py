import numpy as np
import cv2

font                   = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (10,500)
fontScale              = 1
fontColor              = (255,255,255)

colorred              = (0,0,255)
colorgreen              = (0,255,0)
colorblue              = (255,0,0)
colorr              = (120,50,80)

lineType               = 2

fcamera = 458
realR = 32.4
distance = 0
pixelR = 1
cap = cv2.VideoCapture(0)
 
while(True):
    ret, frame = cap.read()
    flipf = cv2.flip( frame, 1)
    
    #sizey, sizex = flipf.size
    sizex= 320 #int(round(sizex/2))
    sizey= 240 #int(round(sizey/2))
    sizer = int(round(sizex/4))
        
   # cv2.imshow(' ',flipf)

    hsv = cv2.cvtColor(flipf, cv2.COLOR_BGR2HSV)
    
##    # lower mask (0-10)
##    lower_red = np.array([0,50,50])
##    upper_red = np.array([10,255,255])
##    mask0 = cv2.inRange(hsv, lower_red, upper_red)
##
##    # upper mask (170-180)
##    lower_red = np.array([170,50,50])
##    upper_red = np.array([180,255,255])
##    mask1 = cv2.inRange(hsv, lower_red, upper_red)
##
##    # join my masks
##    mask = mask0+mask1
    
    lb = np.array([90,80,60])
    ub = np.array([120,255,255])
    
    mask = cv2.inRange(hsv, lb, ub)
       
    kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
    mask_o = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel2)
    
    kernel = np.ones((5,5),np.float32)/(25)
    mask_blur = cv2.filter2D(mask_o,-1,kernel)

 
    cv2.imshow( mask_blur)
      
    
    circles = cv2.HoughCircles(mask_blur, cv2.HOUGH_GRADIENT, 0.9, 120, param1=75, param2=20, minRadius=7, maxRadius=150)
    if circles is not None:

        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            if i[2] == 0:
                distance = 1
            else:
                pixelR = i[2]
                distance = round((realR * fcamera / (2*pixelR)))
            
            cv2.circle(flipf,(i[0],i[1]),i[2],(0,255,0),2)
            cv2.circle(flipf,(i[0],i[1]),2,(0,0,255),3)
            cv2.putText(flipf, str(i[2]), (i[0],i[1]), font, fontScale, colorr, lineType)
            
            cv2.putText(flipf, str(distance)+' mm', (i[0]+20,i[1]+20), font, fontScale, fontColor, lineType)
        
            #in or out
            if (i[0]-sizex)**2+(i[1]-sizey)**2 <= (sizer-i[2])**2:
                inorout = 'in'
                inoutcolor =colorgreen
                
            else:
                inorout = 'out'
                inoutcolor = colorred
                
    
            cv2.putText(flipf, inorout, (sizex,sizey), font, fontScale, inoutcolor, lineType)
        flipfc = flipf
        flipfc = cv2.circle(flipfc,(sizex, sizey), sizer, (0,255,0), 0)   
        cv2.imshow("circles", flipfc)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
