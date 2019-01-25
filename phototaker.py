import numpy as np
import cv2
import time
 

cap = cv2.VideoCapture(1) #cv2.VideoCapture(1)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320.0)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240.0)  

imcount = 0

saveimg = 1


while (1):
	_, img = cap.read()
	cv2.imshow('image',img)
	if saveimg == 1:
		imagename = "00" + str(imcount) +".jpg"
		cv2.imwrite(imagename,img)
		time.sleep(0.1)
		imcount = imcount+1
    	
	k = cv2.waitKey(30) & 0xff
	if k == 27:
		break

cap.release()

cv2.destroyAllWindows() 

