# USAGE
# python test_network.py --model santa_not_santa.model --image images/examples/santa_01.png

# import the necessary packages
from keras.preprocessing.image import img_to_array
from keras.models import load_model
import numpy as np
import argparse
import imutils
import cv2
import time

label = "woven1"
proba = 0

cap = cv2.VideoCapture(1)
model = load_model("outmodel.model")
while (1):
    start = time.time()
    _, image = cap.read()
    orig = image.copy()
    # pre-process the image for classification
    image = cv2.resize(image, (28, 28))
    image = image.astype("float") / 255.0
    image = img_to_array(image)
    image = np.expand_dims(image, axis=0)
     # classify the input image
    (leftpoint,line,noline,rightpoint) = model.predict(image)[0]

    # build the label
    if( leftpoint >line and leftpoint > noline and leftpoint > rightpoint):
        label = "leftpoint"
        proba = leftpoint

    elif ( line >leftpoint and line > noline and line > rightpoint ):
        label = "line"
        proba = line

    elif ( noline >leftpoint and noline > line and noline > rightpoint ):
        label = "noline"
        proba = noline
    
    elif ( rightpoint >leftpoint and rightpoint > line and rightpoint > noline ):
        label = "rightpoint"
        proba = rightpoint

    label = "{}: {:.2f}%".format(label, proba * 100)

    # draw the label on the image
    output = imutils.resize(orig, width=400)
    cv2.putText(output, label, (10, 25),  cv2.FONT_HERSHEY_SIMPLEX,
        0.7, (0, 255, 0), 2)

    end = time.time()
    print(str(end-start))

    # show the output image
    cv2.imshow("Output", output)
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break




cap.release()

cv2.destroyAllWindows() 





