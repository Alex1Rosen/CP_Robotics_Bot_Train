import cv2
import numpy as np

cap = cv2.VideoCapture(0)  #
# Set up the detector with default parameters.
params = cv2.SimpleBlobDetector_Params()
# Change thresholds
params.minThreshold = 0
params.maxThreshold = 1000

params.filterByColor = True
params.blobColor = 0

# Filter by Area.
params.filterByArea = False
params.minArea = 50

# Filter by Circularity
params.filterByCircularity = 0
params.minCircularity = 0.75

# Filter by Convexity
params.filterByConvexity = False
params.minConvexity = 0.25

# Filter by Inertia
params.filterByInertia = False
params.minInertiaRatio = 0.7

detector = cv2.SimpleBlobDetector_create(params)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Our operations on the frame come here

    # imgray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    im_gauss = cv2.GaussianBlur(frame, (7,7), 0)

    frame_HSV = cv2.cvtColor(im_gauss, cv2.COLOR_BGR2HSV)
    frame_threshold = cv2.inRange(frame_HSV, (0, 157, 146), (42, 255, 255))

    mask_rgb = cv2.bitwise_not(cv2.cvtColor(frame_threshold, cv2.COLOR_GRAY2BGR))
    #blobs

    # Detecting blobs.
    keypoints = detector.detect(mask_rgb)

    im_with_keypoints = cv2.drawKeypoints(mask_rgb, keypoints, np.array([]), (0, 0, 255),
                                          cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # filter point further to find largest blobs

    # Show keypoints
    cv2.imshow('Hsv frame', frame_HSV)
    # cv2.imshow('Blur Frame', im_gauss)
    # cv2.imshow('Mask Frame', mask_rgb)
    cv2.imshow('Points', im_with_keypoints)
    cv2.imshow('Color Frame', frame)
    # calculate frame portion

    if cv2.waitKey(1) == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
