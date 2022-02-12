import cv2
import numpy as np

cap = cv2.VideoCapture(0)  #

# Set up the detector with default parameters.####################
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
params.filterByCircularity = True
params.minCircularity = 0.75

# Filter by Convexity
params.filterByConvexity = False
params.minConvexity = 0.25

# Filter by Inertia
params.filterByInertia = False
params.minInertiaRatio = 0.7

detector = cv2.SimpleBlobDetector_create(params)
# Params
width = -1
height = -1
if not cap.isOpened():
    print("Cannot open camera")
    exit()

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    # print(width, height) #640x480
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Our operations on the frame come here

    # imgray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    im_gauss = cv2.GaussianBlur(frame, (9,9), 0)

    frame_HSV = cv2.cvtColor(im_gauss, cv2.COLOR_BGR2HSV)
    frame_threshold = cv2.inRange(frame_HSV, (0, 231, 79), (19, 255, 255))

    mask_rgb = cv2.bitwise_not(cv2.cvtColor(frame_threshold, cv2.COLOR_GRAY2BGR))
    #blobs

    # Detecting blobs.
    keypoints = detector.detect(mask_rgb)

    im_with_keypoints = cv2.drawKeypoints(mask_rgb, keypoints, np.array([]), (0, 0, 255),
                                          cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # filter point further to find largest blobs
    #for pt in keypoints:

    # Filter by keypoint params
    # largest_size = 0
    # # pts = np.asarray([[p.pt[0], p.pt[1], p.size,] for p in keypoints])
    # for p in keypoints:
    #     # filter by response or best point
    #     if(p.size > largest_size):
    #         best_p = p
    #         largest_size = p.size

    # print("X: " + pts[] + " Y: " +pts.)
    # draw lines
    im_with_keypoints = cv2.line(im_with_keypoints, (int(width/3),0), (int(width/3),int(height)), (0, 255, 255)) # left
    im_with_keypoints = cv2.line(im_with_keypoints, (int(2*width/3),0), (int(2*width/3),int(height)), (0, 255, 0)) # right
    im_with_keypoints = cv2.line(im_with_keypoints, (0,int(3*height/4)), (int(width),int(3*height/4)), (255, 255, 0)) # right
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
