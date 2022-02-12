import time
import threading

import numpy as np



# Camera Function
def cam_thread_func(shared_vars):
    ### IMPORT ###
    import cv2

    print("CAMERA THREAD LAUNCHING" )

    ### PARAMETERS ###
    # Set up the detector with default parameters
    params = cv2.SimpleBlobDetector_Params()
    # Change thresholds
    params.minThreshold = 0
    params.maxThreshold = 1000
    # Filter by Color
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

    ### INTIIALIZE ###
    detector = cv2.SimpleBlobDetector_create(params)
    cam_init = False
    while not cam_init:
        try:
            cap = cv2.VideoCapture(0)
            width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            print("CAMERA OPENED! ( " +  str(width) + " , " + str(height) + " )")
            cam_init = True

        except:
            print("Cannot open camera")

    ### THREAD FUNCTIONALITY ###


    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        # Blur
        im_gauss = cv2.GaussianBlur(frame, (9, 9), 0)
        # Convert to HSV
        frame_HSV = cv2.cvtColor(im_gauss, cv2.COLOR_BGR2HSV)
        # Mask Ball
        frame_threshold = cv2.inRange(frame_HSV, (0, 231, 79), (19, 255, 255))
        # Possibly redundant flatten
        mask_rgb = cv2.bitwise_not(cv2.cvtColor(frame_threshold, cv2.COLOR_GRAY2BGR))
        # Detect blobs
        keypoints = detector.detect(mask_rgb)

        # DEBUG: Draw Blobs on screen
        im_with_keypoints = cv2.drawKeypoints(mask_rgb, keypoints, np.array([]), (0, 0, 255),
                                              cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        # # DEBUG: draw lines
        # im_with_keypoints = cv2.line(im_with_keypoints, (int(width / 3), 0), (int(width / 3), int(height)),
        #                              (0, 255, 255))  # left
        # im_with_keypoints = cv2.line(im_with_keypoints, (int(2 * width / 3), 0), (int(2 * width / 3), int(height)),
        #                              (0, 255, 0))  # right
        # im_with_keypoints = cv2.line(im_with_keypoints, (0, int(3 * height / 4)), (int(width), int(3 * height / 4)),
        #                              (255, 255, 0))  # right
        # # DEBUG: Show keypoints
        # cv2.imshow('Hsv frame', frame_HSV)
        # # cv2.imshow('Blur Frame', im_gauss)
        # # cv2.imshow('Mask Frame', mask_rgb)
        cv2.imshow('Points', im_with_keypoints)
        # cv2.imshow('Color Frame', frame)

        # Grab single point from keypoints
        largest_size = 0
        largest_pt = [0,0]
        # Empty point
        best_p = cv2.KeyPoint(int(width/2),int(height/2),1)
        for p in keypoints:
            # filter by response or best point
            if(p.size > largest_size):
                best_p = p
                largest_pt = p.pt
                largest_size = p.size
                # print(str(largest_pt) + str(largest_size) )
                # print("Axis center: " + str(largest_pt[0] - width/2) )

        # calculate frame portion
        shared_vars[2] = best_p.pt
        # print("best point in cam thread: " + str(shared_vars[2].pt))

        # Handle Exit
        if cv2.waitKey(1) == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

# Main
def main(shared_varsq):
    while True:
        if (shared_vars[3] == "m"):
            print("Manual Mode Looping")
            print("best point in cam thread: " + str(shared_vars[2]))
            #

        elif (shared_vars[3] == "cpv"):
            print("Manual Mode Looping")

        else:
            print("No Control Loop Found")


# Called by input to mention motor thread
def movement_actualizer(shared_vars):
    print("Actual")


def motor_handler(shared_vars):
    print("I like to move it move it")


if __name__ == "__main__":
    # shared vars [ L, R, cam_pt, mode ]
    shared_vars = [0, 0, [0,0], "m"]
    # Threads
    cam_thread_obj = threading.Thread(target=cam_thread_func, args=(shared_vars,))
    main_thread_obj = threading.Thread(target=main, args=(shared_vars,))
    motor_handler_obj = threading.Thread(target=motor_handler, args=(shared_vars,))

    # starting thread 1
    cam_thread_obj.start()
    # starting thread 2
    main_thread_obj.start()
    motor_handler_obj.start()

    # Wait until main thread is done
    main_thread_obj.join()
