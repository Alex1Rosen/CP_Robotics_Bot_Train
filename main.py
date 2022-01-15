import time
import threading


import cv2 as cv
import numpy as np

state = "m" # cpv- computer vision, m- manual
# Camera Function
def cam_thread_func(thread_name):
    print("HI " + thread_name)
    cap = cv.VideoCapture(0)
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
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        # Display the resulting frame
        cv.imshow('frame', gray)
        if cv.waitKey(1) == ord('q'):
            break
    # When everything done, release the capture
    cap.release()
    cv.destroyAllWindows()

#Main
def main():
    if(state == "m"):
        print("Manual Mode Looping")

    elif(state == "cpv"):
        print("Manual Mode Looping")

    else:
        print("No Control Loop Found")

# Called by input to mention motor thread
def movement_actualizer():
    print("Actua")

def motor_handler():
    l_speed

    print("I like to move it move it")




if __name__ == "__main__":
    l_speed = 0
    r_speed = 0
    cam_thread_obj= threading.Thread(target=cam_thread_func, args = ("thread_name_string",))
    main_thread_obj = threading.Thread(target=main, args = ())
    motor_handler_obj = threading.Thread(target=motor_handler, args = ())
    
    # starting thread 1
    cam_thread_obj.start()
    # starting thread 2
    main_thread_obj.start()
    motor_handler_obj.start()

    # Wait until main thread is done
    main_thread_obj.join()
