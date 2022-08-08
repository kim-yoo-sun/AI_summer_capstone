import errno
import cv2
import os
import datetime
from PIL import Image
import time

def recording(cap, fps, codec, start_time, cnt, start_t):
    while (cap.isOpened()):

        ret, frame = cap.read()
        frame = cv2.flip(frame, 1)  # 화면 반전 0: 상하, 1: 좌우

        if ret == True:
            cv2.imshow('Frame Save', frame)

            end_time = datetime.datetime.now()
            diff = (end_time - start_time).seconds
            if time.time() - start_t >= 3:
                cv2.imwrite("./dataset/input%d.jpg" % cnt, frame)
                img = cv2.imread("./dataset/input%d.jpg" % cnt)
                print("saved image%d.jpg" % cnt)
                cnt += 1
                start_t = time.time()
            if diff > 186:  # 3분 동안 웹캠에서 프레임 추출
                break
        else:
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':

    try:
        if not (os.path.isdir('./dataset')):
            os.makedirs('./dataset')
    except OSError as e:
        if e.errno != errno.EEXIST:
            print("Failed to create directory!")
            raise

    cap = cv2.VideoCapture(0)
    fps = 11
    codec = cv2.VideoWriter_fourcc(*'DIVX')
    cnt = 1
    start_time = datetime.datetime.now()
    start_t = time.time()
    recording(cap, fps, codec, start_time, cnt, start_t)