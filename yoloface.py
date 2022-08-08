import serial
import time
import re

import argparse
import sys
import os
import numpy as np

from utils import *
##############################아두이노 세팅#########################
import serial       # serial, pyserial 필요
# Set a PORT Number & baud rate
#PORT = 'COM3'
#BaudRate = 57600

#ARD= serial.Serial(PORT,BaudRate)

#####################################################################
parser = argparse.ArgumentParser()
parser.add_argument('--model-cfg', type=str, default='./cfg/yolov3-face.cfg',
                    help='path to config file')
parser.add_argument('--model-weights', type=str,
                    default='./model-weights/yolov3-wider_16000.weights',
                    help='path to weights of model')
parser.add_argument('--image', type=str, default='',        #args.image로 접근 type: str
                    help='path to image file')
parser.add_argument('--video', type=str, default='',
                    help='path to video file')
parser.add_argument('--src', type=int, default=1,
                    help='source of the camera')
parser.add_argument('--output-dir', type=str, default='outputs/',
                    help='path to the output directory')
args = parser.parse_args()

#####################################################################
# print the arguments
print('----- info -----')
print('[i] The config file: ', args.model_cfg)
print('[i] The weights of model file: ', args.model_weights)
print('[i] Path to image file: ', args.image)
print('[i] Path to video file: ', args.video)
print('###########################################################\n')
print(args)
# check outputs directory
if not os.path.exists(args.output_dir):
    print('==> Creating the {} directory...'.format(args.output_dir))
    os.makedirs(args.output_dir)
else:
    print('==> Skipping create the {} directory...'.format(args.output_dir))

# Give the configuration and weight files for the model and load the network
# using them.
net = cv2.dnn.readNetFromDarknet(args.model_cfg, args.model_weights)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
py_serial = serial.Serial(
    
    # Window
    port='COM3',
    
    # 보드 레이트 (통신 속도)
    baudrate=115200, # 38400도 가능
)
wind_name = 'face detection using YOLOv3'
cv2.namedWindow(wind_name, cv2.WINDOW_NORMAL)

output_file = ''
#print(args.image)
if args.image:
    if not os.path.isfile(args.image):
        print("[!] ==> Input image file {} doesn't exist".format(args.image))
        sys.exit(1)
    cap = cv2.VideoCapture(args.image)
    cap = cv2.rotate(cap, cv2.ROTATE_90_COUNTERCLOCKWISE)
    output_file = args.image[:-4].rsplit('/')[-1] + '_yoloface.jpg'
elif args.video:
    if not os.path.isfile(args.video):
        print("[!] ==> Input video file {} doesn't exist".format(args.video))
        sys.exit(1)
    cap = cv2.VideoCapture(args.video)
    cap = cv2.rotate(cap, cv2.ROTATE_90_COUNTERCLOCKWISE)
    output_file = args.video[:-4].rsplit('/')[-1] + '_yoloface.avi'
else:
    # Get data from the camera
    cap = cv2.VideoCapture(args.src)
    # cap = cv2.flip(cap,1)
    # cap = cv2.rotate(np.array(cap).copy(), cv2.ROTATE_90_COUNTERCLOCKWISE)

# Get the video writer initialized to save the output video
if not args.image:
    video_writer = cv2.VideoWriter(os.path.join(args.output_dir, output_file),
                                    cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'),
                                    cap.get(cv2.CAP_PROP_FPS), (
                                        round(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                                        round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))))

while True:

    has_frame, frame = cap.read()
    frame = cv2.flip(frame,1)
    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

    # Stop the program if reached end of video
    if not has_frame:
        print('[i] ==> Done processing!!!')
        print('[i] ==> Output file is stored at', os.path.join(args.output_dir, output_file))
        cv2.waitKey(1000)
        break

    # Create a 4D blob from a frame.
    blob = cv2.dnn.blobFromImage(frame, 1 / 255, (IMG_WIDTH, IMG_HEIGHT),
                                    [0, 0, 0], 1, crop=False)

    # Sets the input to the network
    net.setInput(blob)

    # Runs the forward pass to get output of the output layers
    outs = net.forward(get_outputs_names(net))

    # Remove the bounding boxes with low confidence
    faces = post_process(frame, outs, CONF_THRESHOLD, NMS_THRESHOLD)
    print('[i] ==> # detected faces: {}'.format(len(faces)))
    print('#' * 60)

    # initialize the set of information we'll displaying on the frame
    info = [
        ('number of faces detected', '{}'.format(len(faces)))
    ]

    # for (i, (txt, val)) in enumerate(info):
    #     text = '{}: {}'.format(txt, val)
    #     cv2.putText(frame, text, (10, (i * 20) + 20),
    #                 cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_RED, 2)

#################################################################################################### 추가한 코드
    if centers_x != [] and centers_y != []:
        fw = frame.shape[1]
        fh = frame.shape[0]
        fw_center = fw // 2
        fh_center = fh // 2
        
        print([(x, y) for x, y in zip(centers_x, centers_y)])
        center_avg = get_center_avg()
        print(f'Average of Center: ({center_avg[0]}, {center_avg[1]})')

        '''
                    (0,0)           (1,0)
                            center
                    (0,1)           (1,1)
        '''
        #direction_x = 1 if fw_center < center_avg[0] else 0 # 1 if 우 else 0
        direction_y = 1 if fh_center < center_avg[1] else 0 # 1 if 하 else 0
        #print(str(direction_x) + str(direction_y))
        print(str(direction_y))

        msg = str(direction_y)
        commend = msg

        py_serial.write(commend.encode())

        time.sleep(0.1)
        
        if py_serial.readable():
            
            # 들어온 값이 있으면 값을 한 줄 읽음 (BYTE 단위로 받은 상태)
            # BYTE 단위로 받은 response 모습 : b'\xec\x97\x86\xec\x9d\x8c\r\n'
            response = py_serial.readline()
            # 디코딩 후, 출력 (가장 끝의 \n을 없애주기위해 슬라이싱 사용)
            response = (response[:len(response)-1].decode())
            print(response)
            info = [
            ('number of faces detected', '{}'.format(len(faces)), response, centers_x, centers_y)
            ]

            for (i, (txt, val, response_,centers_x_, centers_y_)) in enumerate(info):
                text = '{}: {}, camera degree: {}'.format(txt, val, response_)
                cv2.putText(frame, text, (10, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_BLUE, 2)
            for (i, (txt, val, response_,centers_x_, centers_y_)) in enumerate(info):
                text = 'object center: {},{}'.format(centers_x_, centers_y_)
                cv2.putText(frame, text, (10, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_BLUE, 2)
        msg = msg.encode('utf-8')
        #ARD.write(msg) # 아두이노로 전송
        #cv2.line(frame, (5,5), (5,5), COLOR_YELLOW, 5)
        cv2.line(frame, (center_avg[0], 0), (center_avg[0], fh), COLOR_BLUE, 1)
        cv2.line(frame, (0, center_avg[1]), (fw, center_avg[1]), COLOR_BLUE, 1)
        centers_x.clear()
        centers_y.clear()

#################################################################################################### 추가한 코드
    # Save the output video to file
    if args.image:
        cv2.imwrite(os.path.join(args.output_dir, output_file), frame.astype(np.uint8))
    else:
        video_writer.write(frame.astype(np.uint8))

    cv2.imshow(wind_name, frame)

    key = cv2.waitKey(1)
    if key == 27 or key == ord('q'):
        print('[i] ==> Interrupted by user!')
        break

cap.release()
cv2.destroyAllWindows()

print('==> All done!')
print('***********************************************************')
