import urllib.request
import cv2
import numpy as np
from threading import Thread
import time

from sklearn.preprocessing import normalize

urlR = 'http://10.3.141.100/capture'
urlL = 'http://10.3.141.101/capture'
fExit = False
fPause = False
retR = False
retL = False

def get_imgL(urlL):
	global fExit, fPause, retL, CamL
	while True:
		if not fPause:	
			imgRespL = urllib.request.urlopen(urlL)
			imgNpL = np.array(bytearray(imgRespL.read()),dtype=np.uint8)
			CamL = cv2.imdecode(imgNpL,-1)
			retL = True
			time.sleep(0.004)
		if fExit:
			break
	
def get_imgR(urlR):
	global fExit, fPause, retR, CamR
	while True:
		if not fPause:
			imgRespR = urllib.request.urlopen(urlR)
			imgNpR = np.array(bytearray(imgRespR.read()),dtype=np.uint8)
			CamR = cv2.imdecode(imgNpR,-1)
			retR = True
			time.sleep(0.004)
		if fExit:
			break		

threadL = Thread(target = get_imgL, args = (urlL, ))
threadR = Thread(target = get_imgR, args = (urlR, ))

CamL = np.zeros((480,640,3), dtype=np.uint8)
CamR = np.zeros((480,640,3), dtype=np.uint8)

cv_file = cv2.FileStorage("./data/stereo_rectify_maps.xml", cv2.FILE_STORAGE_READ)
Left_Stereo_Map_x = cv_file.getNode("Left_Stereo_Map_x").mat()
Left_Stereo_Map_y = cv_file.getNode("Left_Stereo_Map_y").mat()
Right_Stereo_Map_x = cv_file.getNode("Right_Stereo_Map_x").mat()
Right_Stereo_Map_y = cv_file.getNode("Right_Stereo_Map_y").mat()
cv_file.release()

window_size = 3
min_disp = 2
num_disp = 130-min_disp
stereo = cv2.StereoSGBM_create(minDisparity = min_disp,
	numDisparities = num_disp,
	blockSize = window_size,
	uniquenessRatio = 10,
	speckleWindowSize = 100,
	speckleRange = 32,
	disp12MaxDiff = 5,
	P1 = 8*3*window_size**2,
	P2 = 32*3*window_size**2)

stereoR=cv2.ximgproc.createRightMatcher(stereo)

lmbda = 80000
sigma = 1.8
visual_multiplier = 1.0
 
wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=stereo)
wls_filter.setLambda(lmbda)
wls_filter.setSigmaColor(sigma)

threadL.start()
threadR.start()

font = cv2.FONT_HERSHEY_SIMPLEX

while True:
	frameR = CamR
	frameL = CamL

	if retL and retR:
		fPause = True

		Left_nice= cv2.remap(frameL,Left_Stereo_Map_x,Left_Stereo_Map_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
		Right_nice= cv2.remap(frameR,Right_Stereo_Map_x,Right_Stereo_Map_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)

		grayR= cv2.cvtColor(Right_nice,cv2.COLOR_BGR2GRAY)
		grayL= cv2.cvtColor(Left_nice,cv2.COLOR_BGR2GRAY)
		
		grayR = cv2.resize(grayR, (320,240), interpolation = cv2.INTER_AREA)
		grayL = cv2.resize(grayL, (320,240), interpolation = cv2.INTER_AREA)

		disp= stereo.compute(grayL,grayR)
		dispL= disp
		dispR= stereoR.compute(grayR,grayL)
		dispL= np.int16(dispL)
		dispR= np.int16(dispR)

		filteredImg= wls_filter.filter(dispL,grayL,None,dispR)
		filteredImg = cv2.normalize(src=filteredImg, dst=filteredImg, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX);
		filteredImg = np.uint8(filteredImg)
		filt_Color= cv2.applyColorMap(filteredImg,cv2.COLORMAP_JET) 
		
		crop_img = filt_Color[64:175, 185:320]
		crop_img = cv2.flip(crop_img, 1)
		resized = cv2.resize(crop_img, (800,480), interpolation = cv2.INTER_AREA)
		
		cv2.putText(resized, 'Stereo Camera', (340, 20), font, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
		cv2.imshow('Stereo Camera',resized)

		pressedKey = cv2.waitKey(1) & 0xFF
		if pressedKey == ord('q'):
			fExit = True
			break
		
		retR = False
		retL = False
		fPause = False
	else:
		CamL= CamL
		CamR= CamR