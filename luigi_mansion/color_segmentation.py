import cv2
import numpy as np

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def nothing(x):
	pass

cv2.namedWindow("Trackbars")
cv2.setWindowProperty("Trackbars", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
#print("window made")

# Now create 6 trackbars that will control the lower and upper range of 
# H,S and V channels. The Arguments are like this: Name of trackbar, 
# window name, range,callback function. For Hue the range is 0-179 and
# for S,V its 0-255.
cv2.createTrackbar("L - H", "Trackbars", 0, 179, nothing)
cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("L - V", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("U - H", "Trackbars", 179, 179, nothing)
cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)

cv2.setTrackbarPos("L - H", "Trackbars", 0)
cv2.setTrackbarPos("L - S", "Trackbars", 100)
cv2.setTrackbarPos("L - V", "Trackbars", 234)
cv2.setTrackbarPos("U - H", "Trackbars", 19)
cv2.setTrackbarPos("U - S", "Trackbars", 255)
cv2.setTrackbarPos("U - V", "Trackbars", 255)


# Create a window named trackbars.
def cv():
	while True:

		l_h = cv2.getTrackbarPos("L - H", "Trackbars")
		l_s = cv2.getTrackbarPos("L - S", "Trackbars")
		l_v = cv2.getTrackbarPos("L - V", "Trackbars")
		u_h = cv2.getTrackbarPos("U - H", "Trackbars")
		u_s = cv2.getTrackbarPos("U - S", "Trackbars")
		u_v = cv2.getTrackbarPos("U - V", "Trackbars")

		lower_orange = np.array([l_h, l_s, l_v])
		high_orange = np.array([u_h, u_s, u_v])

		key = cv2.waitKey(1)
		if key == 27:
			break
		# cv2.waitKey(0)
		return(lower_orange, high_orange)


MIN_CONTOUR_AREA = 200




# def image_print(img):
# 	"""
# 	Helper function to print out images, for debugging. Pass them in as a list.
# 	Press any key to continue.
# 	"""
# 	cv2.imshow("image", img)
# 	cv2.waitKey(0)

def cd_color_segmentation(img,template):
	"""
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
	########## YOUR CODE STARTS HERE ##########
	# Convert the image to HSV
	img = img[150:, :, :]
	lower_orange = cv()[0]
	high_orange = cv()[1]
	
	blurred_image = cv2.GaussianBlur(img, (3,3), 0) 
	#blurred_image = cv2.dilate(blurred_image, (100,100))
	#blurred_image = cv2.erode(blurred_image, (,))
	
	#use cv2.inRange to apply a mask to the image
	image_hsv = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(image_hsv, lower_orange, high_orange)

	black_image = np.zeros_like(image_hsv, dtype=np.uint8)

    # Set the pixels within the orange range to white in the black image using the mask
	black_image[mask != 0] = [255, 255, 255]
	#image_print(black_image)


	masked_image = cv2.bitwise_and(image_hsv,image_hsv,mask=mask)

	_, thresholded_image = cv2.threshold(mask, 40, 255,0)
	kernel = np.ones((17, 17), np.uint8)
	thresholded_image = cv2.dilate(thresholded_image, kernel)
	#thresholded_image = cv2.erode(thresholded_image, kernel)
	edges = cv2.Canny(thresholded_image, 50, 150, apertureSize=3)
	circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, 1, 20, param1=60, param2=40, minRadius=0, maxRadius=0)

	if circles is not None:
		circles = np.uint16(np.around(circles))
		return (circles[0,:], thresholded_image)

	return ([],thresholded_image)