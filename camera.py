class Camera():

    """Summary
    """

    def __init__(self, x_res=192, y_res=144):
        """Summary
        
        Args:
            x_res (int, optional): Description
            y_res (int, optional): Description
        """
        self.x_res = x_res
		self.y_res = y_res

	def camera_settings():
	    """Sets settings for both cameras
	    """
	    CAMLED = 40
		GPIO.setup(CAMLED, GPIO.OUT)

		camera = PiCamera()
		camera.resolution = (self.x_res, self.y_res)
		camera.framerate = 60
		camera.color_effects = (128, 128) # black and white
		camera.awb_mode = 'off'
		camera.awb_gains = (1, 1)
		camera.iso = 100
		camera.brightness = 25
		camera.contrast = 100

	def img_capture(camera):
	    """Captures an image with one camera
	    
	    Args:
	        camera (string): left or right camera
	    """
	    img = np.empty((self.y_res, self.x_res, 3), dtype=np.uint8)

	    if camera == 'right':
		    GPIO.output(CAMLED, False) # Set to right cam

		else if camera == 'left':	
			GPIO.output(CAMLED, True) # Set to left cam

		else:
			print('camera error: select btw right and left camera')

		camera.capture(img, 'rgb', use_video_port=True)
		#camera.capture('right.jpg', use_video_port=True)

		return img
