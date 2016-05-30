%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	The Vision for Intellignet Vehicles and Applications (VIVA) Challenge
	Laboratory for Intelligent and Safe Automobiles
	University of California San Diego
	cvrr.ucsd.edu/vivachallenge/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

The hand detection benchmark contains two sets of images for the training and testing. 
The images were collected from the LISA testbeds and YouTube to assure diversity of vehicles, subjects, backgrounds, etc.

Data Format:
The format follows the annotation format supported by Piotr's Computer Vision Matlab (PMT) Toolbox (http://vision.ucsd.edu/~pdollar/toolbox/doc/).
Each image contains bounding boxs associated with it under the same name in the 'posGt' folder.
The bounding boxes are described using the top left point, a width, and a height [x y w h] in the 2D image plane.
Additionaly, each bounding box is also assigned a class out of 4, {leftHand_driver,rightHand_driver,leftHand_passenger,rightHand_passenger}.
We recommend using PMT functionalities (bbGt) for reading ground truth labels.  

Image Naming: each image is named according to the following.
videoID_framenumber_vehicletype_driversubjectID_passengersubjectID_viewpoint.png

- videoID: a unique ID number assigned to each video. Images from the same video will have the same videoID number.
- framenumber: The frame number (1-based) from the video.
- vehicletype: Either X, Q, or I for the LISA vehicles. 0 for youtube vehicles.
- driversubjectID: 0-8. If LISA vehicle, this is the subject ID of the driver from 1-8. Youtube videos have 0.
- passengersubjectID: 0-8. same as driversubjectID.
- viewpoint: 0-6. The camera view is marked by a number. 
	0-Unmounted, moving camera. The rest are mounted (besides first person view). 
    	1-Front left (by the A-pillar) looking at the driver. 
	2-Front right, looking at the driver, under the rear view mirror.
	3-Back, observing the driver's hands from the back by the sunroof.
	4-Side view of the right side of the driver.
	5-Top down view, mounted by the rear view mirror.
	6-First person view using a camera mounted on the driver's head.

Hands on the Wheel Benchmark:
The annotations can be found in 'NumberHandsOnWheelAnnotations.mat'. 
For each frame, we provide whether or not the left or right hand is touching the wheel. 
Uncertain instances are marked with 'uncertain'. 
Some instances are marked as 'unannotated' if hand is not in the view.

Results Submission:
The images for the test set are provided without labels. Please submit ONE TEXT FILE to us, where each row corresponds to a detection,
in the following format:

[imagename x y w h score left/right driver/passenger number_hands_on_wheel];
.
.
.

Any missing information can be set to -1 to be ignored
left/right is either 0/1 for left/right hand classification respectively. If your method does not output such information, set to -1.
driver/passenger is either 0/1 for driver/passenger hand classification respectively. If your method does not output such information, set to -1.
number_hands_on_wheel is an integer representing the number of hands on the wheel (0 hands, 1 hand, 2 hands). Set to -1 if method does not produce this classification.
