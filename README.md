# CarND_Project04AdvancedLaneFinding
Solution for Udacity Self driving car nano degree fourth project: Advanced Lane Finding

---

# Advanced Lane Finding Project


[//]: # (Image References)

[RC]: https://www.real-world-physics-problems.com/images/curvilinear_motion_9.png
[CC1]: ./output_images/ExCalib.png
[CCO2]: ./output_images/ExCalibUndist.png
[LCS]: https://i.pinimg.com/originals/63/c8/9a/63c89aba0ed994edcfce462b2a4b2b6b.jpg
[WF1]: ./output_images/whitefilter1.png
[WF2]: ./output_images/whitefilter2.png
[YF1]: ./output_images/yellowFilter1.png
[YF2]: ./output_images/yellowFilter2.png
[CF1]: ./output_images/ColorFiltered1.png
[CF2]: ./output_images/ColorFiltered2.png
[SMT1]: ./output_images/SobelMagnitudeThreshold1.png
[SMT2]: ./output_images/SobelMagnitudeThreshold2.png
[PP1]: ./output_images/PreProcessedImage_Color_Grad_Spatial_Filtering1.png
[PP2]: ./output_images/PreProcessedImage_Color_Grad_Spatial_Filtering2.png
[PP3]: ./output_images/PreProcessedImage_Color_Grad_Spatial_Filtering3.png
[PP4]: ./output_images/PreProcessedImage_Color_Grad_Spatial_Filtering4.png
[W1]: ./output_images/Warpped1.png
[WH1]: ./output_images/Warpped1Hist.png
[W2]: ./output_images/Warpped2.png
[WH2]: ./output_images/Warpped2Hist.png
[W3]: ./output_images/Warpped3.png
[WH3]: ./output_images/Warpped3Hist.png
[W4]: ./output_images/Warpped4.png
[WH4]: ./output_images/Warpped4Hist.png
[L1]: ./output_images/Lane1.png
[L2]: ./output_images/Lane2.png
[L3]: ./output_images/Lane3.png
[L4]: ./output_images/Lane4.png
[OI1]: ./output_images/straight_lines2.png
[OI2]: ./output_images/test1.png
[OI3]: ./output_images/test2.png
[OI4]: ./output_images/test3.png
[OI5]: ./output_images/test6.png

---

# Reflection

Udacity Self Driving Car Nanodegree fourth project is to process frames in a given video clip to obtain:
  * Mark Car Lane.
  * Calculate road curvature.
  * Calculate car center offset from lane center.

Before discussing the solution to achieve this goal, it is important to understand the problem and how to actually achieve the needed output.

A) Mark Car Lane:
Lane lines have some distinct features that can be used to identify them:

  * They have a unique color compared to the rest of the environment (usually white or yellow).
  * Their position is relatively constant in an image from the driver perspective.
  * They have sharp, clear and near vertical edges.

Knowing this then it is clear that the tools that can be used are:
  
  * Color filtering and detection
  * Spatial filtering
  * Edge detection

B) Calculate road curvature:
For road curvature it can be calculated from the following equation:

![alt text][RC]

The lane lines can be approximated to 

f(y)= Ay^2 + By + C

So we only need to calculate the polynomial coefficients to be able to calculate the curvature.

C) Calculate car center offset from lane center:

---

# A) Mark Car Lane

I have built a 9 stage pipe line to achieve this.
  #### * Camera Calibration
  #### * Image distortion correction
  #### * Color Filtering
  #### * Gray scale conversion
  #### * Image blurring
  #### * Sobel Gradient thresholding
  #### * Spatial filtering
  #### * Perspective Correction
  #### * Lane Line Search

### A1) Camera Calibration:

Modern cameras use lens to collect light from the world and project on a film/sensor that stores this light and use it to form an image, this is quite efficient as lens can focus light from different sources to a single point called the lens focal length.

This however lead to light ray distortion at the lens edge as the light ray needs to be bent more aggressively to reach the focal point.
This distortion can lead to change in objects shape, proximity, size in the result image.

As such we need to correct for such distortion before operating on the camera images.

This step needs to be done once per camera as lens position and focal point is constant.

###### P.S. Actually in a noisy medium as in a car with road bumps, sharp turns, etc... it would be a good idea to perform camera calibration multiple times to ensure that the camera correction matrix is up to date but we can neglect this for the purpose of this project.

Camera calibration is done by measuring the distortion of a known shape, usually a chess board, and use this measurement to calculate a distortion correction matrix that then can be used to undistort any later image.

OpenCV library provide excellent support for camera calibration routines, Udacity has provided a sample of set of images from the camera that can be used for calibration.

![alt text][CC1]

I used 'cv2.findChessboardCorners' to find the chess board corners in the image and then calculate camera matrix using the function 'cv2.calibrateCamera'.

### A2) Image distortion correction:

Using the calculated distortion correction matrix from (A1) I am able to use OpenCV function 'cv2.undistort'

The resulting image is clear from distortion effects as can be seen below.

![alt text][CCO2]

---

### A3) Color Filtering

Building on the lane characteristics identified above, I decided to filter the lane lines by their color as it is the most easy way for eliminating non lane objects from the image first.

#### A3a) Color space transformation
There exists multiple color spaces each with advantages/disadvantages the most common of them is the RGB color space used by most modern monitors and TVs.

I have chosen to convert to LAB color space, I think this color space is suitable for the application because:
  * It has a separate lighting, L, axis, allowing for better handling for different lighting conditions.
  * High values in the B axis represent pure yellow color, allowing for easier separation of yellow components in an image.

below is a chart showing the color distribution in LAB color space.
![alt text][LCS]

P.S.: In python all values are re-mapped to be 0-255 range.

#### A3b) Filtering Bright white portions in an image.

![alt text][WF1]
![alt text][WF2]

#### A3c) Filtering yellow portions in an image.

![alt text][YF1]
![alt text][YF2]

#### A3d) Merging of the two filtered portions

P.S.: The filtering values are chosen by Trial and error.
This approach is not optimal because it doesn't take lighting conditions a better approach would be to perform histogram equalization , e.g. CLAHE, prior to applying filtering.

The images below provides an example of what the output of this step looks like:
![alt text][CF1]
![alt text][CF2]

The lane lines are extracted successfully but it is clear that there exists a lot of unwanted objects in the image.

---

### A4) Gray scale conversion

Gray scale images have only shades of gray no other color can be found in such images, this reduces the amount of information we can obtain from an image but significantly reduce the computational bandwidth we need to operate on those images, having obtained the needed information from stage 1 we can downscale the resulting filtered image to gray scale to reduce needed computations in following steps

---

### A5) Image blurring:

Image blurring is a useful tool to reduce the amount of details in an image by changing pixel values based on their neighbors.
The number of pixels affecting the pixel value is known as a kernel size, the larger the kernel the more aggressive the blurring effect would be and more smooth the output image would look like, this is a good preprocessing for the next stage to get rid of unnecessary details that would affect accuracy of our calculations later.

---

### A6) Sobel Gradient thresholding:

Building on the lane characteristics identified above, I decided to use Sobel operator to filter out the near vertical edges.

This was done by calculating the magnitude gradient in both X and Y directions and then filtering for thresholds with direction vector between 40 and 70 vectors.

![alt text][SMT1]
![alt text][SMT2]

---

### A7) Spatial filtering:

The last characteristic that can be used is spatial filtering this useful to remove false positive that lay outside the lane lines.

![alt text][PP1]
![alt text][PP2]
![alt text][PP3]
![alt text][PP4]

---

### A8) Perspective Correction:

Before applying our search algorithm to search for points on the lane lines, it is clear that due to our perspective, i.e. far objects appearing smaller in image, the lane lines, which are for sure parallel, are seen as intersecting lines, this needs to be corrected.

Again, OpenCV provide function 'getPerspectiveTransform' and 'warpPerspective' which can be used to perform the perspective correction

I tried at first to have a warping function that is not dependent on image, i.e. using SIFT algorithm, however SIFT has been moved from OpenCV implementation due to patenting issues.

I tried to be not dependent on image size. However it was not giving satisfactory results so I had to use a number of static points to achieving the warping results needed to achieve robust results.

![alt text][W1]
![alt text][W2]
![alt text][W3]
![alt text][W4]

---

### A9) Lane Line Search

To correctly find the lane lines, I decided to calculate histogram of the warped and thresholded image shown above, this gives me the location of lane line starting pixels as shown below:

![alt text][WH1]
![alt text][WH2]
![alt text][WH3]
![alt text][WH4]

After that I divide the image vertically to two halves where I search each half independently for lanes, i.e. left/right, using a simple sliding window algorithm where I keep following the lane line pts found in thresholded image, i.e. nonzeros, within a certain margin representing the lane measured in pixels and reject any outliers to increase accuracy.

If the position of the pixels begin to shift in a certain direction, I update the center of the window only if the lane shifting exceeds a predefined threshold number of pixels to avoid in correct measurements.

The output of this step is shown below:

![alt text][L1]
![alt text][L2]
![alt text][L3]
![alt text][L4]

---

By This point we have correctly identified the lane lines and we can begin in 

# B) Calculate road curvature:

As described above we only need to calculate the coefficients of the polynomial 

f(y)= Ay^2 + By + C

To be able to substitute values in 

![alt text][RC]

and calculate the road curvature, From above image we have calculated those coefficients but those coefficients we calculated are calculated in image space, i.e. pixels, and not in real world measurements, i.e. meters.

Using the provided input we can assume:
  * each 720 pixel in image represent 30 meters in  y direction, i.e. longitudinal.
  * each 700 pixel in image represent 3.7 meters in x direction, i.e. lateral.

With this information along with the equation mentioned above we are able to calculate the left and right lane curvature.

We can perform some sanity checks to make sure that the two curvatures are somewhat similar and reject frames in video that doesn't obey this rule.

After that The road curvature is simply calculated as the average of bot left and right lane curvatures.

# C) Calculate car center offset from lane center:

This is quite simple, given we have got the lane lines identified and assume the camera is mounted in the center of the car, we only need to find the lane line points nearest to the car, i.e. image bottom, and calculate the distance between those two points in pixel space, then convert it back to real world measurements in meters, using conversion parameters mentioned above.

Combining the three points above we obtain the following images:

![alt text][OI1]
![alt text][OI2]
![alt text][OI3]
![alt text][OI4]
![alt text][OI5]

---

# Video Handling

The above description is applied to each individual frame in the input video stream, however the video frames can be processed for another information that is implicitly encoded in it, Time.

We can apply a low pass filter on the detected lane lines, curvature and offset in each frame to smooth transitions between different roads and measurements.

This is done by storing those elements in a moving window queue and averaging the elements in those queue at each frame to calculate the smoothed values.

---
# Conclusion

Lane lines are identified correctly through complete video, lane lines and offset are calculated and seems reasonable with the given inputs.

