# **Finding Lane Lines on the Road** 

The goals / steps of this project are the following:
* Make a pipeline that finds lane lines on the road
* Reflect on your work in a written report

---

### Reflection

### 1. Description of the pipeline.

My pipeline consisted of 6 steps. 

* read and grayscale the image
* gaussian smoothing
* canny to detect edges
* Creating a mask
* Hough Transformation
* Display lines

I created a function draw_line_on_image that outputs an image with the drawing of identified lines

The main modifications I made are in the function draw_lines:
* the left lane lines were identified by the negative slope and the right lane lines by the positive slope
* then, I extrapolated the points at the bottom and the top the region of interests and stored them
* finally, I averaged these points

I also exluded some noise in the data by excludoing data points where x1=x2 or y1 = y2 and also I limited the slope to values within [-0.45;0.45] 

### 2. Potential shortcomings
* only works for 960 * 540 resolution
* would not work for sharp turns (abs(slope) > 0.45)
* does not adjust the region of interests according to speed and horizon
* would not work if it was snowy, rainy
* not sure it would work at night

### 3. Possible improvements
* Change parameters according to the speed
* Compare series of lines to exclude significant changes of direction
* Include weather data / rain sensors / time data to adjust the model
