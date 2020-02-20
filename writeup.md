## Advanced Lane Finding Project
---

**Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[checkerboard]: ./camera_cal/calibration1.jpg "Distorted"
[undistored_checkerboard]: ./camera_cal_undist/calibration1.jpg "Undistorted"
[undistorted]: ./output_images/test1_undistort.jpg "Road Transformed"
[binary]: ./output_images/test1_binary_threshold.jpg "Binary Example"
[annotated]: ./output_images/straight_lines2_annotated.jpg "Annotated Polygon"
[warped]: ./output_images/test1_warped_perspective.jpg "Warp Example"
[fit_lines]: ./output_images/test1_fit_lines.jpg "Fit Visual"
[result]: ./output_images/test1_final_result.jpg "Output"
[video1]: ./project_video.mp4 "Video"

## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Camera Calibration

The camera calibration was implemented in `compute_camera_matrix()` in `CarND-Advanced-Lane-Lines.ipynb`. The calibration input images consisted of a checkerboard pattern taken from different angles and also different positions within the frame.

The use of the checkerboard pattern enabled us to use OpenCV's `cv2.findChessboardCorners()` function. This function returns coordinates of the checkerboard pattern, and is given the size of the checkerboard which is determined beforehand.

For every calibration image, the corners are detected and appended to a list. Next, OpenCV's `cv2.calibrateCamera()` is used to compute the calibration matrix, which will be used to undistort camera images. The `cv2.calibrateCamera()` function will take the series of chessboard corner coordinates, which will be referred to as `imgpoints` as well as a series of "real world" coordinates called "object points" `objpoints`. Because all the calibration images correspond to the same set of object points, it can be precomputed and replicated for every image. The `objp` is the array of those coordinates, which is replicated in `objpoints` to satisfy the `cv2.calibrateCamera()` API.

The `cv2.undistort()` function can then be used to distort images taken from this same camera.

**Original Image**
![alt text][checkerboard]

**Undistorted Image**
![alt text][undistored_checkerboard]

### Pipeline (single images)

#### 1. Undistort the Image

Here's an example of an image that has been undistorted using `cv2.undistort()` and our camera calibration matrix.
![alt text][undistorted]

#### 2. Binary Threshold from Color and Contrast (Sobel) Gradient

##### Saturation Threshold
Next, a binary image is created from the undistorted image. Here is the binary output of the undistorted test image shown above. The image was converted from RGB to HLS color space. This color space contains hue, saturation, and lighting as orthogonal components. Through experimentation it was found that the S channel is very helpful in capturing lane lines independently of their particular hue and brightness because they tend to be high in saturation. The `hls_select()` method performs the binary thresholding. It supports thresholding on both the L and S channel, but the L channel proved to not be useful enough to use.

##### Sobel X Gradient Threshold
In addition to color, the Sobel operation was used to detect edges by taking the intensity derivative with respect to the X axis. The X axis was chosen because lane lines are much more vertical than they are horizontal. The Sobel thresholding was implemented in `abs_sobel_thresh()`. The function allows the user to specify either X or Y axis, the size of the Sobel kernel used, and the thresholds.

##### Combining Thresholds
With multiple binary images, they can be combined together with `and` or `or` operators. In this case, the `and` operator was used to combine the Sobel and saturation thresholds because it was desired for both conditions to be met prior to consideration.

##### Other Thresholds
Some other thresholds were considered but were ultimately not used in this implementation. There's likely a lot of room for optimization and performance gain with more thresholding techniques, but additional time would be necessary to make that determination.

Here are some other techniques that were considered:
* Sobel Y Gradient - Running the Sobel operation on the Y axis.
* Sobel Magnitude Threshold - Threshold of the magnitude of both the X and Y components of the Sobel operation
* Sobel Direction Threshold - Threshold on the direction of the 2D vector of X and Y components of the Sobel operation.


![alt text][binary]

#### 3. Perspective Transform

Next, the perspective transform was used to warp the image such that the lane lines appeared as if viewed from top down (bird's-eye-view). This transformation simplify our lane curvature and distance calculations because the lanes can be viewed as projected on a 2D plane.

A perspective transform can be done by selecting four points on the source image and their corresponding points on the destination image.

The perspective transform was performed by taking a test image with straight lanes and then selecting four points. The points were chosen using the lane width and the dashed lane lines as reference points. The goal was to select points with known distances between them, and due to regulations, the distances between the freeway lanes and the distance between the dashed markings were known to be 12 feet and 30 feet respectively.

![alt text][annotated]

The destination points were selected to make a rectangle, which will yield the desired bird's eye view. The rectangle was chosen to occupy the majority of the destination image, which implicitly crops the majority of the image, leaving basically the lane lines. The real world dimensions of this rectangle is known to be 12ft x 30ft, so the pixel to distance ratio, which will be constant given the warped view is directly along the Z axis, can be calculated. Some test images were used to confirm that the warped perspective appeared to be correct by visual inspection.

The `transform_perspective()` function implements the perspective transform on a given image.

Here's a snippet of the parameters used for perspective transform:
```python
# Perspective Transform Parameters
pt_src = np.float32(
    [[429, 566],
    [540, 490],
    [751, 490],
    [868, 566]])

dst_width = 800
dst_height = 600
origin_x = (1280 / 2) - (dst_width / 2)
origin_y = 720

pt_dst = np.float32(
    [[origin_x, origin_y],
    [origin_x, origin_y-dst_height],
    [origin_x+dst_width, origin_y-dst_height],
    [origin_x+dst_width, origin_y]]
)

# Camera image scaling parameters
xm_per_pix = (3.6576 / dst_width) # 12 foot lane width
ym_per_pix = (9.144 / dst_height) # 30 foot gap in between dashed lanes
```

![alt text][warped]

#### Best Fit Polynomial
Now that we have a bird's eye view binary image of the lane lines, we can attempt to fit a polynomial. This allows us to do a couple things:
1. Measure the curvature of the lane markings
1. Extrapolate the lane markings down to the bottom of the image, which allows for calculation of the center offset from the camera (and thus the vehicle given the camera is mounted in the center).

##### Histogram / Sliding Window Technique
To find the lane lines, a histogram-based technique was used. For a given column (x value) the number of activated pixels across the bottom half of rows were counted. The intuition is that the bottom half of the image is more reliable given that the objects in the bottom half are close to the vehicle. An assumption that the vehicle is more-or-less centered is made so that the histogram can be split into two haves. The peaks in each half were found and assumed to correspond to the X coordinate of the left and right lane lines.

If no significant peak was found, the full image is used instead of the bottom half as a fallback.

Next, a sliding window technique was used to shift a window slightly to the left or the right based on the average X coordinate of the activated pixels that fall within the window rectangle. The window then moves upwards after each iteration until the top of the image is reached.

After this process is complete, we have a set of discrete X and Y coordinates for each lane line. With this in hand, we use numpy's `np.polyfit()` to find the best-fit coefficients for a 2nd order polynomial. We first scale the X and Y coordinates from pixel-space to meters, so that we can calculate the curvature and center offsets in units of meters.

![alt text][fit_lines]

#### 5. Curvature and Center Position Calculation

The curvature was calculated as follows based on the equation for calculating curvature from a 2nd order polynomial:

```python
def get_curvature(y, fit_poly):
    A = fit_poly[0]
    B = fit_poly[1]
    R = ((1 + (2*A*y + B)**2)**(3/2)) / np.abs(2*A)
    return R
```

The center offset calculation was performed by using the best-fit polynomial to calculate X axis coordinates at the bottom of the frame for each lane. The midpoint of the two X coordinates was then used and compared to the midpoint of the frame. This was calculated in `calculate_center_offset()`

#### 6. Result Plot
The result was generated by unwarping the perspective and overlaying polygon generated.

![alt text][result]

---

### Pipeline (video)

Here's a [link to my video result](./project_video_output.mp4)

---

### Discussion

#### Problems Faced
One particular area that caused some problems was the color and gradient thresholding. The parameter space grew quite large between all the different binary thresholding techniques. It seemed intuitive that a Sobel magnitude and direction threshold would be a better choice over a simpler, absolute Sobel X axis threshold. However, after spinning in circles a bit, the magnitude and direction thresholding was abandoned. As mentioned before, I'd expect a lot of room for optimization in this area.

#### Areas of Improvement

##### Filtering / Smoothing
The biggest feature lacking currently is the inter-frame filtering that was suggested. A low pass filter generally be applied to all the values that are calculated on each individual frame. This will prevent spikes / outliers from individual frames to effect the output in any significant way.

##### Look-Ahead Filter
A look-ahead filter can be used to short circuit the histogram calculation used as part of the best fit calculation. The previous lane positions can be used as it is reasonable to assume the lane lines haven't moved much from one frame to the next.
