"""
A Python OpenCV implementation of the OpenCV C++ linear motion deblur algorithm found at:
https://docs.opencv.org/4.x/d1/dfd/tutorial_motion_deblur_filter.html
"""

import sys
import time
import cv2 as cv
import numpy as np


print(cv.__version__)

LEN = 125
THETA = 0
snr = 300


imgIn = cv.imread("Motion_Deblur_Tutorial.jpg", cv.IMREAD_GRAYSCALE)

if imgIn is None:
    sys.exit("Could not read the image.")

# the C code defines a Mat here, stuff like this isn't really doable in Python

# test for license plate
# imgIn = imgIn[140:180, 80:150]
cv.imshow("Input Image", imgIn)

# it needs to process even image only
img_rows, img_cols = np.shape(imgIn)
img_rows = img_rows & -2
img_cols = img_cols & -2
roi_test = imgIn[0:img_rows, 0:img_cols]

# Hw calculation (start)
# replace roi.size with img_rows and img_cols
## calcPSF(h, roi.size(), LEN, THETA) custom function
## calcWnrFilter(h, Hw, 1.0 / double(snr)) custom function
# Hw calculation (stop)

"""
        imgIn.convertTo(imgIn, CV_32F);
        edgetaper(imgIn, imgIn);
 
        // filtering (start)
        # for this part replace imgIn(roi) with roi
        filter2DFreq(imgIn(roi), imgOut, Hw);
        // filtering (stop)
 
        imgOut.convertTo(imgOut, CV_8U);
        normalize(imgOut, imgOut, 0, 255, NORM_MINMAX);
        imwrite("result.jpg", imgOut);
        return 0;
    }
"""

# note: the help function from the original c code was deleted


def calc_psf(height: int, width: int, psf_length: int, psf_angle: float) -> np.ndarray:
    """Creates a linear point spread function matrix of a specified length and angle
    Since in this scenario the camera or scene is moving, the same pixel takes part
    of its reading from different places in the physical space the camera is imaging.
    This psf can be calibrated to represent which pixels we can derive the probable
    value of a location in the image from.

    C++ code:
        Mat h(filterSize, CV_32F, Scalar(0));
        Point point(filterSize.width / 2, filterSize.height / 2);
        ellipse(h, point, Size(0, cvRound(float(len) / 2.0)),
            90.0 - theta, 0, 360, Scalar(255), FILLED);
        Scalar summa = sum(h);
        outputImg = h / summa[0];

    Args:
        height (int): number of pixels/rows up and down the image to be deblurred
        width (int): number of pixels/columns across the image to be deblurred
        psf_length (int): the distance across the image that the blurring reaches
        psf_angle (float): angle of the psf (in the direction of the motion)

    Returns:
        np.ndarray: psf matrix which has a total sum of 1
    """

    # make psf_matrix
    psf_matrix = np.zeros((height, width), dtype=np.single)
    # running psf_matrix = np.zeros((num_rows, num_cols),dtype=np.single)
    # reduces precision to a 32 bit float but may make it run faster

    # draw an ellipse at psf_angle with psf_length in the middle of the screen
    center = (width // 2, height // 2)  # possibly but probably not in wrong order
    # may want to change 0 to something else to account for lateral motion blur
    # or maybe even make a gradient on the edges of the point spread
    axes = (0, psf_length // 2)  # could probably just use floor division here
    cv.ellipse(psf_matrix, center, axes, 90 - psf_angle, 0, 360, 255, -1)
    return psf_matrix / np.sum(psf_matrix)


def fft_shift(input_img: np.ndarray) -> np.ndarray:
    """Helper method to swap opposing quadrants of the psf
    Swaps q1 with q3 and q2 with q4. This is a necessary preprocessing step for
    the discrete fourier transform. The code in this section was modified from:
    https://docs.opencv.org/3.4/d8/d01/tutorial_discrete_fourier_transform.html
    This site has python, java, and c++ versions of the code; the c++ motion
    deblur code derives its implementation directly from the c++ code here.

    C++ code:
        outputImg = inputImg.clone();
        int cx = outputImg.cols / 2;
        int cy = outputImg.rows / 2;
        Mat q0(outputImg, Rect(0, 0, cx, cy));
        Mat q1(outputImg, Rect(cx, 0, cx, cy));
        Mat q2(outputImg, Rect(0, cy, cx, cy));
        Mat q3(outputImg, Rect(cx, cy, cx, cy));
        Mat tmp;
        q0.copyTo(tmp);
        q3.copyTo(q0);
        tmp.copyTo(q3);
        q1.copyTo(tmp);
        q2.copyTo(q1);
        tmp.copyTo(q2);

    Args:
        input_img (np.ndarray): matrix to be rearranged

    Returns:
        np.ndarray: the rearranged matrix
    """
    # code modified from the webpage the motion deblur c++ code is based on:
    # https://docs.opencv.org/3.4/d8/d01/tutorial_discrete_fourier_transform.html
    img_rows, img_cols = input_img.shape
    output_img = np.empty_like(input_img)
    cx = int(img_rows / 2)
    cy = int(img_cols / 2)
    q0 = input_img[0:cx, 0:cy]  # Top-Left - Create a ROI per quadrant
    q1 = input_img[cx : cx + cx, 0:cy]  # Top-Right
    q2 = input_img[0:cx, cy : cy + cy]  # Bottom-Left
    q3 = input_img[cx : cx + cx, cy : cy + cy]  # Bottom-Right
    tmp = np.copy(q0)  # swap quadrants (Top-Left with Bottom-Right)
    output_img[0:cx, 0:cy] = q3
    output_img[cx : cx + cx, cy : cy + cy] = tmp
    tmp = np.copy(q1)  # swap quadrant (Top-Right with Bottom-Left)
    output_img[cx : cx + cx, 0:cy] = q2
    output_img[0:cx, cy : cy + cy] = tmp
    return output_img


def filter_2D_freq(input_img: np.ndarray, h: np.ndarray) -> np.ndarray:
    # todo: docstring and also testing
    # print("image size: ", input_img.size())
    planes = [
        np.copy(input_img),
        np.zeros(np.shape(input_img), dtype=np.float32),
    ]
    print("2dfreq planes shape: ", np.shape(planes))
    print("dtypes are", planes[0].dtype, planes[1].dtype)
    complex_i = cv.merge(planes)
    complex_i = cv.dft(complex_i, cv.DFT_SCALE)
    planes = cv.split(complex_i)

    planes_h = [
        np.copy(h),
        np.zeros(np.shape(h), dtype=np.single),
    ]
    complex_h = cv.merge(planes_h)
    complex_ih = cv.mulSpectrums(complex_i, complex_h, 0)
    complex_ih = cv.idft(complex_ih)
    planes = cv.split(complex_ih)
    return planes[0]

    """
    void filter2DFreq(const Mat& inputImg, Mat& outputImg, const Mat& H)
    {
        Mat planes[2] = { Mat_<float>(inputImg.clone()), Mat::zeros(inputImg.size(), CV_32F) };
        Mat complexI;
        merge(planes, 2, complexI);
        dft(complexI, complexI, DFT_SCALE);
 
        Mat planesH[2] = { Mat_<float>(H.clone()), Mat::zeros(H.size(), CV_32F) };
        Mat complexH;
        merge(planesH, 2, complexH);
        Mat complexIH;
        mulSpectrums(complexI, complexH, complexIH, 0);
 
        idft(complexIH, complexIH);
        split(complexIH, planes);
        outputImg = planes[0];
    }

    """


def calc_wnr_filter(input_h_psf: np.ndarray, nsr: float):
    # todo: docstring, also testing
    print("pre-fftshift size: ", np.shape(input_h_psf))
    h_psf_shifted = fft_shift(input_h_psf)
    print("post-fftshift size: ", np.shape(h_psf_shifted))
    planes = [
        np.copy(h_psf_shifted),
        np.zeros(np.shape(h_psf_shifted), dtype=np.single),
    ]
    print("fftshift planes size: ", np.shape(planes))
    complex_i = cv.merge(planes)
    complex_i = cv.dft(complex_i)
    planes = cv.split(complex_i)
    denom = nsr + np.square(np.abs(planes[0]))
    return np.divide(planes[0], denom)
    #    void calcWnrFilter(const Mat& input_h_PSF, Mat& output_G, double nsr)
    #    {
    #        Mat h_PSF_shifted;
    #        fftshift(input_h_PSF, h_PSF_shifted);
    #        Mat planes[2] = { Mat_<float>(h_PSF_shifted.clone()), Mat::zeros(h_PSF_shifted.size(), CV_32F) };
    #        Mat complexI;
    #        merge(planes, 2, complexI);
    #        dft(complexI, complexI);
    #        split(complexI, planes);
    #        Mat denom;
    #        pow(abs(planes[0]), 2, denom);
    #        denom += nsr;
    #        divide(planes[0], denom, output_G);
    #    }


def edge_taper(
    input_img: np.ndarray, gamma: float = 5.0, beta: float = 0.2
) -> np.ndarray:
    # todo: docstring
    ny, nx = input_img.shape
    # w1 = np.zeros((1, ny), dtype=np.single)
    # w2 = np.zeros((1, nx), dtype=np.single)
    # print("w1, w2 are of shape: ", np.shape(w1), np.shape(w2))
    # print("^^^^^^^^^^^")

    w1 = np.arange(-np.pi, np.pi, 2.0 * np.pi / ny)
    print(w1[-2])
    w1 = 0.5 * (
        np.tanh((w1 + gamma / 2) / beta) - np.tanh((w1 - gamma / 2) / beta)
    ).reshape((-1, 1))
    print("w1 has size ", np.shape(w1))

    w2 = np.arange(-np.pi, np.pi, 2.0 * np.pi / nx)  # make a column
    w2 = 0.5 * (
        np.tanh((w2 + gamma / 2) / beta) - np.tanh((w2 - gamma / 2) / beta)
    ).reshape((1, -1))
    print("w2 has size ", np.shape(w2))

    w = np.multiply(w2, w1)  # vector multiplication
    print(w[200:203, 10:13])
    print("w is shape ", np.shape(w))
    unconverted_tapered_img = np.multiply(input_img, w)
    tapered_img = np.uint8(unconverted_tapered_img)
    # elementwise multiplication
    # cv.imshow("unconverted tapered", unconverted_tapered_img)
    # cv.imshow("converted tapered", tapered_img)
    # time.sleep(3)
    print(tapered_img[200:203, 10:13])
    print("tapered_image has shape ", np.shape(tapered_img))
    return tapered_img

    """
    void edgetaper(const Mat& inputImg, Mat& outputImg, double gamma, double beta)
    {
        int Nx = inputImg.cols;
        int Ny = inputImg.rows;
        Mat w1(1, Nx, CV_32F, Scalar(0));
        Mat w2(Ny, 1, CV_32F, Scalar(0));

        float* p1 = w1.ptr<float>(0);
        float* p2 = w2.ptr<float>(0);
        float dx = float(2.0 * CV_PI / Nx);
        float x = float(-CV_PI);
        for (int i = 0; i < Nx; i++)
        {
            p1[i] = float(0.5 * (tanh((x + gamma / 2) / beta) - tanh((x - gamma / 2) / beta)));
            x += dx;
        }
        float dy = float(2.0 * CV_PI / Ny);
        float y = float(-CV_PI);
        for (int i = 0; i < Ny; i++)
        {
            p2[i] = float(0.5 * (tanh((y + gamma / 2) / beta) - tanh((y - gamma / 2) / beta)));
            y += dy;
        }
        Mat w = w2 * w1;
        multiply(inputImg, w, outputImg);
    }
    """


# note: make sure that the psf is supposed to be centered around q1
# rather than the center of the image
visible_psf = calc_psf(img_rows // 2, img_cols // 2, LEN, THETA)
# visible_psf = fft_shift(visible_psf)
visible_psf = visible_psf * 1000
# visible_psf = cv.resize(visible_psf, (960, 540))
# cv.imshow("PSF", visible_psf)
print(np.sum(visible_psf))
tapered = edge_taper(roi_test)
# cv.imshow("Tapered", tapered)

# cv.imshow("Region of Interest", fft_shift(roi))
# cv.imshow("Display window", imgIn)
# cv.waitKey(25000)


# Test Full Code
print("rows and cols", img_rows, img_cols)
psf = calc_psf(img_rows, img_cols, LEN, THETA)
print("psf shape", np.shape(psf))
filter = calc_wnr_filter(psf, 1 / snr)
tapered_img = edge_taper(imgIn)
cv.imshow("Edge Tapered Image", tapered_img)

img_rows, img_cols = np.shape(imgIn)
img_rows = img_rows & -2
img_cols = img_cols & -2
roi = tapered_img[0:img_rows, 0:img_cols]
filtered_img = cv.normalize(filter_2D_freq(roi, filter), 0, 255, cv.NORM_MINMAX)

# cv.imshow("Motion-Deblurred Image", filtered_img)
# cv.imshow("PSF", 1000 * psf)
cv.waitKey(25000)
