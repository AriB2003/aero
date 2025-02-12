import cv2 as cv
import numpy as np
import sys

print(cv.__version__)

LEN = 780
THETA = 15
snr = 300

imgIn = cv.imread("testblurred.jpg", cv.IMREAD_GRAYSCALE)

if imgIn is None:
    sys.exit("Could not read the image.")

# the C code defines a Mat here, stuff like this isn't really doable in Python

# it needs to process even image only
img_rows, img_cols = np.shape(imgIn)
img_rows = img_rows & -2
img_cols = img_cols & -2
roi = imgIn[0:img_rows, 0:img_cols]
# this is not correct code; it just creates a rectangle and I want to try and make it unnecessary

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

# help function was deleted


def calcPSF(num_rows: int, num_cols: int, psf_length: int, psf_angle: float):

    # make psf_matrix
    psf_matrix = np.zeros((num_rows, num_cols), dtype=np.single)
    # running psf_matrix = np.zeros((num_rows, num_cols),dtype=np.single)
    # reduces precision to a 32 bit float but may make it run faster

    # draw an ellipse at psf_angle with psf_length in the middle of the screen
    center = (num_cols // 2, num_rows // 2)
    # may want to change 0 to something else to account for lateral motion blur
    # or maybe even make a gradient on the edges of the point spread
    axes = (0, psf_length // 2)  # could probably just use floor division here
    cv.ellipse(psf_matrix, center, axes, 90 - psf_angle, 0, 360, 255, -1)
    print("sum: ", np.sum(psf_matrix))
    print(center)
    left = num_rows // 2 - 50
    right = num_rows // 2 + 50
    up = num_cols // 2 - 50
    down = num_cols // 2 + 50
    # psf_matrix = psf_matrix[left:right, up:down]
    # cv.rectangle(psf_matrix, (left, up), (right, down), 255, -1)
    # print(num_rows, num_cols)
    # print(center)
    print(psf_matrix.shape)
    # print(psf_matrix[10, 10])
    # print(psf_matrix[num_rows // 2, num_cols // 2])
    return psf_matrix / np.sum(psf_matrix)  # maybe do floor divide here


#        Mat h(filterSize, CV_32F, Scalar(0));
#        Point point(filterSize.width / 2, filterSize.height / 2);
#        ellipse(h, point, Size(0, cvRound(float(len) / 2.0)), 90.0 - theta, 0, 360, Scalar(255), FILLED);
#        Scalar summa = sum(h);
#        outputImg = h / summa[0];
#    }
"""
    note to self: clean up above code before continuing

    void fftshift(const Mat& inputImg, Mat& outputImg)
    {
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
    }
 
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
 
    void calcWnrFilter(const Mat& input_h_PSF, Mat& output_G, double nsr)
    {
        Mat h_PSF_shifted;
        fftshift(input_h_PSF, h_PSF_shifted);
        Mat planes[2] = { Mat_<float>(h_PSF_shifted.clone()), Mat::zeros(h_PSF_shifted.size(), CV_32F) };
        Mat complexI;
        merge(planes, 2, complexI);
        dft(complexI, complexI);
        split(complexI, planes);
        Mat denom;
        pow(abs(planes[0]), 2, denom);
        denom += nsr;
        divide(planes[0], denom, output_G);
    }
 
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

psf = calcPSF(img_rows // 2, img_cols // 2, LEN, THETA)
visible_psf = psf * 10000000
resize_psf = cv.resize(visible_psf, (960, 540))
cv.imshow("PSF", resize_psf)
print(np.sum(psf))
# cv.imshow("Region of Interest", roi)
# cv.imshow("Display window", imgIn)
cv.waitKey(15000)
