package com.example.imagereader;


import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.UUID;
import static android.os.Build.VERSION_CODES.KITKAT;
import static org.opencv.core.CvType.CV_8UC1;
import static org.opencv.core.CvType.CV_8UC3;
import static org.opencv.core.CvType.CV_32SC2;

/**
 * A utility class that contains image processing functions to convert to gray scale, remove shadows and more
 * @Author Hima Velaga
 */
public class ImageProcessing {

    /**
     * Mat is converted to HSV and value channel is changed to remove shadows
     * @param hsvImg
     * @return
     */
    public Mat removeShadows(Mat hsvImg){
        List<Mat> channel = new ArrayList<>(3);
        Core.split(hsvImg, channel);
        channel.set(2, new Mat(hsvImg.rows(), hsvImg.cols(), CV_8UC1, new Scalar(200)));//Set V
        //Merge channels
        Core.merge(channel, hsvImg);
        Mat rgbImg = new Mat();
        Imgproc.cvtColor(hsvImg, rgbImg, Imgproc.COLOR_HSV2RGB);
        return hsvImg;
    }

    /**
     * Conversion to grayscale
     * @param rgbImg
     * @return
     */
    public Mat grayScale(Mat rgbImg){
        Mat gray = new Mat(rgbImg.rows(), rgbImg.cols(), CV_8UC1);
        Imgproc.cvtColor(rgbImg, gray, Imgproc.COLOR_RGB2GRAY, 1);
        Core.normalize(gray, gray, 0, 255, Core.NORM_MINMAX, CV_8UC1);
        return gray;
    }

    /**
     * Bilateral filter applied to smooth the image and reduce background noise
     * Canny filter is used to find the outline of the object
     * @param gray
     * @return
     */
    public Mat filter(Mat gray){
        Mat filter = new Mat();
        Imgproc.bilateralFilter(gray, filter, 9, 9 * 2, 9 / 2);
        Mat edges = new Mat();
        Mat img = new Mat();
        double otsu_thresh_val = Imgproc.threshold(filter, img, 0, 255, Imgproc.THRESH_BINARY | Imgproc.THRESH_OTSU);
        double high_thresh_val = otsu_thresh_val;
        double lower_thresh_val = otsu_thresh_val * 0.5;
        System.out.println("thres " + lower_thresh_val + " " + high_thresh_val);
        //Imgproc.Canny(filter, edges, lower_thresh_val, high_thresh_val, 3, false);  //before 50, 225
        Imgproc.Canny(filter, edges, lower_thresh_val, high_thresh_val, 3, false);  //before 50, 225
        return edges;
    }

    /**
     * Finds the biggest contour in a list of cntours
     * @param contours2 list of contours of type MatOfPoint
     * @param contour_area2 area of contour
     * @return index of maximum contour size
     */
    public int findBiggestContour(List<MatOfPoint> contours2, double contour_area2){
        int maxAreaContourId = -1;
        double maxArea = 0;
        //Check for the index of the largest contour
        for (int i = 0; i < contours2.size(); i++) {
            contour_area2 = Imgproc.contourArea(contours2.get(i));
            if (contour_area2 > 20) {
                // count2++;
                if (contour_area2 > maxArea) {
                    maxArea = contour_area2;
                    maxAreaContourId = i;
                }
            }
        }
        return maxAreaContourId;
    }

}
