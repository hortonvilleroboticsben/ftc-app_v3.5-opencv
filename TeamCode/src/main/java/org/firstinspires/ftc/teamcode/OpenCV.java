package org.firstinspires.ftc.teamcode;

import android.app.ActivityManager;
import android.app.Application;
import android.content.Context;
import android.graphics.Bitmap;
import android.os.Looper;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.BlockingQueue;

import static org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuMarkIdentification.TAG;
import static org.opencv.imgproc.Imgproc.minEnclosingCircle;

@TeleOp(name = "OPENCV")
public class OpenCV extends OpMode {

    VuforiaLocalizer vuforia;
    VuforiaTrackable relicTemplate;

    @Override
    public void init() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AXlr9dj/////AAAAGRujCnGOL0aIpjtd4y5IK2sFwI4jstOeOlytTrPr3jzeQQ9tGEHgLmuGdxzuxGNY5641pyXeeyJHccL+I4QCZq8Sodm5DAUBsAQQ9ox1EY3+KNfZISN06k1IqDf7YaRXhE02j+7aE4Apnm3Hvn9V5CDKSTgOq73eJId9uzHkuNaIx+UDV4fRS1HK5L6dSGmIw3+RW7uaFdBF0E2bvWzzpZv51KFzw5oy/9qFB9r6ke5B5Gd2zw9JjafwudFSpLVCiNzIreoTaIFYVMmGMuIFIT4C6oC13EcvbNl5CFpl+irLqhSI//nlL2L2DKxKtW5UTQqNBlOSBdTxWR/kSN12edlwOu0kFgzhKBFapn3KHC0V";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        relicTrackables.activate();

        vuforia.setFrameQueueCapacity(1);

        if(Looper.myLooper() == null) Looper.prepare();

        Context c = hardwareMap.appContext;

        BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(c) {
            @Override
            public void onManagerConnected(int status) {
                switch (status) {
                    case LoaderCallbackInterface.SUCCESS:
                    {
                        Log.i(TAG, "OpenCV loaded successfully");
                    } break;
                    default:
                    {
                        super.onManagerConnected(status);
                    } break;
                }
            }
        };

        Log.i(TAG, "Trying to load OpenCV library");
        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, c, mLoaderCallback);
        } else {
            Log.d(TAG, "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }


    }

    @Override
    public void loop() {
//        int i = vuforia.getFrameQueueCapacity();
//        int j = vuforia.getFrameQueue().size();

        VuforiaLocalizer.CloseableFrame frame = null;
        try {
            if (vuforia.getFrameQueue().size() >= 1)
                frame = vuforia.getFrameQueue().take();
        } catch (Exception e) {
            e.printStackTrace();
        }
        try {

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            long totalFrame = frame.getNumImages();

            Image rgb = null;

            int counter = 0;
            if (counter < totalFrame) {
                if (frame.getImage(counter).getFormat() == PIXEL_FORMAT.RGB565)
                    rgb = frame.getImage(counter);
                counter++;
            }

            Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(rgb.getPixels());

            Point p = new Point();
            Mat tmp = new Mat();

            try {
                tmp = new Mat(rgb.getWidth(), rgb.getHeight(), CvType.CV_8UC4);

            }catch (Exception e){
                e.printStackTrace();
            }
            Utils.bitmapToMat(bm, tmp);

            telemetry.addData("Balls", process(tmp));

//close the frame, prevents memory leaks and crashing
            frame.close();

            telemetry.addData("vuMark", vuMark);

        } catch (Exception e) {
            System.out.println(e);
        }
    }

    @Override
    public void stop() {
        super.stop();
    }

    private double distance(Point center, Point check) {
        return Math.hypot((center.x - check.x), (center.y - check.y));
    }


    public String process(Mat inputFrame) {

        int height = inputFrame.height();
        int width = inputFrame.width();

        Mat mRgba;
        Mat mROI;
        Rect rROI;
        Point centerROI;

        Scalar mBlobColorRgba;
        Scalar mBlobColorHsv;

        ColorBlobDetector mDetectorRed;
        ColorBlobDetector mDetectorBlue;

        Mat mSpectrumRed;
        Mat mSpectrumBlue;
        Size SPECTRUM_SIZE;

        Scalar CONTOUR_COLOR_RED;
        Scalar CONTOUR_COLOR_BLUE;

        Scalar ROI_COLOR;

        mRgba = new Mat(height, width, CvType.CV_8UC4);

        rROI = new Rect((int) width / 3, (int) height / 3, (int) width / 3, (int) height / 3);
        centerROI = new Point(rROI.width / 2, rROI.height / 2);
        mROI = new Mat(mRgba, rROI);

        Log.d(TAG, "onCamerViewStarted Width: " + width + " Height: " + height);

        mDetectorRed = new ColorBlobDetector();
        mDetectorBlue = new ColorBlobDetector();

        mSpectrumRed = new Mat();
        mSpectrumBlue = new Mat();

        mBlobColorRgba = new Scalar(255);

        SPECTRUM_SIZE = new Size(200, 64);

        CONTOUR_COLOR_RED = new Scalar(255, 0, 0, 255);
        CONTOUR_COLOR_BLUE = new Scalar(0, 0, 255, 255);

        ROI_COLOR = new Scalar(255, 255, 255, 255);

        mDetectorRed.setColorRange(new Scalar(237, 120, 130, 0), new Scalar(20, 255, 255, 255));
        mDetectorBlue.setColorRange(new Scalar(125, 120, 130, 0), new Scalar(187, 255, 255, 255));

        mRgba = inputFrame;
        mROI = new Mat(mRgba, rROI);

        double radiusRedBest = 0.0;
        double radiusBlueBest = 0.0;

        String message = "";

        if (true) {
            Imgproc.blur(mROI, mROI, new Size(20, 20));

            mDetectorRed.process(mROI);
            mDetectorBlue.process(mROI);

            List<MatOfPoint> contoursRed = mDetectorRed.getContours();
            Log.e(TAG, "Red Contours count: " + contoursRed.size());
            //Imgproc.drawContours(mRgba, contoursRed, -1, CONTOUR_COLOR_RED);

            List<MatOfPoint> contoursBlue = mDetectorBlue.getContours();
            Log.e(TAG, "Blue Contours count: " + contoursBlue.size());
            //Imgproc.drawContours(mRgba, contoursBlue, -1, CONTOUR_COLOR_BLUE);

            List<Moments> muRed = new ArrayList<Moments>(contoursRed.size());

            Point centerRedBest = null;

            List<MatOfPoint2f> contours2fRed = new ArrayList<MatOfPoint2f>();
            List<MatOfPoint2f> polyMOP2fRed = new ArrayList<MatOfPoint2f>();
            List<MatOfPoint> polyMOPRed = new ArrayList<MatOfPoint>();

            float[] radiusRed = new float[contoursRed.size()];
//TODO
            //for (int i = 0; i < contoursRed.size(); i++) {
            int i = 0;
            Point centerRed = new Point();
            muRed.add(i, Imgproc.moments(contoursRed.get(i), false));

            contours2fRed.add(new MatOfPoint2f());
            polyMOP2fRed.add(new MatOfPoint2f());
            polyMOPRed.add(new MatOfPoint());

            contoursRed.get(i).convertTo(contours2fRed.get(i), CvType.CV_32FC2);
            Imgproc.approxPolyDP(contours2fRed.get(i), polyMOP2fRed.get(i), 3, true);
            polyMOP2fRed.get(i).convertTo(polyMOPRed.get(i), CvType.CV_32S);

            minEnclosingCircle(polyMOP2fRed.get(i), centerRed, radiusRed);
            Imgproc.circle(mRgba, new Point(centerRed.x + rROI.x, centerRed.y + rROI.y), 16, CONTOUR_COLOR_RED, 16);
            Log.e(TAG, "Red Center: (" + (int) centerRed.x + "," + (int) centerRed.y + ") with radius: " + (int) radiusRed[i]);

            if (centerRedBest == null) {
                centerRedBest = centerRed;
                radiusRedBest = radiusRed[0];
            } else {
                if (distance(centerROI, centerRed) < distance(centerROI, centerRedBest)) {
                    centerRedBest = centerRed;
                    radiusRedBest = radiusRed[0];
                }
            }

            //}
            if (centerRedBest != null) {
                Imgproc.circle(mRgba, new Point(centerRedBest.x + rROI.x, centerRedBest.y + rROI.y), (int) radiusRedBest, CONTOUR_COLOR_RED, 16);
            }

            List<Moments> muBlue = new ArrayList<Moments>(contoursBlue.size());
            Point centerBlueBest = null;

            List<MatOfPoint2f> contours2fBlue = new ArrayList<MatOfPoint2f>();
            List<MatOfPoint2f> polyMOP2fBlue = new ArrayList<MatOfPoint2f>();
            List<MatOfPoint> polyMOPBlue = new ArrayList<MatOfPoint>();

            float[] radiusBlue = new float[contoursBlue.size()];

            //for (int i = 0; i < contoursBlue.size(); i++) {
            i = 0;
            Point centerBlue = new Point();
            muBlue.add(i, Imgproc.moments(contoursBlue.get(i), false));

            contours2fBlue.add(new MatOfPoint2f());
            polyMOP2fBlue.add(new MatOfPoint2f());
            polyMOPBlue.add(new MatOfPoint());

            contoursBlue.get(i).convertTo(contours2fBlue.get(i), CvType.CV_32FC2);
            Imgproc.approxPolyDP(contours2fBlue.get(i), polyMOP2fBlue.get(i), 3, true);
            polyMOP2fBlue.get(i).convertTo(polyMOPBlue.get(i), CvType.CV_32S);

            minEnclosingCircle(polyMOP2fBlue.get(i), centerBlue, radiusBlue);
            Imgproc.circle(mRgba, new Point(centerBlue.x + rROI.x, centerBlue.y + rROI.y), 16, CONTOUR_COLOR_BLUE, 16);
            Log.e(TAG, "Blue Center: (" + (int) centerBlue.x + "," + (int) centerBlue.y + ") with radius: " + (int) radiusBlue[i]);

            if (centerBlueBest == null) {
                centerBlueBest = centerBlue;
                radiusBlueBest = radiusBlue[0];
            } else {
                if (distance(centerROI, centerBlue) < distance(centerROI, centerBlueBest)) {
                    centerBlueBest = centerBlue;
                    radiusBlueBest = radiusBlue[0];
                }
            }
            //}
            if (centerBlueBest != null) {
                Imgproc.circle(mRgba, new Point(centerBlueBest.x + rROI.x, centerBlueBest.y + rROI.y), (int) radiusBlueBest, CONTOUR_COLOR_BLUE, 16);
            }

            Mat colorLabel = mRgba.submat(4, 68, 4, 68);
            colorLabel.setTo(mBlobColorRgba);


            if (centerRedBest != null && centerBlueBest != null) {
                message = "";
                if (centerBlueBest.x < centerRedBest.x) {
                    message = "Blue, Red";
                } else {
                    message = "Red ,Blue";
                }
                //telemetry.addData("balls", message);
                Log.e(TAG, message);
            }


            Imgproc.rectangle(mRgba, new Point(rROI.x, rROI.y), new Point(rROI.x + rROI.width, rROI.y + rROI.height), ROI_COLOR, 16);


        }
        return message;
    }



class ColorBlobDetector {
    // Lower and Upper bounds for range checking in HSV color space
    private Scalar mLowerBound = new Scalar(0);
    private Scalar mUpperBound = new Scalar(0);

    // Minimum contour area in percent for contours filtering
    private double mMinContourArea = 0.1;

    // Color radius for range checking in HSV color space with touch color
    private Scalar mColorRadius = new Scalar(25, 50, 50, 0);

    private Mat mSpectrum = new Mat();
    private List<MatOfPoint> mContours = new ArrayList<MatOfPoint>();


    // Cache
    Mat mPyrDownMat = new Mat();
    Mat mHsvMat = new Mat();
    Mat mMask = new Mat();
    Mat mDilatedMask = new Mat();
    Mat mHierarchy = new Mat();

    public void setColorRadius(Scalar radius) {
        mColorRadius = radius;
    }

    public void setColorRange(Scalar minHSV, Scalar maxHSV) {
        mLowerBound = minHSV;
        mUpperBound = maxHSV;
    }

    public void setHsvColor(Scalar hsvColor) {
        double minH = (hsvColor.val[0] >= mColorRadius.val[0]) ? hsvColor.val[0] - mColorRadius.val[0] : 0;
        double maxH = (hsvColor.val[0] + mColorRadius.val[0] <= 255) ? hsvColor.val[0] + mColorRadius.val[0] : 255;

        mLowerBound.val[0] = minH;
        mUpperBound.val[0] = maxH;

        //mLowerBound.val[1] = hsvColor.val[1] - mColorRadius.val[1];
        //mUpperBound.val[1] = hsvColor.val[1] + mColorRadius.val[1];
        mLowerBound.val[1] = 0;
        mUpperBound.val[1] = 255;

        //mLowerBound.val[2] = hsvColor.val[2] - mColorRadius.val[2];
        //mUpperBound.val[2] = hsvColor.val[2] + mColorRadius.val[2];
        mLowerBound.val[2] = 0;
        mUpperBound.val[2] = 255;

        mLowerBound.val[3] = 0;
        mUpperBound.val[3] = 255;

        Log.i(TAG, "Set HSV Color Min: (" +
                mLowerBound.val[0] + "," +
                mLowerBound.val[1] + "," +
                mLowerBound.val[2] + ")");

        Log.i(TAG, "Set HSV Color Max: (" +
                mUpperBound.val[0] + "," +
                mUpperBound.val[1] + "," +
                mUpperBound.val[2] + ")");

        Mat spectrumHsv = new Mat(1, (int) (maxH - minH), CvType.CV_8UC3);

        for (int j = 0; j < maxH - minH; j++) {
            byte[] tmp = {(byte) (minH + j), (byte) 255, (byte) 255};
            spectrumHsv.put(0, j, tmp);
        }

        Imgproc.cvtColor(spectrumHsv, mSpectrum, Imgproc.COLOR_HSV2RGB_FULL, 4);
    }

    public Mat getSpectrum() {
        return mSpectrum;
    }

    public void setMinContourArea(double area) {
        mMinContourArea = area;
    }

    public void process(Mat rgbaImage) {
        Imgproc.pyrDown(rgbaImage, mPyrDownMat);
        Imgproc.pyrDown(mPyrDownMat, mPyrDownMat);

        Imgproc.cvtColor(mPyrDownMat, mHsvMat, Imgproc.COLOR_RGB2HSV_FULL);

        if (mUpperBound.val[0] < mLowerBound.val[0]) {

            Mat mMask1 = new Mat();
            Mat mMask2 = new Mat();


            Scalar tLowerBound = mLowerBound.clone();
            Scalar tUpperBound = mUpperBound.clone();

            tLowerBound.val[0] = 0.0;
            tUpperBound.val[0] = mUpperBound.val[0];

            Core.inRange(mHsvMat, tLowerBound, tUpperBound, mMask1);

            tLowerBound = mLowerBound.clone();
            tUpperBound.val[0] = 255.0;

            Core.inRange(mHsvMat, tLowerBound, tUpperBound, mMask2);

            Core.add(mMask1, mMask2, mMask);

        } else {
            Core.inRange(mHsvMat, mLowerBound, mUpperBound, mMask);
        }

        Imgproc.dilate(mMask, mDilatedMask, new Mat());

        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

        Imgproc.findContours(mDilatedMask, contours, mHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find max contour area
        double maxArea = 0;
        Iterator<MatOfPoint> each = contours.iterator();
        while (each.hasNext()) {
            MatOfPoint wrapper = each.next();
            double area = Imgproc.contourArea(wrapper);
            if (area > maxArea)
                maxArea = area;
        }


        // Filter contours by area and resize to fit the original image size
        mContours.clear();
        each = contours.iterator();
        while (each.hasNext()) {
            MatOfPoint contour = each.next();
            if (Imgproc.contourArea(contour) > mMinContourArea * maxArea) {
                Core.multiply(contour, new Scalar(4, 4), contour);
                mContours.add(contour);
            }
        }
    }

    public List<MatOfPoint> getContours() {
        return mContours;
    }
}
}