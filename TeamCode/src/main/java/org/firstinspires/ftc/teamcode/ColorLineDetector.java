package org.firstinspires.ftc.teamcode;

import android.content.ContentValues;
import android.os.Environment;
import android.util.Log;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static org.opencv.imgproc.Imgproc.HoughLinesP;
import static org.opencv.imgproc.Imgproc.minAreaRect;

/**
 * Created by sam on 5/3/2018.
 */

public class ColorLineDetector {

    File logFile = new File(Environment.getExternalStorageDirectory() + "/Angles/Data/data.txt");
    // Lower and Upper bounds for range checking in HSV color space
    private Scalar mLowerBound = new Scalar(0);
    private Scalar mUpperBound = new Scalar(0);

    // Minimum contour area in percent for contours filtering
    private double mMinContourArea = 0.1;

    // Color radius for range checking in HSV color space with touch color
    private Scalar mColorRadius = new Scalar(25, 50, 50, 0);

    private Mat mSpectrum = new Mat();
    private List<MatOfPoint> mContours = new ArrayList<MatOfPoint>();
    private List<MatOfPoint> mPolys = new ArrayList<MatOfPoint>();


    // Cache
    Mat mPyrDownMat = new Mat();
    Mat mHsvMat = new Mat();
    Mat mMask = new Mat();
    Mat mDilatedMask = new Mat();
    Mat mHierarchy = new Mat();
    Mat mLines = new Mat();
    //Mat mRectangle = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE,new Size(7,7),new Point(7,7));
    ArrayList<Integer> Xs = new ArrayList<>(), Ys = new ArrayList<>(), Xw = new ArrayList<>();
    int totalX = 0, totalY = 0;
    Point centerP = new Point();

    LineClusters clusters = new LineClusters();

    public void setColorRange(Scalar minHSV, Scalar maxHSV) {
        mLowerBound = minHSV;
        mUpperBound = maxHSV;
    }


    public void setHsvColor(Scalar hsvColor) {
        double minH = (hsvColor.val[0] >= mColorRadius.val[0]) ? hsvColor.val[0] - mColorRadius.val[0] : 0;
        double maxH = (hsvColor.val[0] + mColorRadius.val[0] <= 255) ? hsvColor.val[0] + mColorRadius.val[0] : 255;

        mLowerBound.val[0] = minH;
        mUpperBound.val[0] = maxH;

        mLowerBound.val[1] = 0;
        mUpperBound.val[1] = 255;

        mLowerBound.val[2] = 0;
        mUpperBound.val[2] = 255;

        mLowerBound.val[3] = 0;
        mUpperBound.val[3] = 255;

        Log.i(ContentValues.TAG, "Set HSV Color Min: (" +
                mLowerBound.val[0] + "," +
                mLowerBound.val[1] + "," +
                mLowerBound.val[2] + ")");

        Log.i(ContentValues.TAG, "Set HSV Color Max: (" +
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

    final int PYR = 1;

    public void processLines(Mat mRgba) {
        Imgproc.cvtColor(mRgba, mHsvMat, Imgproc.COLOR_RGB2HSV_FULL);

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

//        Imgproc.erode(mMask, mMask, Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(5,5)));
//        Imgproc.dilate(mMask,mMask,Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT,new Size(13,13)));


        Imgproc.Canny(mMask, mMask, 50, 100);

        HoughLinesP(mMask, mLines, 5, Math.PI / 180, 7, 60 / PYR, 20 / PYR);

    }

    public void showLines(Mat mRgba) {

        try {
            double val0, val1, val2, val3;
            clusters = new LineClusters();
            for (int i = 0; i < mLines.rows(); i++) {
                double[] val = mLines.get(i, 0);

                val0 = val[0] * PYR;
                val1 = val[1] * PYR;
                val2 = val[2] * PYR;
                val3 = val[3] * PYR;


                double angle = ((Math.atan2(val1 - val3, val0 - val2) * 180 / Math.PI) + 180) % 180;

//                Log.println(Log.ASSERT,"This part is working","");
                if ((isWithin(angle, 30, 150) && !isWithin(angle, 85, 95)) &&
                        val1 > mRgba.rows() / 4 && val3 > mRgba.rows() / 4) {
                    Imgproc.line(mRgba,
                            new Point(val0, val1),
                            new Point(val2, val3),
                            new Scalar(255, 255, 255),
                            10);
                    clusters.add(new Line(new Point(val0, val1), new Point(val2, val3), angle));
//                    Log.println(Log.ASSERT, "TAG", angle + " is the angle of line " + i);

                    Imgproc.line(mRgba,
                            new Point(val0, val1),
                            new Point(val2, val3),
                            new Scalar(255, 255, 255),
                            10);
                    clusters.add(new Line(new Point(val0, val1), new Point(val2, val3), angle));
//                    Log.println(Log.ASSERT,"Angle ",angle + " is the angle of line " + i);
                }

//                else Imgproc.line(mRgba,
//                            new Point(val0, val1),
//                            new Point(val2, val3),
//                            new Scalar(255, 0, 0),
//                            10);


            }
//            Log.println(Log.ASSERT, "TAG", clusters.toString() + "\n");
            final int SHOW_THRESH = 2;
            for (int i = 0; i < clusters.clusterGroups.size() && i < 2; i++) {
                if (clusters.clusterGroups.get(i).lines.size() >= SHOW_THRESH) {
                    Point[] rectPoints = new Point[4];
                    MatOfPoint2f mp2f = new MatOfPoint2f();
                    mp2f.fromList(clusters.clusterGroups.get(i).points);
                    RotatedRect rRect = minAreaRect(mp2f);
                    rRect.points(rectPoints);
                    MatOfPoint mPoints = new MatOfPoint(rectPoints);
                    List<MatOfPoint> lPoints = new ArrayList<>();
                    lPoints.add(mPoints);
//                    Log.println(Log.ASSERT, "TAG", Arrays.toString(rectPoints) + "Points");


                    Imgproc.polylines(mRgba, lPoints, true, new Scalar(0, 255, 0), 10);
                }
            }

            int countedCount = 0;
            totalX = 0;
            totalY = 0;

            for (int i = 0; i < 2 && i < clusters.clusterGroups.size() - 1; i++) {
                LineCluster l = clusters.clusterGroups.get(i);
                LineCluster l0 = clusters.clusterGroups.get(i + 1);
                if (l.lines.size() >= SHOW_THRESH) {
                    Point p = calculateIntersect(l.angle, l.center(), l0.angle, l0.center());
                    if (!(p.x <= 0 || p.x >= mRgba.cols())) {
                        totalX += p.x;
                        totalY += p.y;
                        countedCount++;
                    }
                }
            }
            Log.println(Log.ASSERT,"countedCount",countedCount+"");
            if (countedCount > 0) {
                totalX = totalX / (countedCount);
                totalY = totalY / (countedCount);


                final int QUEUE_SIZE = 20;
                //was 30
                if (!(Xs.size() < QUEUE_SIZE) || !(Ys.size() < QUEUE_SIZE)) {
                    Xs.remove(0);
                    Ys.remove(0);
                }

                Xs.add(totalX);
                Ys.add(totalY);
                int sum = 0;
                for (int i = 0; i < Xs.size(); i++) {
                    sum += Xs.get(i) * (QUEUE_SIZE - i);
                }
                sum = (int) (sum / (QUEUE_SIZE * ((QUEUE_SIZE + 1) / 2)));
                centerP = new Point(sum, median(Ys));
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        try {
            Imgproc.circle(mRgba, centerP, 2, new Scalar(255, 0, 0), 5);
        } catch (Exception e) {
            e.printStackTrace();
        }
//        Log.println(Log.ASSERT,"Number of Clusters",""+clusters.clusterGroups.size());
    }

    //fjsla

    public double median(ArrayList<Integer> median) {
        ArrayList<Integer> m = new ArrayList<>();
        for (int i : median) m.add(i);
        Collections.sort(m);
        if (median.size() % 2 == 0)
            return (m.get(m.size() / 2) + m.get(m.size() / 2 - 1)) / 2;
        else return m.get(m.size() / 2);
    }


    public boolean isWithin(double val, double low, double high) {

        return val >= low && val <= high;
    }

    class Line {
        Point p1, p2;
        double angle;

        public Line(Point p1, Point p2, double angle) {
            this.p1 = p1;
            this.p2 = p2;
            this.angle = angle;
        }
    }

    class LineCluster implements Comparable {

        RotatedRect rRect = new RotatedRect();
        List<Line> lines = new ArrayList<>();
        List<Point> points = new ArrayList<>();
        double angle = 0;
        double area = 0;

        public LineCluster(Line line) {
            addLine(line);
        }

        public void addLine(Line line) {
            lines.add(line);
            points.add(line.p1);
            points.add(line.p2);
            avgAngle();
            updateRect();
        }

        private void updateRect() {
            MatOfPoint2f mp2f = new MatOfPoint2f();
            mp2f.fromList(points);
            rRect = minAreaRect(mp2f);
            area = rRect.size.area();
        }

        public boolean isClose(Line p, double tolerance) {
//            Log.println(Log.ASSERT,"Line 1", "angle: "+ p.angle + " p1: " + p.p1 + " p2:" + p.p2);
//            Log.println(Log.ASSERT,"Line 2", "angle: "+ angle + " center: " +center());
            Point center = pointIntersect(angle, center(), 0, p.p1);
            Point center2 = pointIntersect(angle, center(), 0, p.p2);
            boolean isWithin1 = isWithin(center.x, p.p1.x - tolerance, p.p1.x + tolerance);
            boolean isWithin2 = isWithin(center2.x, p.p2.x - tolerance, p.p2.x + tolerance);

//            Log.println(Log.ASSERT,"CalculatedIntersect", ""+center);
//            Log.println(Log.ASSERT,"isWithin1",isWithin1+" isWithin2"+isWithin2);
            if (isWithin1 || isWithin2) {

                return true;
            }

            return false;
        }

        public Point center() {
            return rRect.center;
        }

        public void avgAngle() {
            int n = 0;
            for (int i = 0; i < lines.size(); i++) {
                n += lines.get(i).angle;
            }
            angle = n / lines.size();
        }

        public String toString() {
            avgAngle();
            return "Angle is " + angle + "Lines are " + lines.size();
        }

        @Override
        public int compareTo(Object o) {
            if (o instanceof LineCluster) {
                if (area > ((LineCluster) o).area) return 1;
                else if (area < ((LineCluster) o).area) return -1;
                else return 0;
            } else return 0;
        }
    }

    class LineClusters {
        List<LineCluster> clusterGroups = new ArrayList<>();

        public void add(Line line) {
//            Log.println(Log.ASSERT,"Line Added of ","Angle: " + line.angle);
            boolean foundCluster = false;
            outerloop:
            for (int i = 0; i < clusterGroups.size(); i++) {
                for (int j = 0; j < clusterGroups.get(i).lines.size(); j++) {
                    if (isWithin(line.angle, clusterGroups.get(i).angle - 10, clusterGroups.get(i).angle + 10)) {
                        if (clusterGroups.get(i).isClose(line, 80)) {
                            clusterGroups.get(i).addLine(line);
                            foundCluster = true;
                            break outerloop;
                        }
                    }
                }
            }

            if (!foundCluster) {
                clusterGroups.add(new LineCluster(line));
            }
            Collections.sort(clusterGroups);

        }


        public void writeSolutionPoint() {
            if (clusterGroups.size() >= 2) {
                if (centerP != null) logToFile(centerP.y);
            }
        }


        public String toString() {
            String returnVal = "";
            for (int i = 0; i < clusterGroups.size(); i++) {
                returnVal += "cluster " + i + " " + clusterGroups.get(i).toString() + "\n";
            }
            return returnVal;
        }


    }

    public List<MatOfPoint> getContours() {
        return mContours;
    }


    public Point calculateIntersect(double angle1, Point p1, double angle2, Point p2) {
        Point intersect = new Point();
        angle1 *= Math.PI / 180;
        angle2 *= Math.PI / 180;
        int x1 = (int) p1.x;
        int y1 = -(int) p1.y;
        int x2 = (int) p2.x;
        int y2 = -(int) p2.y;
        double m1 = Math.tan(angle1);
        double m2 = Math.tan(angle2);
        int xf = (int) (((-m2 * x2) + y2 + (m1 * x1) - y1) / (m1 - m2));
        int yf = (int) ((m1 * xf) - (m1 * x1) + y1);
        intersect = new Point(xf, yf);
        return intersect;
    }

    public Point pointIntersect(double angle1, Point p1, double angle2, Point p2) {
        Point intersect = new Point();
        angle1 *= Math.PI / 180;
        angle2 *= Math.PI / 180;
        int x1 = (int) p1.x + 10000;
        int y1 = (int) p1.y + 10000;
        int x2 = (int) p2.x + 10000;
        int y2 = (int) p2.y + 10000;
        double m1 = Math.tan(angle1);
        double m2 = Math.tan(angle2);
        int xf = (int) (((-m2 * x2) + y2 + (m1 * x1) - y1) / (m1 - m2));
        int yf = (int) ((m1 * xf) - (m1 * x1) + y1);
        intersect = new Point(xf - 10000, yf - 10000);
        return intersect;
    }

    public void logToFile(Object o) {
        String text = o.toString();

        if (!logFile.exists()) {
            try {
                logFile.createNewFile();
            } catch (Exception e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
        try {
            //BufferedWriter for performance, true to set append to file flag
            BufferedWriter buf = new BufferedWriter(new FileWriter(logFile, true));
            buf.append(text);
            buf.append(System.lineSeparator());
            buf.close();
        }
        //this is a commit
        catch (Exception e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

}
