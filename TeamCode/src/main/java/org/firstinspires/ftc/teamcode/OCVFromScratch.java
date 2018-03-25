package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.content.Context;
import android.util.Log;
import android.view.SurfaceView;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

/**
 * Created by sam on 3/24/2018.
 */

@TeleOp(name = "OCVFS", group = "Testing")
public class OCVFromScratch extends OpMode implements CameraBridgeViewBase.CvCameraViewListener {


    private CameraBridgeViewBase mOpenCvCameraView;
    private Context context;
    public Mat mRgba;


    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(context) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS: {
                    Log.e("OpenCV", "OpenCV loaded successfully");
                    mOpenCvCameraView.enableView();
                }
                break;
                default: {
                    super.onManagerConnected(status);
                }
                break;
            }
        }
    };


    @Override
    public void init() {
        context = hardwareMap.appContext;
        try {
            View rootView = ((Activity) context).getWindow().getDecorView().findViewById(android.R.id.content);
            View v = rootView.findViewById(R.id.opencvView);
            mOpenCvCameraView = (CameraBridgeViewBase) v;
            mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
            mOpenCvCameraView.setCvCameraViewListener(this);
        }catch(Exception e){
            Log.e("View","Problem loading view from context");
            e.printStackTrace();
        }

        if (!OpenCVLoader.initDebug()) {
            Log.e("OpenCV", "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, context, mLoaderCallback);
        } else {
            Log.e("OpenCV", "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }

    }

    @Override
    public void loop() {
        telemetry.addData("Hello, World",""+new java.util.Date().getTime()+"");
    }

    @Override
    public void stop() {
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }


    @Override
    public void onCameraViewStarted(int width, int height) {
        mRgba = new Mat(height, width, CvType.CV_8UC4);
    }

    @Override
    public void onCameraViewStopped() {
        mRgba.release();
    }

    @Override
    public Mat onCameraFrame(Mat inputFrame) {
        mRgba = inputFrame;
        Log.e("FrameTime",""+new java.util.Date().getTime()+"");
        return mRgba;
    }
}
