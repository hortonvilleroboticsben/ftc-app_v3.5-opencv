package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.os.Looper;
import android.util.Log;
import android.view.SurfaceView;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;

import static org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuMarkIdentification.TAG;

@TeleOp(name = "visionthing", group = "Competition")
public class OpenCV_OnlyVisionTest extends OpMode implements CameraBridgeViewBase.CvCameraViewListener2{

    public static JavaCameraView openCVCamera;
    SurfaceView mView = new SurfaceView(hardwareMap.appContext);

    private static boolean initialized = false;
    public Mat mRgba;
    public int width,height;
    FPS fps;


    public OpenCV_OnlyVisionTest() {
        initialized = false;
        openCVCamera = null;
    }


    public void onCameraViewStarted(int width, int height) {
        mRgba = new Mat(height, width, CvType.CV_8UC4);


        Log.e(TAG, "onCamerViewStarted Width: " + width + " Height: " + height);

    }

    @Override
    public void onCameraViewStopped() {
        mRgba.release();
    }


    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        Log.e("time",""+new java.util.Date().getTime()+"");
        mRgba = inputFrame.rgba();
        return mRgba;
    }

    boolean isInitialized() {
        return initialized;
    }

    private void error(String message) {
        Log.e("FTCVision", message);
        telemetry.addData("Vision Status", message);
    }

    /**
     * Set the camera to use
     * This method may fail if the camera is locked.
     *
     * @param camera Camera to use
     */
    public void setCamera(Cameras camera) {
        if (openCVCamera == null)
            return;
        openCVCamera.disableView();
        if (initialized) openCVCamera.disconnectCamera();
        openCVCamera.setCameraIndex(camera.getID());
        if (initialized)
            if (!openCVCamera.connectCamera(width, height))
                error("Could not initialize camera!\r\n" +
                        "This may occur because the OpenCV Manager is not installed,\r\n" +
                        "CAMERA permission is not allowed in AndroidManifest.xml,\r\n" +
                        "or because another app is currently locking it.");
        openCVCamera.enableView();
    }

    /**
     * Set the maximum frame size that the camera uses
     * This method will fail if the camera is locked - it is recommended to check the result.
     *
     * @param frameSize Maximum (target) frame size
     * @return Actual frame size or null if cannot be set
     */
    public Size setFrameSize(Size frameSize) {
        if (openCVCamera == null)
            return null;

        openCVCamera.disableView();
        if (initialized) openCVCamera.disconnectCamera();
        openCVCamera.setMaxFrameSize((int) frameSize.width, (int) frameSize.height);
        if (initialized)
            if (!openCVCamera.connectCamera((int) frameSize.width, (int) frameSize.height))
                error("Could not initialize camera!\r\n" +
                        "This may occur because the OpenCV Manager is not installed,\r\n" +
                        "CAMERA permission is not allowed in AndroidManifest.xml,\r\n" +
                        "or because another app is currently locking it.");
        openCVCamera.enableView();

        width = openCVCamera.getWidth();
        height = openCVCamera.getHeight();
        if (width == 0 || height == 0) {
            Log.e("FTCVision", "OpenCV Camera failed to initialize width and height properties on startup.\r\n" +
                    "This is generally okay, but if you use width or height during init() you may\r\n" +
                    "run into a problem.");
        }

        return new Size(width, height);
    }


    public void initializeOpenCV() {
        if (Looper.myLooper() == null) Looper.prepare();

        Context c = hardwareMap.appContext;

        BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(c) {
            @Override
            public void onManagerConnected(int status) {
                switch (status) {
                    case LoaderCallbackInterface.SUCCESS: {
                        Log.e(TAG, "OpenCV loaded successfully");
//                       mOpenCvCameraView.enableView();

                    }
                    break;
                    default: {
                        super.onManagerConnected(status);
                    }
                    break;
                }
            }
        };

        Log.e(TAG, "Trying to load OpenCV library");
        if (!OpenCVLoader.initDebug()) {
            Log.e(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, c, mLoaderCallback);
        } else {
            Log.e(TAG, "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }


    @Override
    public void init() {
        initializeOpenCV();
        openCVCamera = new JavaCameraView(hardwareMap.appContext, Cameras.SECONDARY.getID());
        //openCVCamera.surface(mView.getHolder());
        openCVCamera.setVisibility(SurfaceView.VISIBLE);
        openCVCamera.setCvCameraViewListener(this);
        if (openCVCamera != null)
            openCVCamera.disableView();
        openCVCamera.enableView();
        if (!openCVCamera.connectCamera(400, 400))
            error("Could not initialize camera!\r\n" +
                    "This may occur because the OpenCV Manager is not installed,\r\n" +
                    "CAMERA permission is not allowed in AndroidManifest.xml,\r\n" +
                    "or because another app is currently locking it.");

        //Initialize FPS counter and sensors
        fps = new FPS();


        //Done!
        width = openCVCamera.getWidth();
        height = openCVCamera.getHeight();
        initialized = true;

//        setCamera(Cameras.PRIMARY);
//        setFrameSize(new Size(400,400));
    }

    @Override
    public void loop() {
    }
}