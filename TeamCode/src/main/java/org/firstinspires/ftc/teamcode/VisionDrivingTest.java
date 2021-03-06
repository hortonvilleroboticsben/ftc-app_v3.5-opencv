package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.os.Looper;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;

import java.util.Arrays;

import static org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuMarkIdentification.TAG;
@Deprecated
//@Autonomous(name = "aslfkjasdlk", group = "Final")
public class VisionDrivingTest extends StateMachine_v6 {
    int question_number = 1;
    byte Alliance = BLUE;
    boolean btnOS;
    Orientation axes;
    int StartPos = 0;

    long timeElapsed = 0;
    long maxFrameWait = 0;
    Timer t = new Timer();
    Timer t0 = new Timer();

    boolean done = false;

    StateMachine_v6 drive = new StateMachine_v6(),
                    arm = new StateMachine_v6(),
                    glyph = new StateMachine_v6(),
                    vision = new StateMachine_v6();

    @Override
    public void init() {
        super.init();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AXlr9dj/////AAAAGRujCnGOL0aIpjtd4y5IK2sFwI4jstOeOlytTrPr3jzeQQ9tGEHgLmuGdxzuxGNY5641pyXeeyJHccL+I4QCZq8Sodm5DAUBsAQQ9ox1EY3+KNfZISN06k1IqDf7YaRXhE02j+7aE4Apnm3Hvn9V5CDKSTgOq73eJId9uzHkuNaIx+UDV4fRS1HK5L6dSGmIw3+RW7uaFdBF0E2bvWzzpZv51KFzw5oy/9qFB9r6ke5B5Gd2zw9JjafwudFSpLVCiNzIreoTaIFYVMmGMuIFIT4C6oC13EcvbNl5CFpl+irLqhSI//nlL2L2DKxKtW5UTQqNBlOSBdTxWR/kSN12edlwOu0kFgzhKBFapn3KHC0V";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        relicTrackables.activate();

        vuforia.setFrameQueueCapacity(1);




        if (Looper.myLooper() == null) Looper.prepare();

        Context c = hardwareMap.appContext;

        BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(c) {
            @Override
            public void onManagerConnected(int status) {
                switch (status) {
                    case LoaderCallbackInterface.SUCCESS: {
                        Log.i(TAG, "OpenCV loaded successfully");
                    }
                    break;
                    default: {
                        super.onManagerConnected(status);
                    }
                    break;
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

        ////////////////////////////////////////////////////////////////////////////////////////////
    }

//    @Override
//    public void init_loop() {
//        axes = IMUnav.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//
//        telemetry.addData("Setup", Arrays.toString(new double[]{axes.firstAngle, axes.secondAngle, axes.thirdAngle}));
//        telemetry.addData("SetupOK", (int) Math.abs(axes.firstAngle) <= 1 && (int) Math.abs(axes.secondAngle) <= 1 && (int) Math.abs(axes.thirdAngle) <= 1);
//
//        switch (question_number) {
//            case 1:
//                telemetry.addData("Alliance: a=blue b=red", "");
//                if ((gamepad1.a ^ gamepad1.b) && !gamepad1.start && btnOS == false) {
//                    Alliance = (gamepad1.a) ? BLUE : RED;
//                    btnOS = true;
//                } else if (!gamepad1.a && !gamepad1.b && !gamepad1.start && btnOS == true) {
//                    btnOS = false;
//                    question_number++;
//                }
//                break;
//            case 2:
//                telemetry.addData("StartPos: a=1 b=2", "");
//                if ((gamepad1.a ^ gamepad1.b) && !gamepad1.start && btnOS == false) {
//                    StartPos = (gamepad1.a) ? 1 : 2;
//                    btnOS = true;
//                } else if (!gamepad1.a && !gamepad1.b && !gamepad1.start && btnOS == true) {
//                    btnOS = false;
//                    question_number++;
//                }
//                break;
//            case 3:
//                telemetry.addData("Ready to Begin", "");
//                break;
//        }
//
//        if (StartPos == 0 || Alliance == 0)
//            telemetry.addData("WARNING YOU DON'T HAVE ALL INITIALIZATION PARAMETERS IN", "");
//
//    }


    @Override
    public void start() {
        t.reset();
        t0.reset();
    }

    @Override
    public void loop() {
        O = IMUnav.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        drive.initializeMachine();
        if(vuforia.getFrameQueue().peek() != null) {
            maxFrameWait = Math.max(maxFrameWait, t0.getElapsedTime());
        }else t0.reset();
        drive.centerOnTriangle(vuforiaFrameToMat(),Alliance);
        if(drive.next_state_to_execute()) {
            done = true;
            drive.incrementState();
        }
        drive.GyroTurn(-90,0.07);
        if(!done){
            timeElapsed = t.getElapsedTime();
        }

        telemetry.addData("done",done);
        telemetry.addData("timeScanning", timeElapsed);
        telemetry.addData("maxFrameWait", maxFrameWait);
        telemetry.addData("angle",Arrays.toString(new double[] {O.firstAngle, O.secondAngle, O.thirdAngle}));
    }
}
