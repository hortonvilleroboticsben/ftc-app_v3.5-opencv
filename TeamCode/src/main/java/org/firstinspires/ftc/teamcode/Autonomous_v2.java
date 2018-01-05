package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.os.Looper;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
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

@Autonomous(name = "Auto", group = "Final")
public class Autonomous_v2 extends StateMachine_v6 {
    int question_number = 1;
    byte Alliance = 0;
    boolean btnOS;
    Orientation axes;
    int StartPos = 0;

    StateMachine_v6 drive = new StateMachine_v6(),
                    arm = new StateMachine_v6(),
                    glyph = new StateMachine_v6(),
                    vision = new StateMachine_v6();

    @Override
    public void init() {
        super.init();

        set_position(srvGr1,GR1CLOSED);
        set_position(srvGr2,GR2CLOSED);

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

        mtrLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void init_loop() {
        axes = IMUnav.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        telemetry.addData("Setup", Arrays.toString(new double[]{axes.firstAngle, axes.secondAngle, axes.thirdAngle}));
        telemetry.addData("SetupOK", (int) Math.abs(axes.firstAngle) <= 1 && (int) Math.abs(axes.secondAngle) <= 1 && (int) Math.abs(axes.thirdAngle) <= 1);

        switch (question_number) {
            case 1:
                telemetry.addData("Alliance: a=blue b=red", "");
                if ((gamepad1.a || gamepad1.b) && !gamepad1.start && btnOS == false) {
                    Alliance = (gamepad1.a) ? BLUE : RED;
                    btnOS = true;
                } else if (!gamepad1.a && !gamepad1.b && !gamepad1.start && btnOS == true) {
                    btnOS = false;
                    question_number++;
                }
                break;
            case 2:
                telemetry.addData("StartPos: a=1 b=2", "");
                if ((gamepad1.a || gamepad1.b) && !gamepad1.start && btnOS == false) {
                    StartPos = (gamepad1.a) ? 1 : 2;
                    btnOS = true;
                } else if (!gamepad1.a && !gamepad1.b && !gamepad1.start && btnOS == true) {
                    btnOS = false;
                    question_number++;
                }
                break;
            case 3:
                telemetry.addData("Ready to Begin", "");
                break;
        }

        if (StartPos == 0 || Alliance == 0)
            telemetry.addData("WARNING YOU DON'T HAVE ALL INITIALIZATION PARAMETERS IN", "");

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        vision.initializeMachine();
        drive.initializeMachine();
        arm.initializeMachine();
        glyph.initializeMachine();

        drive.ServoMove(srvPhone, CAM_VUMARK);
        drive.Pause(1000);
        drive.SetFlag(vision, "Read Relic");
        drive.AbsoluteMotorMove(mtrLift,liftPos.TWO.getVal(),0.3);

        vision.WaitForFlag("Read Relic");
        vision.ProcessRelic();
        vision.SetFlag(drive, "Relic Read");

        drive.WaitForFlag("Relic Read");
        drive.ServoMove(srvPhone, CAM_JEWELS);
        drive.Pause(1000);
        drive.SetFlag(vision, "Read Jewels");

        vision.WaitForFlag("Read Jewels");
        vision.ProcessJewels(vuforiaFrameToMat());
        vision.SetFlag(arm, "Jewels Read");

        arm.WaitForFlag("Jewels Read");
        arm.ServoMove(srvLR, LR_CENTER);
        arm.ServoIncrementalMove(srvUD, UD_DOWN, 0.01);
//            arm.ServoMove(srvUD, UD_DOWN + .04);
//            arm.Pause(800);
//            arm.ServoMove(srvUD, UD_DOWN);


        if(Alliance == BLUE) {
            //TODO: DRIVE must grab GLYPH.
            if (arm.next_state_to_execute()) {
                if (ballArray[0] == BallColor.RED) set_position(srvLR, LR_LEFT);
                else set_position(srvLR, LR_RIGHT);
                arm.incrementState();
            }
            arm.Pause(500);
            arm.SetFlag(drive, "Jewels Hit");
            arm.ServoMove(srvUD, UD_UP);
            arm.ServoMove(srvLR, LR_HOME);

            drive.WaitForFlag("Jewels Hit");
            drive.ServoMove(srvPhone, CAM_FRONT);

            if (StartPos == 1) {
                drive.Drive(35.75, 0.2);
                drive.Turn(-88, 0.2);
            }
            else if(StartPos == 2){
                drive.Drive(24,.2);
                drive.Turn(-89,.2);
                drive.Drive(11.8,.2);
                drive.Turn(-90.5,.2);
            }

/////////////////////////RED ALLIANCE////////////////////////////////////

        }else if(Alliance == RED){
            if (arm.next_state_to_execute()) {
                if (ballArray[0] == BallColor.BLUE) set_position(srvLR, LR_LEFT);
                else set_position(srvLR, LR_RIGHT);
                arm.incrementState();
            }
            arm.Pause(500);
            arm.SetFlag(drive, "Jewels Hit");
            arm.ServoMove(srvUD, UD_UP);
            arm.ServoMove(srvLR, LR_HOME);

            drive.WaitForFlag("Jewels Hit");
            drive.ServoMove(srvPhone, CAM_FRONT);
            if(StartPos == 1) {
                drive.Drive(-34.5, 0.2);
                drive.Turn(-88, 0.2);
            }else if(StartPos == 2){
                drive.Drive(-24,.2);
                drive.Turn(88,.2);
                drive.Drive(-11.3,.2);
                drive.Turn(-89.5,.2);
            }
        }
        if(vuMark != null) {
            if (vuMark ==RelicRecoveryVuMark.LEFT) {
                drive.Turn(25, 0.2);
                drive.Drive(-8.5, 0.2);
                drive.SetFlag(glyph,"open grabber");
            } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                drive.Turn(-25, 0.2);
                drive.Drive(-9, 0.2);
                drive.SetFlag(glyph,"open grabber");
            } else {
                drive.Turn(5,0.2);
                drive.Drive(-9, 0.2);
                drive.SetFlag(glyph,"open grabber");
            }
        }
        glyph.WaitForFlag("open grabber");
        glyph.ServoMove(srvGr1,GR1OPEN);
        glyph.ServoMove(srvGr2,GR2OPEN);
        glyph.Pause(200);
        glyph.Drive(10,0.2);

        if(ballArray[0] != null && ballArray[1] != null)
            telemetry.addData("ballArray", Arrays.toString(ballArray));
        else
            telemetry.addData("ballArray", "[null, null]");
        if(vuMark != null)
            telemetry.addData("vuMark", vuMark.toString());
        else
            telemetry.addData("vuMark", "null");
    }
}
