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

@Autonomous(name = "AutoNew", group = "Final")
public class Autonomous_v3 extends StateMachine_v7 {
    int question_number = 1;
    byte Alliance = 0;
    boolean btnOS;
    Orientation axes;
    int StartPos = 0;

    StateMachine_v7 drive = new StateMachine_v7(),
                    arm = new StateMachine_v7(),
                    glyph = new StateMachine_v7(),
                    vision = new StateMachine_v7();

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
                if ((gamepad1.a ^ gamepad1.b) && !gamepad1.start && btnOS == false) {
                    Alliance = (gamepad1.a) ? BLUE : RED;
                    btnOS = true;
                } else if (!gamepad1.a && !gamepad1.b && !gamepad1.start && btnOS == true) {
                    btnOS = false;
                    question_number++;
                }
                break;
            case 2:
                telemetry.addData("StartPos: a=1 b=2", "");
                if ((gamepad1.a ^ gamepad1.b) && !gamepad1.start && btnOS == false) {
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
        drive.Pause(200);
        drive.SetFlag(vision, "Read Relic");

        arm.ServoMove(srvGr1,GR1CLOSED);
        arm.ServoMove(srvGr2,GR2CLOSED);
        arm.Pause(500);
        arm.AbsoluteMotorMove(mtrLift,liftPos.TWO.getVal(),0.8);

        vision.WaitForFlag("Read Relic");
        vision.ProcessRelic();
        vision.SetFlag(drive, "Relic Read");

        drive.WaitForFlag("Relic Read");
        drive.ServoMove(srvPhone, CAM_JEWELS);
        drive.Pause(200);
        drive.SetFlag(vision, "Read Jewels");

        vision.WaitForFlag("Read Jewels");
        vision.ProcessJewels(vuforiaFrameToMat());
        vision.SetFlag(arm, "Jewels Read");

        arm.WaitForFlag("Jewels Read");
        arm.ServoMove(srvLR, LR_CENTER);
        arm.ServoIncrementalMove(srvUD, UD_DOWN, 0.1);
        arm.Pause(500);
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
                if(vuMark != null) {
                    if (vuMark == RelicRecoveryVuMark.LEFT) {
                        drive.Drive(28.75,0.2);
                    } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                        drive.Drive(42.75,0.2);
                    } else {
                        drive.Drive(35.75,0.2);
                        }
                }
                drive.SetFlag(arm,"Off Platform");
                drive.Pause(200);

                arm.WaitForFlag("Off Platform");
                arm.AbsoluteMotorMove(mtrLift,liftPos.AUTO.getVal(),.4);

                drive.Turn(-88, .2);
            }


            else if(StartPos == 2){
                drive.Drive(26,.2);
                drive.Drive(-4,.2);
                drive.OWTurn(-45,.2);
                drive.SetFlag(glyph,"Grab Relic");
                drive.SetFlag(arm, "Off Platform");

                glyph.WaitForFlag("Grab Relic");
                glyph.AbsoluteMotorMove(mtrExtend,extendPos.BLUE_AUTO.getVal(),0.05);
                glyph.ServoMove(srvClaw,CLAWCLOSED);
                glyph.Pause(200);
                glyph.SetFlag(drive,"move again");

                drive.WaitForFlag("move again");
                drive.OWTurn(45,.2);

                if(vuMark != null) {
                    if (vuMark == RelicRecoveryVuMark.LEFT) {
                        drive.Drive(4.8,0.2);
                    } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                        drive.Drive(18.8,0.2);
                    } else {
                        drive.Drive(11.8,0.2);
                    }
                }
                drive.Turn(-90,.2);
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
                if(vuMark != null) {
                    if (vuMark == RelicRecoveryVuMark.LEFT) {
                        drive.Drive(-41,0.2);
                    } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                        drive.Drive(-27,0.2);
                    } else {
                        drive.Drive(-34,0.2);
                    }
                }
                drive.SetFlag(arm, "Off Platform");
                drive.Pause(200);

                arm.WaitForFlag("Off Platform");
                arm.AbsoluteMotorMove(mtrLift, liftPos.ONE.getVal()+500, 0.5);

                drive.Turn(-89.5, 0.2);
            }else if(StartPos == 2){
                drive.Drive(-24,.2);
                drive.SetFlag(arm, "Off Platform");

                arm.WaitForFlag("Off Platform");
                arm.AbsoluteMotorMove(mtrLift, liftPos.AUTO.getVal(), 0.5);

                drive.Turn(86.5,.2);
                if(vuMark != null) {
                    if (vuMark == RelicRecoveryVuMark.LEFT) {
                        drive.Drive(-16.33,0.2);
                    } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                        drive.Drive(-2.33,0.2);
                    } else {
                        drive.Drive(-9.33,0.2);
                    }
                }
                //drive.Drive(-9.33,.2);
                drive.Turn(-88,.2);
            }
        }
//        if(vuMark != null) {
//            if (vuMark == RelicRecoveryVuMark.LEFT) {
//                drive.Turn(29, 0.2);
//                drive.Drive(-4.5, 0.2);
//                drive.SetFlag(glyph,"open grabber");
//            } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
//                drive.Turn(-24, 0.2);
//                drive.Drive(-4.75, 0.2);
//                drive.SetFlag(glyph,"open grabber");
//            } else {
//                drive.Turn(3.25,0.2);
//                drive.Drive(-4.5, 0.2);
//                drive.SetFlag(glyph,"open grabber");
//            }
//        }

        drive.Drive(-2.5, 0.2);
        drive.SetFlag(glyph,"open grabber");

        glyph.WaitForFlag("open grabber");
        glyph.ServoMove(srvGr1,GR1OPEN);
        glyph.ServoMove(srvGr2,GR2OPEN);
        glyph.Pause(200);
        glyph.AbsoluteMotorMove(mtrLift, liftPos.ONE.getVal(), 0.5);

        if(StartPos == 1) {
            glyph.Pause(200);
            glyph.SetFlag(drive, "enter pit");

            drive.WaitForFlag("enter pit");
            drive.Drive(7, 0.2);
            drive.Turn(175, 0.4);
            drive.Shimmy(-36, 0.5,500);
            drive.SetFlag(glyph,"grab that block");

            glyph.WaitForFlag("grab that block");
            glyph.ServoMove(srvGr1,GR1CLOSED);
            glyph.ServoMove(srvGr2,GR2CLOSED);
            glyph.Pause(300);
            glyph.SetFlag(drive,"go back");
            glyph.AbsoluteMotorMove(mtrLift,liftPos.AUTO.getVal(),0.6);

            drive.WaitForFlag("go back");
            drive.Pause(200);
            drive.Drive(10,0.2);
            drive.Turn(180,0.2);


        }else if(Alliance == RED) {
            glyph.SetFlag(drive,"you are good to go now");

            drive.WaitForFlag("you are good to go now");
            drive.Drive(10,0.2);
            drive.Turn(90,0.2);
            if(vuMark != null) {
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    drive.Drive(16.33,0.2);
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    drive.Drive(2.33,0.2);
                } else {
                    drive.Drive(9.33,0.2);
                }
            }
            drive.OWTurn(45,0.2);
            drive.SetFlag(glyph,"grab dat relic");

            glyph.AbsoluteMotorMove(mtrExtend,extendPos.RED_AUTO.getVal(),0.05);
            glyph.ServoMove(srvClaw,CLAWCLOSED);
            glyph.AbsoluteMotorMove(mtrExtend,extendPos.HOME.getVal(),0.05);
            glyph.SetFlag(drive,"watch those wrist rockets");

            drive.WaitForFlag("watch those wrist rockets");
            drive.Drive(-11,0.2);
        }



        if(ballArray[0] != null && ballArray[1] != null)
            telemetry.addData("ballArray", Arrays.toString(ballArray));
        else
            telemetry.addData("ballArray", "[null, null]");
        if(vuMark != null)
            telemetry.addData("vuMark", vuMark.toString());
        else
            telemetry.addData("vuMark", "null");

        telemetry.addData("voltage", getBatteryVoltage());


    }
}
