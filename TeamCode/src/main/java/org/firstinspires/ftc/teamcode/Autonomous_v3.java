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
//@Autonomous(name = "AutoNew", group = "Final")
public class Autonomous_v3 extends StateMachine_v7 {
    int question_number = 1;
    boolean btnOS;

    byte Alliance = 0;
    int StartPos = 0;
    long StartPause = 0;
    boolean GrabRelic = false ,EnterPit = false;

    Orientation axes;

    StateMachine_v7 drive = new StateMachine_v7(),
            arm = new StateMachine_v7(),
            glyph = new StateMachine_v7(),
            vision = new StateMachine_v7();

    @Override
    public void init() {
        super.init();
        srvLevel.setPosition(LEVELINIT);



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

        if(gamepad2.guide && gamepad2.back) reset_encoders(mtrLift);

        if(!gamepad2.right_stick_button) set_power(mtrLift, .75*gamepad2.right_stick_y);
            else set_power(mtrLift, .1*gamepad2.right_stick_y);

        switch (question_number) {
            case 1:
                telemetry.addData("Alliance: a=blue b=red", "");
                if ((gamepad1.a ^ gamepad1.b) && !gamepad1.start && !btnOS) {
                    Alliance = (gamepad1.a) ? BLUE : RED;
                    btnOS = true;
                } else if (!gamepad1.a && !gamepad1.b && !gamepad1.start && btnOS) {
                    btnOS = false;
                    question_number++;
                }
                break;
            case 2:
                telemetry.addData("StartPos: a=1 b=2", "");
                if ((gamepad1.a ^ gamepad1.b) && !gamepad1.start && !btnOS) {
                    StartPos = (gamepad1.a) ? 1 : 2;
                    btnOS = true;
                } else if (!gamepad1.a && !gamepad1.b && !gamepad1.start && btnOS) {
                    btnOS = false;
                    question_number++;
                }
                break;
            case 3:
                if(StartPos == 1){
                    telemetry.addData("Enter Pit: a=yes b=no", "");
                    if ((gamepad1.a ^ gamepad1.b) && !gamepad1.start && !btnOS) {
                        EnterPit = gamepad1.a;
                        btnOS = true;
                    } else if (!gamepad1.a && !gamepad1.b && !gamepad1.start && btnOS) {
                        btnOS = false;
                        question_number++;
                    }
                }
                if(StartPos == 2){
                    telemetry.addData("Grab Relic: a=yes b=no", "");
                    if ((gamepad1.a ^ gamepad1.b) && !gamepad1.start && !btnOS) {
                        GrabRelic = gamepad1.a;
                        btnOS = true;
                    } else if (!gamepad1.a && !gamepad1.b && !gamepad1.start && btnOS) {
                        btnOS = false;
                        question_number++;
                    }
                }
                break;
            case 4:
                telemetry.addData("Add time d_up = +1sec, confirm = a", "");
                telemetry.addData("Beginning Pause",StartPause);
                if (gamepad1.dpad_up ^ gamepad1.dpad_down && !btnOS) {
                    if(gamepad1.dpad_up) StartPause += 1000;
                    if(gamepad1.dpad_down) StartPause -= 1000;
                    btnOS = true;
                } else if (!gamepad1.dpad_down && !gamepad1.dpad_up && !gamepad1.a) {
                    btnOS = false;
                } else if (gamepad1.a && !btnOS) {
                    current_number++;
                }
                break;
            case 5:
                telemetry.addData("Ready to Begin", "");
                break;
        }

        if (StartPos == 0 || Alliance == 0)
            telemetry.addData("WARNING YOU DON'T HAVE ALL INITIALIZATION PARAMETERS IN", "");

    }

    @Override
    public void start() {
        reset_drive_encoders();
        reset_encoders(mtrExtend);
        reset_encoders(mtrLift);
    }

    //just in case if I made changes

    @Override
    public void loop() {
        vision.initializeMachine();
        drive.initializeMachine();
        arm.initializeMachine();
        glyph.initializeMachine();

        drive.ServoMove(srvPhone, CAM_VUMARK);
        drive.Pause(200);
        drive.SetFlag(vision, "Read Relic");

        arm.ServoMove(srvGr1,GR1AUTO);
        arm.ServoMove(srvGr2,GR2AUTO);
        arm.Pause(750);
        arm.AbsoluteMotorMove(mtrLift,liftPos.TWO.getVal(),0.8);

        vision.WaitForFlag("Read Relic");
        vision.ProcessRelic();
//        if(vision.next_state_to_execute()) {
//            if(vuMark!=null) {
//                drive.incrementState();
//            }
//        }
        vision.SetFlag(drive, "Relic Read");

        drive.WaitForFlag("Relic Read");
        drive.ServoMove(srvPhone, CAM_JEWELS);
        drive.Pause(1000);
        drive.SetFlag(vision, "Read Jewels");

        vision.WaitForFlag("Read Jewels");
        vision.ProcessJewels(vuforiaFrameToMat());
//        if(vision.next_state_to_execute()){
//            if(ballArray[0]!=null&&ballArray[1]!=null){
//                vision.incrementState();
//            }
//        }
        vision.SetFlag(arm, "Jewels Read");

        arm.WaitForFlag("Jewels Read");
        arm.ServoMove(srvLR, LR_CENTER);
        arm.ServoIncrementalMove(srvUD, UD_DOWN, 0.1);
        arm.Pause(500);


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
                        drive.Drive(28,0.2);
                    } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                        drive.Drive(42.75,0.2);
                    } else {
                        drive.Drive(35.75,0.2);
                    }
                }
                drive.SetFlag(arm,"Off Platform");
                drive.Pause(200);
//                if(drive.next_state_to_execute()) {
//                    drive.useThisForTesting = true;
//                }

                arm.WaitForFlag("Off Platform");
                arm.AbsoluteMotorMove(mtrLift,liftPos.AUTO.getVal(),.4);

                drive.Turn(-88, .2);
            }


            else if(StartPos == 2){
                drive.Drive(26,.2);
                drive.Drive(-2,.2);

                drive.SetFlag(arm, "Off Platform");
                if(GrabRelic) {
                    drive.Drive(-4,.2);
                    drive.ServoMove(srvGr1,GR1OPEN);
                    drive.ServoMove(srvGr2,GR2OPEN);
                    drive.Pause(300);
                    drive.OWTurn(-53.5,.2);

                    drive.SetFlag(glyph, "Grab Relic");

                    glyph.WaitForFlag("Grab Relic");
                    glyph.AbsoluteMotorMove(mtrExtend, -1000, -1);
                    glyph.ServoMove(srvClaw, CLAWOPEN);
                    glyph.ServoMove(srvLevel, LEVELDOWN);
                    if(glyph.next_state_to_execute()){
                        set_power(mtrExtend, -0.45);
                        glyph.incrementState();
                    }
                    if(glyph.next_state_to_execute()){
                        if(isWithin(get_encoder_count(mtrExtend), -1400+100, -1400-100)) {
                            glyph.incrementState();
                        }
                    }
                    glyph.ServoMove(srvLevel, LEVELPARTIALDOWN);
                    glyph.Pause(300);
                    if(glyph.next_state_to_execute()){
                        set_power(mtrExtend, 0.0);
                        set_position(srvClaw, CLAWCLOSED);
                        glyph.incrementState();
                    }
                    glyph.Pause(750);
                    if(glyph.next_state_to_execute()){
                        set_power(mtrExtend, 0.3);
                        glyph.incrementState();
                    }
                    glyph.AbsoluteMotorMove(mtrExtend, extendPos.HOME.getVal(), 0.5);
                    glyph.ServoMove(srvLevel, LEVELDOWN);
                    glyph.SetFlag(drive, "move again");

                    drive.WaitForFlag("move again");
                    drive.OWTurn(-53.5, -.2);
                    drive.ServoMove(srvGr1,GR1AUTO);
                    drive.ServoMove(srvGr2,GR2AUTO);
                    drive.Drive(5,.2);
                }
                drive.ServoMove(srvGr1, GR1CLOSED);
                drive.ServoMove(srvGr2, GR2CLOSED);
                drive.ServoMove(srvGr1, GR1AUTO);
                drive.ServoMove(srvGr2, GR2AUTO);
                drive.Turn(88,0.2);
                drive.ServoMove(srvLevel, LEVELUP);
                arm.WaitForFlag("Off Platform");
                arm.AbsoluteMotorMove(mtrLift,liftPos.AUTO.getVal()-150,.4);

                if(vuMark != null) {
                    if (vuMark == RelicRecoveryVuMark.LEFT) {
                        drive.Drive(-3.5,0.2);
                    } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                        drive.Drive(-18.3,0.2);
                    } else {
                        drive.Drive(-11.8,0.2);
                    }
                }
                drive.AbsoluteMotorMove(mtrLift,liftPos.CARRY.getVal(),0.3);
                drive.Turn(87,.2);
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
                drive.Drive(2,.2);
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
                drive.AbsoluteMotorMove(mtrLift,liftPos.CARRY.getVal(),0.3);
                drive.Turn(-88,.2);
            }
        }

        drive.Drive((StartPos==2)?-6:-3.5, 0.2);
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
            if(EnterPit) {
                drive.ServoMove(srvGr1, GR1AUTO);
                drive.ServoMove(srvGr2,GR2AUTO);
                drive.SetFlag(glyph,"da way");
                glyph.WaitForFlag("da way");
                glyph.AbsoluteMotorMove(mtrLift,liftPos.AUTO.getVal(),0.5);
                drive.Drive(-8,0.5);
                drive.Turn(-55,0.3);
                drive.Drive(-17,0.5);
                drive.Turn(75,75);
                drive.ServoMove(srvGr2,GR2OPEN);
                drive.SetFlag(glyph,"oscillate");

                glyph.WaitForFlag("oscillate");
                if(drive.next_state_to_execute()){
                    drive.timerToggle.reset();
                    drive.incrementState();
                }
                glyph.ServoToggle(snsBtnGlyph.isPressed() || drive.timerToggle.getElapsedTime()>5000);
                drive.DriveWithCondition(-15,0.5,snsBtnGlyph.isPressed() || drive.timerToggle.getElapsedTime()>5000);

//                if(getBatteryVoltage() < 10) {
//                    drive.incrementState();
//                }

                //drive.Shimmy(-24, 0.2, 1000);
                drive.SetFlag(glyph, "grab that block");

                glyph.WaitForFlag("grab that block");
                glyph.ServoMove(srvGr1, GR1AUTO);
                glyph.ServoMove(srvGr2, GR2AUTO);
                glyph.Pause(300);
                glyph.SetFlag(drive, "go back");
                glyph.AbsoluteMotorMove(mtrLift, liftPos.TWO.getVal()-300, 0.6);

                drive.WaitForFlag("go back");
                drive.Pause(200);
                drive.Drive(6,0.2);
                drive.Turn(-45, 0.2);
                drive.Drive(-16,0.2);
                //drive.RetraceEncoders(0.2);
                drive.ServoMove(srvGr1,GR1OPEN);
                drive.ServoMove(srvGr2,GR2OPEN);

            }

        }else if(Alliance == RED && GrabRelic) {
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

            //glyph.AbsoluteMotorMove(mtrExtend,extendPos.RED_AUTO.getVal(),0.05);
            glyph.ServoMove(srvClaw,CLAWCLOSED);
            glyph.AbsoluteMotorMove(mtrExtend,extendPos.HOME.getVal(),0.05);
            glyph.SetFlag(drive,"watch those wrist rockets");

            drive.WaitForFlag("watch those wrist rockets");
            drive.Drive(-11,0.2);

            drive.Drive(6, 0.5);
            if(StartPos == 1)drive.Turn(180, 0.5);
            else{
                drive.Turn(-90, 0.5);
                drive.ServoMove(srvLevel, LEVELDOWN);
                drive.Drive(-5,0.5);
                drive.Turn(-47, 0.5);
                if(vuMark != RelicRecoveryVuMark.LEFT) drive.Drive(7,0.5);
            }

        }



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
