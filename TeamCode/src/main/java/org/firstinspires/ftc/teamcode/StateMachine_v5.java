package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.CameraBridgeViewBase;
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

import static android.content.ContentValues.TAG;
import static org.opencv.imgproc.Imgproc.minEnclosingCircle;


class StateMachine_v5 extends Subroutines_v13 {

    public String flag = "";
    int current_number = 0;                           //The order of a specific state//
    int state_in_progress = 1;                        //Which state in the list is to be run//
    boolean stateComplete = false;                    //DRIVE and TURN: Whether state_type:DRIVE or state_type:TURN is completed with assigned motion//
    private boolean oneShot = false;
    private final double gearRatio = 1.0;
    private final double countsPerRev = 560;
    private final double wheelDiameter = 4.166666666666667;                      //double wheelDiameter = 4.19;
    private final double turnDiameter = 15.45;        // private final double turnDiameter = 13.95;

    VuforiaLocalizer vl;
    VuforiaTrackable relicTemplate;
    RelicRecoveryVuMark vuMark;

    BallColor[] ballArray = {null, null};


    @Override
    public String toString() {
        return "cn:" + current_number + "\tSIP:" + state_in_progress + "\tFlag:" + flag;
    }

    boolean waitHasFinished(long milliseconds) {
        boolean returnVal = false;

        if (initOS) {
            systemTime = System.nanoTime() / 1000000;
            initOS = false;
        } else if ((System.nanoTime() / 1000000) - systemTime >= milliseconds) {
            initOS = true;
            returnVal = true;
        }

        return returnVal;
    }

    @Override
    public void init() {
        super.init();
        int vuforiaID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(vuforiaID);
        params.vuforiaLicenseKey = "AXlr9dj/////AAAAGRujCnGOL0aIpjtd4y5IK2sFwI4jstOeOlytTrPr3jzeQQ9tGEHgLmuGdxzuxGNY5641pyXeeyJHccL+I4QCZq8Sodm5DAUBsAQQ9ox1EY3+KNfZISN06k1IqDf7YaRXhE02j+7aE4Apnm3Hvn9V5CDKSTgOq73eJId9uzHkuNaIx+UDV4fRS1HK5L6dSGmIw3+RW7uaFdBF0E2bvWzzpZv51KFzw5oy/9qFB9r6ke5B5Gd2zw9JjafwudFSpLVCiNzIreoTaIFYVMmGMuIFIT4C6oC13EcvbNl5CFpl+irLqhSI//nlL2L2DKxKtW5UTQqNBlOSBdTxWR/kSN12edlwOu0kFgzhKBFapn3KHC0V";
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vl = ClassFactory.createVuforiaLocalizer(params);
        VuforiaTrackables relicTrackables = this.vl.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTrackables.activate();
    }

    void initializeMachine(StateMachine_v5 object) {
        object.current_number = 0;
        object.stateComplete = false;
    }

    void incrementState(StateMachine_v5 obj) {
        obj.state_in_progress++;
        obj.stateComplete = true;
    }

    void SetFlag(StateMachine_v5 object, StateMachine_v5 receiver, String key) {
        if (next_state_to_execute(object)) {
            receiver.flag = key;
            incrementState(object);
        }
    }

    void WaitForFlag(StateMachine_v5 object, String key) {
        if (next_state_to_execute(object)) {
            if (object.flag.equals(key)) {
                incrementState(object);
            }
        }
    }

    void ClearFlag(StateMachine_v5 object) {
        if (next_state_to_execute(object)) {
            object.flag = "";
            incrementState(object);
        }
    }

    //manages current state number and compares it to state in progress
    boolean next_state_to_execute(StateMachine_v5 object) {
        object.current_number++;
        return (object.state_in_progress == object.current_number && !object.stateComplete);
    }

    boolean previous_state_running(StateMachine_v5 object) {
        return object.state_in_progress == object.current_number;
    }

    boolean next_state_to_execute(StateMachine_v5 object, boolean filler) {
        object.current_number++;
        return (object.state_in_progress == object.current_number);
    }

    void Drive(StateMachine_v5 object, double distance, double speed) {
        if (next_state_to_execute(object)) {
            double wheelCircumference = wheelDiameter * Math.PI;
            double revs = distance / wheelCircumference;
            double targetDegrees = gearRatio * revs * countsPerRev;

            if (speed < 0) {
                targetDegrees = Math.abs(targetDegrees) * -1;
            }

            if (!driveFinished) {
                run_drive_to_position();
                set_drive_target((int) targetDegrees, (int) targetDegrees);
                set_drive_power(speed, speed);
            }

            if (have_drive_encoders_reached(targetDegrees, targetDegrees) || driveFinished) { //if move is finished
                if (!driveFinished) reset_drive_encoders(); //if encoders have not been reset,
                driveFinished = true;                       //reset encoders
                set_drive_power(-0.0f, -0.0f);//stop robot
                if (have_drive_encoders_reset()) {//if encoders have actually reset,
                    driveFinished = false;  //move on to next method
                    incrementState(object);
                }
            }
        }
    }

    void Turn(StateMachine_v5 object, double degrees, double speed) {
        if (next_state_to_execute(object)) {
            double wheelCircumference = wheelDiameter * Math.PI;//
            double turnCircumference = turnDiameter * Math.PI;
            double turnDistance = turnCircumference / wheelCircumference;
            double degreeValue = turnDistance / 360;
            double revs = degreeValue * Math.abs(degrees);
            double targetDegrees = gearRatio * revs * countsPerRev;
            //double rampSpeed = rampTurn(targetDegrees, speed);

            if (degrees < 0) {
                if (!driveFinished) {
                    run_drive_to_position();
                    set_drive_target((int) -targetDegrees, (int) targetDegrees);
                    set_drive_power(-speed, speed);
                }

                if (have_drive_encoders_reached(-targetDegrees, targetDegrees) || driveFinished) {
                    if (!driveFinished) reset_drive_encoders();
                    driveFinished = true;
                    set_drive_power(0.0f, 0.0f);
                    if (have_drive_encoders_reset()) {
                        driveFinished = false;
                        incrementState(object);
                    }
                }
            } else if (degrees > 0) {
                if (!driveFinished) {
                    run_drive_to_position();
                    set_drive_target((int) targetDegrees, (int) -targetDegrees);
                    set_drive_power(speed, -speed);
                }

                if (have_drive_encoders_reached(targetDegrees, -targetDegrees) || driveFinished) {
                    if (!driveFinished) reset_drive_encoders();
                    driveFinished = true;
                    set_drive_power(0.0f, 0.0f);
                    if (have_drive_encoders_reset()) {
                        driveFinished = false;
                        incrementState(object);
                    }
                }
            }
        }
    }

    void OWTurn(StateMachine_v5 object, double degrees, double speed) {
        if (next_state_to_execute(object)) {
            double wheelCircumference = wheelDiameter * Math.PI;//
            double turnCircumference = turnDiameter * 2 * Math.PI;
            double turnDistance = turnCircumference / wheelCircumference;
            double degreeValue = turnDistance / 360;
            double revs = degreeValue * Math.abs(degrees);
            double targetDegrees = gearRatio * revs * countsPerRev;

            if (speed < 0) targetDegrees *= -1;

            if (degrees < 0) {
                if (!driveFinished) {
                    run_using_encoder(mtrRightDrive);
                    run_to_position(mtrLeftDrive);
                    set_encoder_target(mtrLeftDrive, (int) targetDegrees);
                    set_power(mtrLeftDrive, speed);
                    set_power(mtrRightDrive, 0);
                }

                if (has_encoder_reached(mtrLeftDrive, (int) targetDegrees) || driveFinished) {
                    if (!driveFinished) reset_drive_encoders();
                    driveFinished = true;
                    set_drive_power(0.0f, 0.0f);
                    if (have_drive_encoders_reset()) {
                        driveFinished = false;
                        incrementState(object);
                    }
                }
            } else if (degrees > 0) {
                if (!driveFinished) {
                    run_using_encoder(mtrLeftDrive);
                    run_to_position(mtrRightDrive);
                    set_encoder_target(mtrRightDrive, (int) targetDegrees);
                    set_power(mtrRightDrive, speed);
                    set_power(mtrLeftDrive, 0);
                }

                if (has_encoder_reached(mtrRightDrive, (int) targetDegrees) || driveFinished) {
                    if (!driveFinished) reset_drive_encoders();
                    driveFinished = true;
                    set_drive_power(0.0f, 0.0f);
                    if (have_drive_encoders_reset()) {
                        driveFinished = false;
                        incrementState(object);
                    }
                }
            }
        }
    }

    public void GyroTurn(StateMachine_v5 obj, double degrees, double speed) {
        if (next_state_to_execute(obj)) {
            DcMotor.RunMode r = mtrLeftDrive.getMode();
            run_using_drive_encoders();
            Orientation o = IMUnav.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            double z = o.thirdAngle;
            telemetry.addData("rot", z);
            if (Math.abs(z) + 1 <= Math.abs(degrees)) {
                set_drive_power(-speed * Math.signum(degrees), speed * Math.signum(degrees));
            } else {
                set_drive_power(0, 0);
                reset_drive_encoders();
                mtrLeftDrive.setMode(r);
                mtrRightDrive.setMode(r);
                incrementState(obj);
            }
        }
    }

    void Pause(StateMachine_v5 object, long milliseconds) {
        if (next_state_to_execute(object)) {
            if (waitHasFinished(milliseconds)) {
                incrementState(object);
            }
        }
    }

    void ResetDrive(StateMachine_v5 object) {
        if (next_state_to_execute(object)) {
            reset_drive_encoders();
            if (get_encoder_count(mtrLeftDrive) == 0 && get_encoder_count(mtrRightDrive) == 0) {
                run_using_drive_encoders();
                incrementState(object);
            }
        }
    }

    public void ResetEncoder(StateMachine_v5 object, DcMotor motor) {
        if (next_state_to_execute(object)) {
            reset_encoders(motor);
            if (get_encoder_count(motor) == 0) {
                run_using_encoder(motor);
                incrementState(object);
            }
        }
    }

    public void Stop(StateMachine_v5 object) {
        if (next_state_to_execute(object)) {
            set_drive_power(0.0f, 0.0f);
            incrementState(object);
        }
    }

    public void ServoMove(StateMachine_v5 object, Servo servo, double position) {
        if (next_state_to_execute(object)) {
            set_position(servo, position);
            incrementState(object);
        }
    }

    public void ServoMove(StateMachine_v5 object, CRServo CRservo, double position) {
        if (next_state_to_execute(object)) {
            set_position(CRservo, position);
            incrementState(object);
        }
    }

    public void WriteI2C(StateMachine_v5 object, I2cDevice device, I2cAddr address, int register, int value) {
        if (next_state_to_execute(object)) {

            device.enableI2cWriteMode(address, register, value);
            if (device.isI2cPortInWriteMode()) {

            }

        }

    }

    public void WriteColorValues(StateMachine_v5 object, ColorSensor colorSensor) {
        if (next_state_to_execute(object)) {
            String colorVal = getColorVal(colorSensor, "red") + ", " + getColorVal(colorSensor, "blue")
                    + ", " + getColorVal(colorSensor, "green");
            writeToFile(colorVal);
        }
    }

    public void MotorMove(StateMachine_v5 s, DcMotor motor, long encCount, double power) {
        if (next_state_to_execute(s)) {
            DcMotor.RunMode r = motor.getMode();
            run_to_position(motor);
            set_encoder_target(motor, (int) encCount);
            set_power(motor, power);
            if (has_encoder_reached(motor, encCount)) {
                set_power(motor, 0);
                reset_encoders(motor);
                motor.setMode(r);
                incrementState(s);
            }
        }
    }



    public void FlipArm(StateMachine_v5 s, long encCount, double power) {
        if (next_state_to_execute(s)) {
            run_to_position(mtrArmFlip);
            set_encoder_target(mtrArmFlip, (int) encCount);
            set_power(mtrArmFlip, power);
            if (!mtrArmFlip.isBusy()) {
                set_power(mtrArmFlip, 0);
                incrementState(s);
            }
        }
    }

    public int x = 0;
    public int y = 0;
    public int X = 0;
    public int Y = 0;
    public int x_tol = 0;
    public int y_tol = 0;
    public int counter = 0;
    public boolean OS = false;
    public boolean broken = false;
    public int wave = 0;

    void resetVariables(StateMachine_v5 object) {
        object.x = 0;
        object.y = 0;
        object.X = 0;
        object.Y = 0;
        object.counter = 0;
        object.OS = false;
        object.x_tol = 0;
        object.y_tol = 0;
        object.broken = false;
    }

    void resetMachine(StateMachine_v5 object) {
        resetVariables(object);
        object.wave = 0;
        object.current_number = 0;
        object.state_in_progress = 1;
    }

    int stringVal(String string, int val) {
        char stringVal = string.charAt(val);
        String brokenString = "" + stringVal;
        return Integer.parseInt(brokenString);
    }

    enum BallColor {
        BLUE,RED
    }

}