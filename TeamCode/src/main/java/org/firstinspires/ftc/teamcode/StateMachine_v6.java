package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.test.suitebuilder.annotation.Suppress;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

class StateMachine_v6 extends Subroutines_v13 {

    public String flag = "";
    int current_number = 0;                           //The order of a specific state//
    int state_in_progress = 1;                        //Which state in the list is to be run//
    boolean stateComplete = false;                    //DRIVE and TURN: Whether state_type:DRIVE or state_type:TURN is completed with assigned motion//
    private boolean oneShot = false;
    private final double gearRatio = 1.0;
    private final double countsPerRev = 560;
    private final double wheelDiameter = 4.166666666666667;                      //double wheelDiameter = 4.19;
    private final double turnDiameter = 15.45;        // private final double turnDiameter = 13.95;

    @Override
    public String toString(){
        return "cn:"+current_number+"\tSIP:"+state_in_progress+"\tFlag:"+flag;
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
    }

    void initializeMachine(){
        current_number = 0;
        stateComplete = false;
    }

    void incrementState(){
        state_in_progress++;
        stateComplete = true;
    }

    void SetFlag(StateMachine_v6 receiver, String key){
        if(next_state_to_execute()){
            receiver.flag = key;
            incrementState();
        }
    }

    void WaitForFlag(String key){
        if(next_state_to_execute()){
            if(flag.equals(key)){
                incrementState();
            }
        }
    }

    void ClearFlag(){
        if(next_state_to_execute()){
            flag = "";
            incrementState();
        }
    }

    //manages current state number and compares it to state in progress
    boolean next_state_to_execute() {
        return (state_in_progress == ++current_number && !stateComplete);
    }

    boolean previous_state_running(){
        return state_in_progress == current_number;
    }

    @Deprecated
    boolean next_state_to_execute(boolean filler) {
        current_number++;
        return (state_in_progress == current_number);
    }

    void Drive(double distance, double speed) {
        if(next_state_to_execute()) {
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
                    incrementState();
                }
            }
        }
    }

    void Turn(double degrees, double speed) {
        if(next_state_to_execute()) {
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

                if (have_drive_encoders_reached( -targetDegrees, targetDegrees) || driveFinished) {
                    if (!driveFinished) reset_drive_encoders();
                    driveFinished = true;
                    set_drive_power(0.0f, 0.0f);
                    if (have_drive_encoders_reset()) {
                        driveFinished = false;
                        incrementState();
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
                        incrementState();
                    }
                }
            }
        }
    }

    void OWTurn(double degrees, double speed) {
        if(next_state_to_execute()) {
            double wheelCircumference = wheelDiameter * Math.PI;//
            double turnCircumference = turnDiameter * 2 * Math.PI;
            double turnDistance = turnCircumference / wheelCircumference;
            double degreeValue = turnDistance / 360;
            double revs = degreeValue * Math.abs(degrees);
            double targetDegrees = gearRatio * revs * countsPerRev;

            if(speed < 0) targetDegrees*=-1;

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
                        incrementState();
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
                        incrementState();
                    }
                }
            }
        }
    }

    public void GyroTurn(double degrees, double speed){
        if (next_state_to_execute()) {
            DcMotor.RunMode r = mtrLeftDrive.getMode();
            run_using_drive_encoders();
            Orientation o = IMUnav.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            double z = o.thirdAngle;
            telemetry.addData("rot", z);
            if(Math.abs(z) + 1 <= Math.abs(degrees)){
                set_drive_power(-speed*Math.signum(degrees), speed*Math.signum(degrees));
            }else{
                set_drive_power(0,0);
                reset_drive_encoders();
                mtrLeftDrive.setMode(r);
                mtrRightDrive.setMode(r);
                incrementState();
            }
        }
    }

    void Pause(long milliseconds) {
        if (next_state_to_execute()) {
            if (waitHasFinished(milliseconds)) {
                incrementState();
            }
        }
    }

    void ResetDrive() {
        if (next_state_to_execute()) {
            reset_drive_encoders();
            if (get_encoder_count(mtrLeftDrive) == 0 && get_encoder_count(mtrRightDrive) == 0) {
                run_using_drive_encoders();
                incrementState();
            }
        }
    }

    public void ResetEncoder(DcMotor motor) {
        if (next_state_to_execute()) {
            reset_encoders(motor);
            if (get_encoder_count(motor) == 0) {
                run_using_encoder(motor);
                incrementState();
            }
        }
    }

    public void Stop() {
        if (next_state_to_execute()) {
            set_drive_power(0.0f, 0.0f);
            incrementState();
        }
    }

    public void ServoMove(Servo servo, double position) {
        if (next_state_to_execute()) {
            set_position(servo, position);
            incrementState();
        }
    }

    public void ServoMove(CRServo CRservo, double position) {
        if (next_state_to_execute()) {
            set_position(CRservo, position);
            incrementState();
        }
    }

    @Deprecated
    public void WriteI2C(StateMachine_v6 object, I2cDevice device, I2cAddr address, int register, int value){
        if(next_state_to_execute()){

            device.enableI2cWriteMode(address,register,value);
            if(device.isI2cPortInWriteMode()){
                incrementState();
            }

        }

    }

    public void WriteColorValues(ColorSensor colorSensor){
        if(next_state_to_execute()){
            String colorVal = getColorVal(colorSensor, "red") + ", " + getColorVal(colorSensor, "blue")
                    + ", " + getColorVal(colorSensor, "green");
            writeToFile(colorVal);
            incrementState();
        }
    }

    public void MotorMove(DcMotor motor, long encCount, double power){
        if(next_state_to_execute()) {
            DcMotor.RunMode r = motor.getMode();
            run_to_position(motor);
            set_encoder_target(motor, (int) encCount);
            set_power(motor, power);
            if(has_encoder_reached(motor, encCount)) {
                set_power(motor, 0);
                reset_encoders(motor);
                motor.setMode(r);
                incrementState();
            }
        }
    }

    public void FlipArm(long encCount, double power){
        if(next_state_to_execute()) {
            run_to_position(mtrArmFlip);
            set_encoder_target(mtrArmFlip, (int) encCount);
            set_power(mtrArmFlip, power);
            if(has_encoder_reached(mtrArmFlip, encCount)) {
                set_power(mtrArmFlip, 0);
                incrementState();
            }
        }
    }

//    // TODO: 9/19/2017
//    public void Shift(StateMachine_v5 object, double positionX, double positionY){
//
//        if(next_state_to_execute(object)){
//
//
//            double targetXCounts = positionX * 1120;
//            double targetYCounts = positionY * 1120;
//
//            if( targetXCounts > get_encoder_count(mtrShiftX)){
//
//            }
//
//
//
//        }
//
//    }

    private int x = 0;
    private int y = 0;
    private int X = 0;
    private int Y = 0;
    private int x_tol = 0;
    private int y_tol = 0;
    private int counter = 0;
    private boolean OS = false;
    private boolean broken = false;
    private int wave = 0;

    @Deprecated
    void resetVariables(StateMachine_v6 object){
        object.x = 0; object.y = 0; object.X = 0; object.Y = 0; object.counter = 0; object.OS = false;
        object.x_tol = 0; object.y_tol = 0; object.broken = false;
    }

    @Deprecated
    void resetMachine(StateMachine_v6 object){
        resetVariables(object); object.wave = 0;
        object.current_number = 0; object.state_in_progress = 1;
    }

    @Deprecated
    int stringVal(String string, int val){
        char stringVal = string.charAt(val);
        String brokenString = "" + stringVal;
        return Integer.parseInt(brokenString);
    }




}