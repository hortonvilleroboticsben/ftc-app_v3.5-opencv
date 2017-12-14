package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.RobotLog;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;


class Subroutines_v13 extends OpMode {
    //Variable initialization//
    long systemTime = 0;             //System time used to determine how much time has passed for waitHasFinished()//
    boolean initOS = true;           //Oneshot used for waitHasFinished//

    private boolean warning_generated;
    private String warning_message;

    public final byte BLUE = 1;
    public final byte RED = -1;
    boolean driveFinished = false;


    double rangeVal[] = {0,0};
    final byte FILTERED = 1;
    final byte CACHE = 0;

    final double GR1CLOSED = 0.143;
    final double GR1OPEN = 0.45;
    final double GR2CLOSED = .383;
    final double GR2OPEN = 0.7;


    @Override
    public void init() {

        warning_generated = false;
        warning_message = "Can't find:  ";

        Gamepad gamepad1 = new Gamepad();
        Gamepad gamepad2 = new Gamepad();

        mtrLeftDrive = addMotor(MotorLocation.LEFT,"mtrLeftDrive");
        mtrRightDrive = addMotor(MotorLocation.RIGHT,"mtrRightDrive");
        mtrLift = addMotor(MotorLocation.SHIFT,"mtrLift");
        mtrFlip = addMotor(MotorLocation.SHIFT,"mtrFlip");
        mtrArmFlip = addMotor(MotorLocation.SHIFT,"mtrArmFlip");
        mtrArmSpin = addMotor(MotorLocation.SHIFT,"mtrArmSpin");

        //snsColorRight = addColorSensor(0x2a,true,"snsColorRight");
        try {
            snsColorRight = hardwareMap.colorSensor.get("snsColorRight");
            snsColorRight.setI2cAddress(I2cAddr.create8bit(0x2a));
            snsColorRight.enableLed(true);
        } catch (Exception p_exception) {
            addWarningMessage("snsColorRight");
            RobotLog.i(p_exception.getLocalizedMessage());
            snsColorRight = null;
        }
        //snsColorLeft = addColorSensor(0x1a,true,"snsColorLeft");
        try {
            snsColorLeft = hardwareMap.colorSensor.get("snsColorLeft");
            snsColorLeft.setI2cAddress(I2cAddr.create8bit(0x1a));
            snsColorLeft.enableLed(true);
        } catch (Exception p_exception) {
            addWarningMessage("snsColorLeft");
            RobotLog.i(p_exception.getLocalizedMessage());
            snsColorLeft = null;
        }

        srvGr1 = addServo(GR1OPEN, "srvGr1");
        srvGr2 = addServo(GR2OPEN, "srvGr2");
        srvExtend = addCRServo("srvExtend");
        srvClaw = addServo(0.1,"srvClaw");
        srvShift = addCRServo("srvShift");
        srvLevel = addServo(0.0,"srvLevel");

        try{
            IMUnav = hardwareMap.get(BNO055IMU.class, "imu");
            Parameters p = new Parameters();
            p.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            p.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            p.calibrationDataFile = "BNO055IMUCalibration.json";
            try{IMUnav.initialize(p);}
            catch(Exception e){}
            IMUnav.isSystemCalibrated();
        }catch(Exception p_exception){
            addWarningMessage("imu");
            RobotLog.i(p_exception.getLocalizedMessage());
            IMUnav = null;
        }

        update_telemetry();

    }
    @Override
    public void loop() {}

    public DcMotor addMotor(MotorLocation side, String configName){
        DcMotor m;
        try {
            m = hardwareMap.dcMotor.get(configName);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if(side == MotorLocation.RIGHT)m.setDirection(DcMotorSimple.Direction.REVERSE);
            if(side == MotorLocation.LEFT)m.setDirection(DcMotorSimple.Direction.FORWARD);
            reset_encoders(m);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        } catch (Exception p_exception) {
            addWarningMessage(configName);
            RobotLog.i(p_exception.getLocalizedMessage());
            m = null;
        }
        return m;
    }
    public enum MotorLocation {
        LEFT,RIGHT,SHIFT
    }
    public Servo addServo(double pos, String configName){
        Servo srv;
        try {
            srv = hardwareMap.servo.get(configName);
            srv.setPosition(pos);
        } catch (Exception p_exception) {
            addWarningMessage(configName);
            RobotLog.i(p_exception.getLocalizedMessage());
            srv = null;
        }
        return srv;
    }
    public CRServo addCRServo(String configName){
        CRServo srv;
        try {
            srv = hardwareMap.crservo.get(configName);
            srv.setPower(0.0);
        } catch (Exception p_exception) {
            addWarningMessage(configName);
            RobotLog.i(p_exception.getLocalizedMessage());
            srv = null;
        }
        return srv;
    }
    public ColorSensor addColorSensor(int address, boolean LED, String configName){
        ColorSensor sns;
        try {
            sns = hardwareMap.colorSensor.get(configName);
            sns.setI2cAddress(I2cAddr.create8bit(address));
            sns.enableLed(LED);
        } catch (Exception p_exception) {
            addWarningMessage(configName);
            RobotLog.i(p_exception.getLocalizedMessage());
            sns = null;
        }
        return sns;
    }
    public ColorSensor addColorSensor(boolean LED, String configName){
        ColorSensor sns;
        try {
            sns = hardwareMap.colorSensor.get(configName);
            sns.enableLed(LED);
        } catch (Exception p_exception) {
            addWarningMessage(configName);
            RobotLog.i(p_exception.getLocalizedMessage());
            sns = null;
        }
        return sns;
    }
    public OpticalDistanceSensor addODSSensor(String configName){
        OpticalDistanceSensor sns;
        try {
            sns = hardwareMap.opticalDistanceSensor.get(configName);
        } catch (Exception p_exception) {
            addWarningMessage(configName);
            RobotLog.i(p_exception.getLocalizedMessage());
            sns = null;
        }
        return sns;
    }
    public TouchSensor addTouchSensor(String configName){
        TouchSensor sns;
        try {
            sns = hardwareMap.touchSensor.get(configName);
        } catch (Exception p_exception) {
            addWarningMessage(configName);
            RobotLog.i(p_exception.getLocalizedMessage());
            sns = null;
        }
        return sns;
    }
    public BNO055IMU addIMUSensor(String configName) {

        BNO055IMU imu = null;
        try{
            imu = hardwareMap.get(BNO055IMU.class, configName);
            Parameters p = new Parameters();
            p.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            p.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            p.calibrationDataFile = "BNO055IMUCalibration.json";
            imu.initialize(p);
            imu.isSystemCalibrated();
        }catch(Exception p_exception){
            addWarningMessage(configName);
            RobotLog.i(p_exception.getLocalizedMessage());
        }
        return imu;

    }


    public void addWarningMessage(String message) {
        if (warning_generated) {
            warning_message += ", ";
        }
        warning_generated = true;
        warning_message += message;
    }
    private boolean a_warning_generated() {
        return warning_generated;
    }
    private String getWarning_message() {
        return warning_message;
    }
    private void update_telemetry() {
        if (a_warning_generated()) {
            telemetry.addData("", getWarning_message());
        }
    } // update_telemetry

    public boolean isPressed(TouchSensor touchSensor){
        return touchSensor != null && touchSensor.isPressed();
    }

    public double getOdsRaw(OpticalDistanceSensor sensor) {
        double distReturn = -1;
        if (sensor != null) {
            distReturn = sensor.getRawLightDetected();
        }
        return distReturn;
    }

    public void initializeIMU(BNO055IMU n){
        if(n.isSystemCalibrated()) {
            telemetry.addData("IMU Status", "Ready.");
        }else{
            telemetry.addData("IMU Status", "Calibrating...");
            update_telemetry();
        }
    }

    public double getVoltage(AnalogInput sensor) {
        double distReturn = -1;
        if (sensor != null) {
            distReturn = sensor.getVoltage();
        }
        return distReturn;
    }

    public double get_servo_position(Servo servo) {
        double positionReturn = 0.0;

        if (servo != null) {
            positionReturn = servo.getPosition();
        }
        return positionReturn;
    }

    void set_position(Servo servo, double position) {
        if (servo != null) {
            servo.setPosition(position);
        }
    }

    void set_position(CRServo servo, double direction) {
        if (servo != null) {
            servo.setPower(direction);
        }
    }

    void startReadingUltrasonic(ModernRoboticsI2cRangeSensor sensor){
        if(sensor != null){

            rangeVal[CACHE] = sensor.rawUltrasonic();
            if(rangeVal[CACHE] != rangeVal[FILTERED] && rangeVal[CACHE] != 255) rangeVal[FILTERED] = rangeVal[CACHE];

        }
    }

    double getRangeOptical(ModernRoboticsI2cRangeSensor sensor){
        if(sensor != null){
            return sensor.cmOptical();
        }
        else return 0;
    }

    double readRangeUltasonic(ModernRoboticsI2cRangeSensor sensor){
        double returnVal;
        if(sensor != null){

            returnVal = rangeVal[FILTERED];
        } else {
            returnVal = -1;
        }
        return returnVal;
    }

    public int getColorVal(ColorSensor sensor, String color) {
        int colorReturn = -1;
        color = color.toLowerCase();
        if (sensor != null) {
            if (color.matches("red")) colorReturn = sensor.red();
            if (color.matches("blue")) colorReturn = sensor.blue();
            if (color.matches("green")) colorReturn = sensor.green();
            if (color.matches("alpha")) colorReturn = sensor.alpha();
            if (color.matches("argb")) colorReturn = sensor.argb();
        }
        return colorReturn;
    }

    public double get_motor_power(DcMotor motor) {
        double powerReturn = 0.0;

        if (motor != null) {
            powerReturn = motor.getPower();
        }
        return powerReturn;
    }
    void set_power(DcMotor motor, double power) {
        if (motor != null) {
            motor.setPower(power);
        }
    }
    void set_drive_power(double p_left_power, double p_right_power) {
        set_power(mtrLeftDrive,   p_left_power);
        set_power(mtrRightDrive,  p_right_power);
    } // set_drive_power
    void run_using_encoder(DcMotor motor) {
        if (motor != null) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    } // run_using_encoder
    void run_using_drive_encoders() {
        run_using_encoder(mtrLeftDrive);
        run_using_encoder(mtrRightDrive);
    } // run_using_encoders
    void run_to_position(DcMotor motor) {
        if (motor != null) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    void run_drive_to_position() {
        run_to_position(mtrLeftDrive);
        run_to_position(mtrRightDrive);
    }
    void run_without_encoder(DcMotor motor) {
        if (motor != null) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    } // run_without_encoder
    void run_without_drive_encoders() {
        run_without_encoder(mtrLeftDrive);
        run_without_encoder(mtrRightDrive);
    } // run_without_drive_encoders
    void set_encoder_target(DcMotor motor, int target) {
        if (motor != null) {
            motor.setTargetPosition(target);
        }
    }
    void set_drive_target(int leftTarget, int rightTarget) {
        set_encoder_target(mtrLeftDrive, leftTarget);
        set_encoder_target(mtrRightDrive, rightTarget);
    }
    void reset_encoders(DcMotor motor) {
        if (motor != null) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    void reset_encoders(DcMotor motor1, DcMotor motor2) {
        if (motor1 != null) {
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (motor2 != null) {
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    void reset_encoders(DcMotor motor1, DcMotor motor2, DcMotor motor3) {
        if (motor1 != null) {
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (motor2 != null) {
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (motor3 != null) {
            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    void reset_encoders(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        if (motor1 != null) {
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (motor2 != null) {
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (motor3 != null) {
            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (motor4 != null) {
            motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    void reset_drive_encoders() {
        reset_encoders(mtrLeftDrive, mtrRightDrive);
    } // reset_drive_encoders
    int get_encoder_count(DcMotor motor) {
        int encoderReturn = 0;

        if (motor != null) {
            encoderReturn = motor.getCurrentPosition();
        }

        return encoderReturn;
    } //get_encoder_count
    boolean has_encoder_reached(DcMotor motor, double p_count) {
        boolean l_return = false;

        if (motor != null) {
            if (Math.abs(motor.getCurrentPosition()) > (int) Math.abs(p_count) -  10) {
                l_return = true;
            }
        }
        return l_return;
    } // has_encoder_reached
    boolean have_drive_encoders_reached(double p_left_count, double p_right_count) {
        boolean l_return = false;

        if (has_encoder_reached(mtrLeftDrive, p_left_count) &&
                has_encoder_reached(mtrRightDrive, p_right_count)) {
            l_return = true;
        }

        return l_return;
    } // have_encoders_reached
    boolean has_encoder_reset(DcMotor motor) {
        boolean l_return = false;

        if (get_encoder_count(motor) == 0) {
            l_return = true;
        }
        return l_return;
    } // has_encoder_reset
    boolean have_drive_encoders_reset() {
        boolean l_return = false;
        if (Math.abs(get_encoder_count(mtrLeftDrive)) < 15 & Math.abs(get_encoder_count(mtrRightDrive)) < 15) {
            l_return = true;
        }

        return l_return;
    } // have_drive_encoders_reset

    public void writeFile(Integer[][] fileName, Integer locationX, Integer locationY, Integer value){
        fileName[locationY][locationX] = value;
    }
    public Integer readFile(Integer[][] fileName, Integer locationX, Integer locationY){
        return fileName[locationY][locationX];
    }
    public void writeToFile(String data){

        File folder = new File(Environment.getExternalStorageDirectory() + "/testWrite");
        String fileName = "test.txt";

        boolean success = true;
        if(!folder.exists()) {
            success = folder.mkdir();
        }

        if(success) {
            File file = new File(folder, fileName);
            FileOutputStream stream = null;

            try {
                stream = new FileOutputStream(file);
                stream.write(data.getBytes());
            } catch (Exception var16) {
                var16.printStackTrace();
            } finally {
                try {
                    stream.close();
                } catch (IOException var15) {
                    var15.printStackTrace();
                }

            }

        } else {
            telemetry.addData("Error", "Unable to create directory");
        }
    }
    public String readFromFile() {
        String output = "";
        File folder = new File(Environment.getExternalStorageDirectory() + "/testWrite");
        String fileName = "test.txt";


        try {
            File file = new File(folder, fileName);
            FileInputStream stream = new FileInputStream(file);

            int size = stream.available();
            byte[] buffer = new byte[size];
            stream.read(buffer);
            stream.close();
            output = new String(buffer);
        } catch (IOException e) {
            telemetry.addData("Error", "Couldn't read from file.");
        }

        return output;

    }
    public void appendToFile(String appendData){
        File folder = new File(Environment.getExternalStorageDirectory() + "/testWrite");
        String fileName = "test.txt";
        String currentVal = "";

        try{
            File file = new File(folder, fileName);
            currentVal = readFromFile() + appendData;
            writeToFile(currentVal);
        }catch(Exception e){
            telemetry.addData("Error", "Couldn't append to file.");
        }
    }
    public void appendToNewLine(String appendData){
        File folder = new File(Environment.getExternalStorageDirectory() + "/testWrite");
        String fileName = "test.txt";
        String currentVal = "";

        try{
            File file = new File(folder, fileName);
            currentVal = readFromFile() + "\n" + appendData;
            writeToFile(currentVal);
        }catch(Exception e){
            telemetry.addData("Error", "Couldn't append to file.");
        }
    }

    DcMotor mtrRightDrive;
    DcMotor mtrLeftDrive;
    DcMotor mtrLift;
    DcMotor mtrFlip;
    DcMotor mtrArmSpin;
    DcMotor mtrArmFlip;

    Servo srvGr1;
    Servo srvGr2;
    CRServo srvExtend;
    Servo srvClaw;
    CRServo srvShift;
    Servo srvLevel;

    ColorSensor snsColorLeft;
    ColorSensor snsColorRight;
    BNO055IMU IMUnav;

}