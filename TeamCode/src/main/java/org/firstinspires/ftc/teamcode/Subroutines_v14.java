package org.firstinspires.ftc.teamcode;

import android.content.ContentValues;
import android.content.Context;
import android.graphics.Bitmap;
import android.os.Environment;
import android.os.Looper;
import android.text.method.Touch;
import android.util.Log;

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
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
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
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;

import static org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuMarkIdentification.TAG;
import static org.opencv.imgproc.Imgproc.HoughLinesP;
import static org.opencv.imgproc.Imgproc.minAreaRect;


class Subroutines_v14 extends OpMode {
    //Variable initialization//
    long systemTime = 0;             //System time used to determine how much time has passed for waitHasFinished()//
    boolean initOS = true;           //Oneshot used for waitHasFinished//

    private boolean warning_generated;
    private String warning_message;

    public final byte BLUE = 1;
    public final byte RED = -1;
    boolean driveFinished = false;

    ArrayList<Long> lDeltaEncoder = new ArrayList<>();
    Long lPreviousEncoder = new Long(0);
    ArrayList<Long> rDeltaEncoder = new ArrayList<>();
    Long rPreviousEncoder = new Long(0);

    double rangeVal[] = {0, 0};
    final byte FILTERED = 1;
    final byte CACHE = 0;

    final double GR1CLOSED = 0.15;
    final double GR1OPEN = 1.0;
    final double GR2CLOSED = 1-GR1CLOSED;
    final double GR2OPEN = 1-GR1OPEN;

    final double CLAWCLOSED = 0.685;
    final double CLAWOPEN = 1;

    final double LEVELUP = 0;
    final double LEVELDOWN = .57;
    final double LEVELPARTIALDOWN = 0.25;

    final double UD_DOWN = .25;
    final double UD_UP = .735;

    final double LR_CENTER = .53;
    final double LR_LEFT = .415;
    final double LR_RIGHT = .63;
    final double LR_HOME = .53;//WAS .49

    final double CAM_SIDE = .565;
    final double CAM_VUMARK = .545;
    final double CAM_JEWELS = .65;
    final double CAM_FRONT = .06;

    static VuforiaLocalizer vuforia;
    static VuforiaTrackable relicTemplate;

    public Mat mLines;

    VuforiaLocalizer.CloseableFrame frame = null;

    enum liftPos{
        ONE(0), CARRY(-300),AUTO(-600),TWO(-1265),THREE(-1963),FOUR(-2850);

        private int val;

        liftPos(int val){
            this.val = val;
        }

        public int getVal(){
            return val;
        }
    }

    enum extendPos{
        HOME(0), PARTIAL_EXTEND(-867), PARTIAL_RETRACT(-2000), FULL_EXTEND(-3380);

        private int val;

        extendPos(int val){
            this.val = val;
        }

        public int getVal(){
            return val;
        }
    }


    @Override
    public void init() {

        warning_generated = false;
        warning_message = "Can't find:  ";

        Gamepad gamepad1 = new Gamepad();
        Gamepad gamepad2 = new Gamepad();

        mtrLeftDrive = addMotor(MotorLocation.LEFT, "mtrLeftDrive");
        mtrRightDrive = addMotor(MotorLocation.RIGHT, "mtrRightDrive");
        mtrLift = addMotor(MotorLocation.SHIFT, "mtrLift");
        mtrExtend = addMotor(MotorLocation.SHIFT, "mtrExtend");

        srvGr1 = addServo(GR1OPEN, "srvGr1");
        srvGr2 = addServo(GR2OPEN, "srvGr2");
        srvClaw = addServo(0.1, "srvClaw");
        srvLR = addServo(LR_HOME, "srvLR");
        srvUD = addServo(UD_UP, "srvUD");
        srvPhone = addServo(CAM_SIDE, "srvPhone");
        srvLevel = addServo(LEVELDOWN, "srvLevel");

        snsBtnGlyph = addTouchSensor("btnGlyph");
        snsBtnRelic = addTouchSensor("btnRelic");

        try {
            IMUnav = hardwareMap.get(BNO055IMU.class, "imu");
            Parameters p = new Parameters();
            p.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            p.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            p.calibrationDataFile = "BNO055IMUCalibration.json";
            try {
                IMUnav.initialize(p);
            } catch (Exception e) {
            }
            IMUnav.isSystemCalibrated();
        } catch (Exception p_exception) {
            addWarningMessage("imu");
            RobotLog.i(p_exception.getLocalizedMessage());
            IMUnav = null;
        }

        update_telemetry();

    }

    @Override
    public void loop() {
    }

    public DcMotor addMotor(MotorLocation side, String configName) {
        DcMotor m;
        try {
            m = hardwareMap.dcMotor.get(configName);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (side == MotorLocation.RIGHT) m.setDirection(DcMotorSimple.Direction.REVERSE);
            if (side == MotorLocation.LEFT) m.setDirection(DcMotorSimple.Direction.FORWARD);
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
        LEFT, RIGHT, SHIFT
    }

    public Servo addServo(double pos, String configName) {
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

    public CRServo addCRServo(String configName) {
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

    public ColorSensor addColorSensor(int address, boolean LED, String configName) {
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

    public ColorSensor addColorSensor(boolean LED, String configName) {
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

    public OpticalDistanceSensor addODSSensor(String configName) {
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

    public TouchSensor addTouchSensor(String configName) {
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
        try {
            imu = hardwareMap.get(BNO055IMU.class, configName);
            Parameters p = new Parameters();
            p.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            p.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            p.calibrationDataFile = "BNO055IMUCalibration.json";
            imu.initialize(p);
            imu.isSystemCalibrated();
        } catch (Exception p_exception) {
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
    //this is just here so vcs detects that I changed the program and I can commit.
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

    public boolean isWithin(double val, double high, double low){
        return val <= high && val >= low;
    }

    public boolean isPressed(TouchSensor touchSensor) {
        return touchSensor != null && touchSensor.isPressed();
    }

    public double getOdsRaw(OpticalDistanceSensor sensor) {
        double distReturn = -1;
        if (sensor != null) {
            distReturn = sensor.getRawLightDetected();
        }
        return distReturn;
    }

    public void initializeIMU(BNO055IMU n) {
        if (n.isSystemCalibrated()) {
            telemetry.addData("IMU Status", "Ready.");
        } else {
            telemetry.addData("IMU Status", "Calibrating...");
            update_telemetry();
        }
    }

    public double getVoltage(AnalogInput sensor) {
        double distReturn = Double.MAX_VALUE;
        try {
            if (sensor != null) {
                distReturn = sensor.getVoltage();
            }
        }catch(Exception e) {
            e.printStackTrace();
        }
        return distReturn;
    }

    public double getBatteryVoltage() {
        try {
            if (hardwareMap.voltageSensor.iterator().hasNext()) {
                return hardwareMap.voltageSensor.iterator().next().getVoltage();
            } else return Double.MAX_VALUE;
        }catch(Exception e) {
            e.printStackTrace();
            return Double.MAX_VALUE;
        }
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

    void startReadingUltrasonic(ModernRoboticsI2cRangeSensor sensor) {
        if (sensor != null) {

            rangeVal[CACHE] = sensor.rawUltrasonic();
            if (rangeVal[CACHE] != rangeVal[FILTERED] && rangeVal[CACHE] != 255)
                rangeVal[FILTERED] = rangeVal[CACHE];

        }
    }

    double getRangeOptical(ModernRoboticsI2cRangeSensor sensor) {
        if (sensor != null) {
            return sensor.cmOptical();
        } else return 0;
    }

    double readRangeUltasonic(ModernRoboticsI2cRangeSensor sensor) {
        double returnVal;
        if (sensor != null) {

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
        set_power(mtrLeftDrive, p_left_power);
        set_power(mtrRightDrive, p_right_power);
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
            if (Math.abs(motor.getCurrentPosition()) > (int) Math.abs(p_count) - 10) {
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

    public void writeFile(Integer[][] fileName, Integer locationX, Integer locationY, Integer value) {
        fileName[locationY][locationX] = value;
    }

    public Integer readFile(Integer[][] fileName, Integer locationX, Integer locationY) {
        return fileName[locationY][locationX];
    }

//    public void writeImage(Mat mt) {
//        Bitmap bmp = null;
//        try {
//            bmp = Bitmap.createBitmap(mt.cols(),mt.rows(),Bitmap.Config.ARGB_8888);
//            Utils.matToBitmap(mt,bmp);
//        } catch(CvException e) {
//            Log.d(TAG,e.getMessage());
//        }
//        FileOutputStream fo = null;
//        String fileName = "test.png";
//
//        File folder = new File(Environment.getExternalStorageDirectory() + "/testWrite");
//
//        File dest = new File(folder,fileName);
//        try {
//            fo = new FileOutputStream(dest);
//            bmp.compress(Bitmap.CompressFormat.PNG, 100, fo);
//        } catch (Exception e) {
//            e.printStackTrace();
//        } finally {
//            try {
//                if(fo != null) fo.close();
//            } catch(IOException e) {
//                e.printStackTrace();
//            }
//        }
//    }

    public void writeToFile(String data) {

        File folder = new File(Environment.getExternalStorageDirectory() + "/data");
        String fileName = "xy.txt";

        boolean success = true;
        if (!folder.exists()) {
            success = folder.mkdir();
        }

        if (success) {
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

    public void appendToFile(String appendData) {
        File folder = new File(Environment.getExternalStorageDirectory() + "/testWrite");
        String fileName = "test.txt";
        String currentVal = "";

        try {
            File file = new File(folder, fileName);
            currentVal = readFromFile() + appendData;
            writeToFile(currentVal);
        } catch (Exception e) {
            telemetry.addData("Error", "Couldn't append to file.");
        }
    }

    public void appendToNewLine(String appendData) {
        File folder = new File(Environment.getExternalStorageDirectory() + "/testWrite");
        String fileName = "test.txt";
        String currentVal = "";

        try {
            File file = new File(folder, fileName);
            currentVal = readFromFile() + "\n" + appendData;
            writeToFile(currentVal);
        } catch (Exception e) {
            telemetry.addData("Error", "Couldn't append to file.");
        }
    }

    void initializeVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AXlr9dj/////AAAAGRujCnGOL0aIpjtd4y5IK2sFwI4jstOeOlytTrPr3jzeQQ9tGEHgLmuGdxzuxGNY5641pyXeeyJHccL+I4QCZq8Sodm5DAUBsAQQ9ox1EY3+KNfZISN06k1IqDf7YaRXhE02j+7aE4Apnm3Hvn9V5CDKSTgOq73eJId9uzHkuNaIx+UDV4fRS1HK5L6dSGmIw3+RW7uaFdBF0E2bvWzzpZv51KFzw5oy/9qFB9r6ke5B5Gd2zw9JjafwudFSpLVCiNzIreoTaIFYVMmGMuIFIT4C6oC13EcvbNl5CFpl+irLqhSI//nlL2L2DKxKtW5UTQqNBlOSBdTxWR/kSN12edlwOu0kFgzhKBFapn3KHC0V";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        relicTrackables.activate();

        vuforia.setFrameQueueCapacity(1);
    }


    void initializeOpenCV() {
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

    }

    void initializeVision() {
        initializeVuforia();
        initializeOpenCV();
    }

    Mat vuforiaFrameToMat() {
        VuforiaLocalizer.CloseableFrame frame = null;
        try {
            long totalFrame = 0;
            if (vuforia.getFrameQueue().size() >= 1){
                frame = vuforia.getFrameQueue().take();
                totalFrame = frame.getNumImages();
            }

            Image rgb = null;

            int counter = 0;
            if (counter < totalFrame) {
                if (frame.getImage(counter).getFormat() == PIXEL_FORMAT.RGB565)
                    rgb = frame.getImage(counter);
                counter++;
            }

            Bitmap bm = null;

            if(rgb != null) bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
            else return new Mat(700,1280,CvType.CV_8UC4);
            bm.copyPixelsFromBuffer(rgb.getPixels());


            Mat tmp;

            tmp = new Mat(rgb.getWidth(), rgb.getHeight(), CvType.CV_8UC4);

            Utils.bitmapToMat(bm, tmp);

            return tmp;




        } catch (Exception e) {
            e.printStackTrace();
            return new Mat(700, 1200, CvType.CV_8UC4);
        }
    }

    double distance(Point center, Point check) {
        return Math.hypot((center.x - check.x), (center.y - check.y));
    }

    void updateEncoderDelta() {
        if(mtrLeftDrive!=null&&mtrRightDrive!=null) {
            if(!isWithin(get_encoder_count(mtrLeftDrive),lPreviousEncoder+70,lPreviousEncoder-70)
             ||!isWithin(get_encoder_count(mtrRightDrive),rPreviousEncoder+70,rPreviousEncoder-70)) {

                lDeltaEncoder.add(new Long(get_encoder_count(mtrLeftDrive) - lPreviousEncoder));
                rDeltaEncoder.add(new Long(get_encoder_count(mtrRightDrive) - rPreviousEncoder));
            }
        }
    }

    void clearEncoderDelta() {
        if(lDeltaEncoder.size() > 0 && rDeltaEncoder.size() > 0) {
            lDeltaEncoder.remove(lDeltaEncoder.size() - 1);
            rDeltaEncoder.remove(rDeltaEncoder.size() - 1);
        }
    }

    void resetEncoderDelta() {
        lDeltaEncoder.clear();
        rDeltaEncoder.clear();
    }

    Long lDelta() {
        return lDeltaEncoder.get(lDeltaEncoder.size()-1);
    }
    Long rDelta() {
        return rDeltaEncoder.get(rDeltaEncoder.size()-1);
    }

    boolean deltaNotZero() {
        return (lDeltaEncoder.size()>0&&rDeltaEncoder.size()>0);
    }





    class ColorBlobDetector {
        // Lower and Upper bounds for range checking in HSV color space
        private Scalar mLowerBound = new Scalar(0);
        private Scalar mUpperBound = new Scalar(0);

        // Minimum contour area in percent for contours filtering
        private double mMinContourArea = 0.1;

        // Color radius for range checking in HSV color space with touch color
        private Scalar mColorRadius = new Scalar(25, 50, 50, 0);

        private Mat mSpectrum = new Mat();
        private List<MatOfPoint> mContours = new ArrayList<MatOfPoint>();

        // Cache
        Mat mPyrDownMat = new Mat();
        Mat mHsvMat = new Mat();
        Mat mMask = new Mat();
        Mat mDilatedMask = new Mat();
        Mat mHierarchy = new Mat();

        public void setColorRadius(Scalar radius) {
            mColorRadius = radius;
        }

        public void setColorRange(Scalar minHSV, Scalar maxHSV) {
            mLowerBound = minHSV;
            mUpperBound = maxHSV;
        }

        public void setHsvColor(Scalar hsvColor) {
            double minH = (hsvColor.val[0] >= mColorRadius.val[0]) ? hsvColor.val[0] - mColorRadius.val[0] : 0;
            double maxH = (hsvColor.val[0] + mColorRadius.val[0] <= 255) ? hsvColor.val[0] + mColorRadius.val[0] : 255;

            mLowerBound.val[0] = minH;
            mUpperBound.val[0] = maxH;

            //mLowerBound.val[1] = hsvColor.val[1] - mColorRadius.val[1];
            //mUpperBound.val[1] = hsvColor.val[1] + mColorRadius.val[1];
            mLowerBound.val[1] = 0;
            mUpperBound.val[1] = 255;

            //mLowerBound.val[2] = hsvColor.val[2] - mColorRadius.val[2];
            //mUpperBound.val[2] = hsvColor.val[2] + mColorRadius.val[2];
            mLowerBound.val[2] = 0;
            mUpperBound.val[2] = 255;

            mLowerBound.val[3] = 0;
            mUpperBound.val[3] = 255;

            Log.i(TAG, "Set HSV Color Min: (" +
                    mLowerBound.val[0] + "," +
                    mLowerBound.val[1] + "," +
                    mLowerBound.val[2] + ")");

            Log.i(TAG, "Set HSV Color Max: (" +
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



        public boolean isWithin(double val, double low, double high){
            return val >= low && val <= high;
        }

        public void process(Mat rgbaImage) {
            Imgproc.pyrDown(rgbaImage, mPyrDownMat);
            Imgproc.pyrDown(mPyrDownMat, mPyrDownMat);

            Imgproc.cvtColor(mPyrDownMat, mHsvMat, Imgproc.COLOR_RGB2HSV_FULL);

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

            Imgproc.dilate(mMask, mDilatedMask, new Mat());

            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

            Imgproc.findContours(mDilatedMask, contours, mHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find max contour area
            double maxArea = 0;
            Iterator<MatOfPoint> each = contours.iterator();
            while (each.hasNext()) {
                MatOfPoint wrapper = each.next();
                double area = Imgproc.contourArea(wrapper);
                if (area > maxArea)
                    maxArea = area;
            }


            // Filter contours by area and resize to fit the original image size
            mContours.clear();
            each = contours.iterator();
            while (each.hasNext()) {
                MatOfPoint contour = each.next();
                if (Imgproc.contourArea(contour) > mMinContourArea * maxArea) {
                    Core.multiply(contour, new Scalar(4, 4), contour);
                    mContours.add(contour);
                }
            }
        }

        public void processLines(Mat mRgba, CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
            mRgba = inputFrame.rgba();
            Imgproc.cvtColor(mRgba,mHsvMat,Imgproc.COLOR_RGB2HSV_FULL);
            //Imgproc.pyrDown(mHsvMat,mHsvMat);
            //Imgproc.pyrDown(mHsvMat,mHsvMat);

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

                Core.add(mMask1,mMask2,mMask);

            } else {
                Core.inRange(mHsvMat, mLowerBound, mUpperBound, mMask);
            }

            Imgproc.Canny(mMask, mMask, 50, 100);

            HoughLinesP(mMask, mLines, 5, Math.PI/180, 7,60, 20);

            //Imgproc.morphologyEx(mMask,mMask,Imgproc.MORPH_CLOSE,mRectangle);

        }

        public void showLines(Mat mRgba){

            try {
                double val0,val1,val2,val3;
                LineClusters clusters = new LineClusters();
                for (int i = 0; i < mLines.rows(); i++) {
                    double[] val = mLines.get(i,0);
//                double rho = val[0], theta = val[1];
//                double cosTheta = Math.cos(theta);
//                double sinTheta = Math.sin(theta);
//                double x = cosTheta * rho;
//                double y = sinTheta * rho;
//                Point p1 = new Point(x + 10000 * -sinTheta, y + 10000 * cosTheta);
//                Point p2 = new Point(x - 10000 * -sinTheta, y - 10000 * cosTheta);

                    val0 = val[0];
                    val1 = val[1];
                    val2 = val[2];
                    val3 = val[3];


                    double angle = ((Math.atan2(val1-val3,val0-val2)*180/Math.PI) + 180)%180;

                    //Log.println(Log.ASSERT, "TAG", angle+"degrees loop:" + i);

                    if(isWithin(angle,30,150) && !isWithin(angle,80,100)) {
                        Imgproc.line(mRgba,
                                new Point(val0, val1),
                                new Point(val2, val3),
                                new Scalar(255, 255, 255),
                                10);
                        clusters.add(new Line(new Point(val0,val1),new Point(val2,val3),angle));
                        Log.println(Log.ASSERT,"TAG",angle + " is the angle of line " + i);
                    }
//                else Imgproc.line(mRgba,
//                            new Point(val0, val1),
//                            new Point(val2, val3),
//                            new Scalar(255, 0, 0),
//                            10);
                }
                Log.println(Log.ASSERT, "TAG", clusters.toString()+"\n");
                for(int i = 0; i < clusters.clusters.size(); i ++) {
                    if(clusters.clusters.get(i).lines.size() > 3) {
                        Point[] rectPoints = new Point[4];
                        MatOfPoint2f mp2f = new MatOfPoint2f();
                        mp2f.fromList(clusters.clusters.get(i).points);
                        RotatedRect rRect = minAreaRect(mp2f);
                        rRect.points(rectPoints);
                        MatOfPoint mPoints = new MatOfPoint(rectPoints);
                        List<MatOfPoint> lPoints = new ArrayList<>();
                        lPoints.add(mPoints);
                        Log.println(Log.ASSERT, "TAG", Arrays.toString(rectPoints) + "Points");

                        Imgproc.polylines(mRgba, lPoints, true, new Scalar(0, 255, 0), 10);
                    }

                }

            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        class Line {
            Point p1, p2;
            double angle;
            public Line(Point p1,Point p2, double angle) {
                this.p1 = p1;
                this.p2 = p2;
                this.angle = angle;
            }
        }

        class LineCluster {

            RotatedRect rRect = new RotatedRect();
            List<Line> lines = new ArrayList<>();
            List<Point> points = new ArrayList<>();
            double angle = 0;

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
            }
            public boolean isClose(Point p, double tolerance) {
                for(int i = 0; i < points.size(); i++) {
                    if(Math.hypot(p.x-points.get(i).x,p.y-points.get(i).y) <= tolerance) {
                        return true;
                    }
                }
                return false;
            }
            public Point center() {return rRect.center;}
            public void avgAngle() {
                int n = 0;
                for(int i = 0; i < lines.size(); i++) {
                    n += lines.get(i).angle;
                }
                angle = n / lines.size();
            }
            public String toString() {
                avgAngle();
                return "Angle is " + angle + "Lines are " + lines.size();
            }
        }

        class LineClusters {
            List<LineCluster> clusters = new ArrayList<>();

            public void add(Line line){
                boolean foundCluster = false;
                outerloop:
                for(int i = 0; i < clusters.size(); i ++) {
                    for(int j = 0; j < clusters.get(i).lines.size(); j++) {
                        if(isWithin(line.angle ,clusters.get(i).angle - 7, clusters.get(i).angle + 7)) {
                            if(clusters.get(i).isClose(line.p1,80) ||clusters.get(i).isClose(line.p2,80)) {
                                clusters.get(i).addLine(line);
                                foundCluster = true;
                                break outerloop;
                            }
                        }
                    }
                }
                if(!foundCluster) {
                    clusters.add(new LineCluster(line));
                }
            }

            public String toString() {
                String returnVal = "";
                for(int i = 0; i < clusters.size();i++) {
                    returnVal += "cluster " + i + " " +clusters.get(i).toString() + "\n";
                }
                return returnVal;
            }
        }

        public List<MatOfPoint> getContours() {
            return mContours;
        }
    }



    static DcMotor mtrRightDrive;
    static DcMotor mtrLeftDrive;
    static DcMotor mtrLift;
    static DcMotor mtrExtend;

    static Servo srvGr1;
    static Servo srvGr2;
    static Servo srvClaw;
    static Servo srvLR;
    static Servo srvUD;
    static Servo srvPhone;
    static Servo srvLevel;

    static BNO055IMU IMUnav;
    static TouchSensor snsBtnGlyph;
    static TouchSensor snsBtnRelic;

    class ColorLineDetector {

        File logFile = new File(Environment.getExternalStorageDirectory()+"/Angles/Data/data.txt");
        // Lower and Upper bounds for range checking in HSV color space
        private Scalar mLowerBound = new Scalar(0);
        private Scalar mUpperBound = new Scalar(0);

        // Minimum contour area in percent for contours filtering
        private  double mMinContourArea = 0.1;

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

            Imgproc.erode(mMask, mMask, Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(5,5)));
            Imgproc.dilate(mMask,mMask,Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT,new Size(13,13)));


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


                    if ((isWithin(angle, 30, 150) && !isWithin(angle, 80, 100)) &&
                            val1 > mRgba.rows() / 4 && val3 > mRgba.rows() / 4) {
                        Imgproc.line(mRgba,
                                new Point(val0, val1),
                                new Point(val2, val3),
                                new Scalar(255, 255, 255),
                                10);
                        clusters.add(new Line(new Point(val0, val1), new Point(val2, val3), angle));
                        Log.println(Log.ASSERT, "TAG", angle + " is the angle of line " + i);
                    }
                }
                Log.println(Log.ASSERT, "TAG", clusters.toString() + "\n");
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
                        Log.println(Log.ASSERT, "TAG", Arrays.toString(rectPoints) + "Points");


                        Imgproc.polylines(mRgba, lPoints, true, new Scalar(0, 255, 0), 10);
                    }
                }

                int countedCount = 0;
                totalX = 0;
                totalY = 0;

                for (int i = 0; i < 2 && i < clusters.clusterGroups.size()-1; i++) {
                    LineCluster l = clusters.clusterGroups.get(i);
                    LineCluster l0 = clusters.clusterGroups.get(i+1);
                    if (l.lines.size() >= SHOW_THRESH) {
                        Point p = calculateIntersect(l.angle, l.center(), l0.angle, l0.center());
                        if(!(p.x <=0 || p.x >= mRgba.cols())) {
                            totalX += p.x;
                            totalY += p.y;
                            countedCount++;
                        }
                    }
                }

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
                for(int i = 0; i < Xs.size();i++) {
                    sum += Xs.get(i)*(QUEUE_SIZE-i);
                }
                sum = (int) (sum/(QUEUE_SIZE * ((QUEUE_SIZE+1)/2))) ;
                centerP = new Point(sum, median(Ys));
            }catch (Exception e) {
                e.printStackTrace();
            }
            try{
                Imgproc.circle(mRgba, centerP, 2, new Scalar(255, 0, 0), 5);
            } catch (Exception e) {
                e.printStackTrace();
            }
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

        class LineCluster implements Comparable{

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
                if (isWithin(calculateIntersect(angle,center(),0,p.p1).x,p.p1.x-tolerance,p.p1.x-tolerance)||
                        isWithin(calculateIntersect(angle,center(),0,p.p2).x,p.p2.x-tolerance,p.p2.x-tolerance)) {
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
                if(o instanceof LineCluster) {
                    if (area > ((LineCluster)o).area) return 1;
                    else if(area < ((LineCluster)o).area) return -1;
                    else return 0;
                }else return 0;
            }
        }

        class LineClusters {
            List<LineCluster> clusterGroups = new ArrayList<>();

            public void add(Line line) {
                boolean foundCluster = false;
                outerloop:
                for (int i = 0; i < clusterGroups.size(); i++) {
                    for (int j = 0; j < clusterGroups.get(i).lines.size(); j++) {
                        if (isWithin(line.angle, clusterGroups.get(i).angle - 7, clusterGroups.get(i).angle + 7)) {
                            if (clusterGroups.get(i).isClose(line, 80)){
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
                if(clusterGroups.size() >= 2){
                    if(centerP!=null)logToFile(centerP.y);
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
            angle1 *= Math.PI/180;
            angle2 *= Math.PI/180;
            int x1 = (int) p1.x;
            int y1 = -(int) p1.y;
            int x2 = (int) p2.x;
            int y2 = -(int) p2.y;
            double m1 = Math.tan(angle1);
            double m2 = Math.tan(angle2);
            int xf = (int) (((-m2*x2) + y2 + (m1 * x1) - y1)/(m1-m2));
            int yf = (int) ((m1*xf) - (m1*x1) + y1);
            intersect = new Point(xf,yf);
            return intersect;
        }
        public Point pointIntersect(double angle1, Point p1, double angle2, Point p2) {
            Point intersect = new Point();
            angle1 *= Math.PI/180;
            angle2 *= Math.PI/180;
            int x1 = (int) p1.x + 10000;
            int y1 = (int) p1.y + 10000;
            int x2 = (int) p2.x + 10000;
            int y2 = (int) p2.y + 10000;
            double m1 = Math.tan(angle1);
            double m2 = Math.tan(angle2);
            int xf = (int) (((-m2*x2) + y2 + (m1 * x1) - y1)/(m1-m2));
            int yf = (int) ((m1*xf) - (m1*x1) + y1);
            intersect = new Point(xf - 10000,yf - 10000);
            return intersect;
        }

        public void logToFile(Object o)
        {
            String text = o.toString();

            if (!logFile.exists())
            {
                try
                {
                    logFile.createNewFile();
                }
                catch (Exception e)
                {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            }
            try
            {
                //BufferedWriter for performance, true to set append to file flag
                BufferedWriter buf = new BufferedWriter(new FileWriter(logFile, true));
                buf.append(text);
                buf.append(System.lineSeparator());
                buf.close();
            }
            //this is a commit
            catch (Exception e)
            {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }

    }

}

