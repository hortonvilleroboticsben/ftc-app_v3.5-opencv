package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuMarkIdentification;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.opencv.android.CameraBridgeViewBase;
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
import java.util.Arrays;
import java.util.List;

import static org.opencv.imgproc.Imgproc.minEnclosingCircle;

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
    private int count = 0;

    private int countA = 0;

    static BallColor[] ballArray = {null, null};
    static RelicRecoveryVuMark vuMark = null;

//    public ColorLineDetector lDetectorBlue = new ColorLineDetector();
//    public ColorLineDetector lDetectorRed = new ColorLineDetector();
    //static


    static Orientation O;

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
        //O = IMUnav.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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

    void GyroEndTurn(double degrees, double speed) {
        StateMachine_v6 s = new StateMachine_v6();

        s.Turn(degrees - 3, speed);
        s.GyroTurn(3,10);
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
            Orientation o = O;
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

    public void RelativeMotorMove(DcMotor motor, long encCount, double power){
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

    public void AbsoluteMotorMove(DcMotor m,long encCount, double power){
        if(next_state_to_execute()) {
            run_to_position(m);
            set_encoder_target(m, (int) encCount);
            if (get_encoder_count(m) > m.getTargetPosition()) {
                if (Math.abs(m.getCurrentPosition()) > (int) Math.abs(encCount) - 100) {
                    run_using_encoder(m);
                    set_power(m,0);
                    incrementState();
                } else set_power(m, power);
            } else {
                if (!(Math.abs(m.getCurrentPosition()) > (int) Math.abs(encCount) + 100)) {
                    run_using_encoder(m);
                    set_power(m,0);
                    incrementState();
                } else set_power(m, power);

            }
        }
    }

    public void ServoIncrementalMove(Servo srv, double pos, double inc){
        if(next_state_to_execute()) {
            inc = Math.abs(inc);
            inc = get_servo_position(srv) < pos ? inc : -inc;

            if(waitHasFinished(3)) {
                set_position(srv, get_servo_position(srv) + inc);
            }

            if(get_servo_position(srv) <= pos+inc && get_servo_position(srv) >= pos-inc){
                incrementState();
            }
        }
    }

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

    void ProcessRelic() {
        if (next_state_to_execute()) {
            if(++count <= 5) {
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
                telemetry.addData("vuMark", vuMark);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) incrementState();
            }else{
                vuMark = RelicRecoveryVuMark.UNKNOWN;
                incrementState();
            }
        }
    }

    void ProcessJewels(Mat m) {
        if (next_state_to_execute()) {
            int height = m.height();
            int width = m.width();

            Mat mRgba;
            Mat mROI;
            Rect rROI;
            Point centerROI;

            Scalar mBlobColorRgba;

            ColorBlobDetector mDetectorRed;
            ColorBlobDetector mDetectorBlue;


            Mat mSpectrumRed;

            Scalar CONTOUR_COLOR_RED;
            Scalar CONTOUR_COLOR_BLUE;

            Scalar ROI_COLOR;

            mRgba = new Mat(height, width, CvType.CV_8UC4);

            rROI = new Rect((int) width / 3, 2*(int)height / 3, (int) width / 3, (int) height / 3);
            centerROI = new Point(rROI.width / 2, rROI.height / 2);
            mROI = new Mat(mRgba, rROI);

            Log.d(ConceptVuMarkIdentification.TAG, "onCamerViewStarted Width: " + width + " Height: " + height);

            mDetectorRed = new ColorBlobDetector();
            mDetectorBlue = new ColorBlobDetector();

            mSpectrumRed = new Mat();

            mBlobColorRgba = new Scalar(255);

            CONTOUR_COLOR_RED = new Scalar(255, 0, 0, 255);
            CONTOUR_COLOR_BLUE = new Scalar(0, 0, 255, 255);

            ROI_COLOR = new Scalar(255, 255, 255, 255);

            mDetectorRed.setColorRange(new Scalar(237, 120, 130, 0), new Scalar(20, 255, 255, 255));
            mDetectorBlue.setColorRange(new Scalar(125, 120, 130, 0), new Scalar(187, 255, 255, 255));

            mRgba = m;
            mROI = new Mat(mRgba, rROI);

            double radiusRedBest = 0.0;
            double radiusBlueBest = 0.0;

            String message = "";


            Imgproc.blur(mROI, mROI, new Size(20, 20));

            mDetectorRed.process(mROI);
            mDetectorBlue.process(mROI);

            List<MatOfPoint> contoursRed = mDetectorRed.getContours();
            Log.e(ConceptVuMarkIdentification.TAG, "Red Contours count: " + contoursRed.size());
            //Imgproc.drawContours(mRgba, contoursRed, -1, CONTOUR_COLOR_RED);

            List<MatOfPoint> contoursBlue = mDetectorBlue.getContours();
            Log.e(ConceptVuMarkIdentification.TAG, "Blue Contours count: " + contoursBlue.size());
            //Imgproc.drawContours(mRgba, contoursBlue, -1, CONTOUR_COLOR_BLUE);

            List<Moments> muRed = new ArrayList<Moments>(contoursRed.size());

            Point centerRedBest = null;

            List<MatOfPoint2f> contours2fRed = new ArrayList<MatOfPoint2f>();
            List<MatOfPoint2f> polyMOP2fRed = new ArrayList<MatOfPoint2f>();
            List<MatOfPoint> polyMOPRed = new ArrayList<MatOfPoint>();

            float[] radiusRed = new float[contoursRed.size()];
//TODO
            for (int i = 0; i < contoursRed.size(); i++) {
                Point centerRed = new Point();
                muRed.add(i, Imgproc.moments(contoursRed.get(i), false));

                contours2fRed.add(new MatOfPoint2f());
                polyMOP2fRed.add(new MatOfPoint2f());
                polyMOPRed.add(new MatOfPoint());

                contoursRed.get(i).convertTo(contours2fRed.get(i), CvType.CV_32FC2);
                Imgproc.approxPolyDP(contours2fRed.get(i), polyMOP2fRed.get(i), 3, true);
                polyMOP2fRed.get(i).convertTo(polyMOPRed.get(i), CvType.CV_32S);

                minEnclosingCircle(polyMOP2fRed.get(i), centerRed, radiusRed);
                Imgproc.circle(mRgba, new Point(centerRed.x + rROI.x, centerRed.y + rROI.y), 16, CONTOUR_COLOR_RED, 16);
                Log.e(ConceptVuMarkIdentification.TAG, "Red Center: (" + (int) centerRed.x + "," + (int) centerRed.y + ") with radius: " + (int) radiusRed[i]);

                if (centerRedBest == null) {
                    centerRedBest = centerRed;
                    radiusRedBest = radiusRed[0];
                } else {
                    if (distance(centerROI, centerRed) < distance(centerROI, centerRedBest)) {
                        centerRedBest = centerRed;
                        radiusRedBest = radiusRed[0];
                    }
                }

            }
            if (centerRedBest != null) {
                Imgproc.circle(mRgba, new Point(centerRedBest.x + rROI.x, centerRedBest.y + rROI.y), (int) radiusRedBest, CONTOUR_COLOR_RED, 16);
            }

            List<Moments> muBlue = new ArrayList<Moments>(contoursBlue.size());
            Point centerBlueBest = null;

            List<MatOfPoint2f> contours2fBlue = new ArrayList<MatOfPoint2f>();
            List<MatOfPoint2f> polyMOP2fBlue = new ArrayList<MatOfPoint2f>();
            List<MatOfPoint> polyMOPBlue = new ArrayList<MatOfPoint>();

            float[] radiusBlue = new float[contoursBlue.size()];

            for (int i = 0; i < contoursBlue.size(); i++) {

                Point centerBlue = new Point();
                muBlue.add(i, Imgproc.moments(contoursBlue.get(i), false));

                contours2fBlue.add(new MatOfPoint2f());
                polyMOP2fBlue.add(new MatOfPoint2f());
                polyMOPBlue.add(new MatOfPoint());

                contoursBlue.get(i).convertTo(contours2fBlue.get(i), CvType.CV_32FC2);
                Imgproc.approxPolyDP(contours2fBlue.get(i), polyMOP2fBlue.get(i), 3, true);
                polyMOP2fBlue.get(i).convertTo(polyMOPBlue.get(i), CvType.CV_32S);

                minEnclosingCircle(polyMOP2fBlue.get(i), centerBlue, radiusBlue);
                Imgproc.circle(mRgba, new Point(centerBlue.x + rROI.x, centerBlue.y + rROI.y), 16, CONTOUR_COLOR_BLUE, 16);
                Log.e(ConceptVuMarkIdentification.TAG, "Blue Center: (" + (int) centerBlue.x + "," + (int) centerBlue.y + ") with radius: " + (int) radiusBlue[i]);

                if (centerBlueBest == null) {
                    centerBlueBest = centerBlue;
                    radiusBlueBest = radiusBlue[0];
                } else {
                    if (distance(centerROI, centerBlue) < distance(centerROI, centerBlueBest)) {
                        centerBlueBest = centerBlue;
                        radiusBlueBest = radiusBlue[0];
                    }
                }
            }
            if (centerBlueBest != null) {
                Imgproc.circle(mRgba, new Point(centerBlueBest.x + rROI.x, centerBlueBest.y + rROI.y), (int) radiusBlueBest, CONTOUR_COLOR_BLUE, 16);
            }

            Mat colorLabel = mRgba.submat(4, 68, 4, 68);
            colorLabel.setTo(mBlobColorRgba);


            if (centerRedBest != null && centerBlueBest != null) {
                message = "";
                if (centerBlueBest.x < centerRedBest.x) {
                    ballArray[0] = BallColor.BLUE;
                    ballArray[1] = BallColor.RED;
                } else {
                    ballArray[0] = BallColor.RED;
                    ballArray[1] = BallColor.BLUE;
                }
                telemetry.addData("balls", Arrays.toString(ballArray));
                Log.e(ConceptVuMarkIdentification.TAG, message);
                incrementState();
            }


            Imgproc.rectangle(mRgba, new Point(rROI.x, rROI.y), new Point(rROI.x + rROI.width, rROI.y + rROI.height), ROI_COLOR, 16);
            //writeImage(mRgba);

        }
    }

    public void centerOnTriangle(Mat inputFrame, int alliance) {
        if(next_state_to_execute()) {
            String ret = "";
            try {
                if (alliance == RED) {
                    Mat mRed = inputFrame;
                    ColorLineDetector lDetectorRed = new ColorLineDetector();
                    lDetectorRed.setColorRange(new Scalar(217, 150, 150, 0), new Scalar(45, 255, 255, 255));
                    lDetectorRed.processLines(mRed);
                    lDetectorRed.showLines(mRed);
                    Point intersect = lDetectorRed.pointIntersect(
                            lDetectorRed.clusters.clusterGroups.get(0).angle,
                            lDetectorRed.clusters.clusterGroups.get(0).center(),
                            lDetectorRed.clusters.clusterGroups.get(1).angle,
                            lDetectorRed.clusters.clusterGroups.get(1).center()
                    );
                    if (lDetectorRed.clusters.clusterGroups.size() > 1) {
                        if (intersect.x > (mRed.cols() / 2) + 20) {
                            set_drive_power(0.05, 0.05);
                            countA = 0;
                        } else if (intersect.x < (mRed.cols() / 2) - 20) {
                            set_drive_power(-0.05, -0.05);
                            countA = 0;
                        } else {
                            countA++;
                        }

                        if (countA > 0) {
                            countA = 0;
                            set_drive_power(0, 0);
                            reset_drive_encoders();
                            incrementState();
                        }
                    } else if (lDetectorRed.clusters.clusterGroups.get(0).center().x > mRed.cols() / 2) {
                        set_drive_power(0.05, 0.05);
                    } else if (lDetectorRed.clusters.clusterGroups.get(0).center().x < mRed.cols() / 2) {
                        set_drive_power(-0.05, -0.05);
                    }
                    telemetry.addData("Center X",lDetectorRed.centerP.x);
                } else if (alliance == BLUE) {
                    Mat mBlue = inputFrame;
                    ColorLineDetector lDetectorBlue = new ColorLineDetector();
                    lDetectorBlue.setColorRange(new Scalar(125, 120, 130, 0), new Scalar(187, 255, 255, 255));
                    lDetectorBlue.processLines(mBlue);
                    lDetectorBlue.showLines(mBlue);
                    Point intersect = lDetectorBlue.pointIntersect(
                            lDetectorBlue.clusters.clusterGroups.get(0).angle,
                            lDetectorBlue.clusters.clusterGroups.get(0).center(),
                            lDetectorBlue.clusters.clusterGroups.get(1).angle,
                            lDetectorBlue.clusters.clusterGroups.get(1).center()
                    );
                    if (lDetectorBlue.clusters.clusterGroups.size() > 1) {
                        if (intersect.x > (mBlue.cols() / 2) + 25) {
                            set_drive_power(0.07, 0.07);
                            countA = 0;
                        } else if (intersect.x < (mBlue.cols() / 2) - 25) {
                            set_drive_power(-0.07, -0.07);
                            countA = 0;
                        } else {
                            countA++;
                        }

                        if (countA > 0) {
                            countA = 0;
                            set_drive_power(0, 0);
                            reset_drive_encoders();
                            incrementState();
                        }
                    } else if (lDetectorBlue.clusters.clusterGroups.get(0).center().x > mBlue.cols() / 2) {
                        set_drive_power(0.05, 0.05);
                    } else if (lDetectorBlue.clusters.clusterGroups.get(0).center().x < mBlue.cols() / 2) {
                        set_drive_power(-0.05, -0.05);
                    }
                    ret = String.valueOf(intersect.x);
                    telemetry.addData("Center X",ret);
                    lDetectorBlue.logToFile(lDetectorBlue.calculateIntersect( lDetectorBlue.clusters.clusterGroups.get(0).angle,
                            lDetectorBlue.clusters.clusterGroups.get(0).center(),
                            lDetectorBlue.clusters.clusterGroups.get(1).angle,
                            lDetectorBlue.clusters.clusterGroups.get(1).center()));
                }
            }catch(Exception e) {
                e.printStackTrace();
            }

        }
    }

    enum BallColor {
        BLUE, RED
    }

}