package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@TeleOp(name = "teleOp", group = "Competition")
public class TeleOp_V2 extends StateMachine_v6 {

    StateMachine_v6 robotFront = new StateMachine_v6();


    final int LIFTUP = 4000;
    final int LIFTMEDIUM = 3600;
    final int LIFTDOWN = 150;

    double speedRatio = .5;
    double adjustment = 0.0;
    boolean adjOS = true;

    boolean glyphOS = true;
    boolean glyphPos = true;

    boolean balanceOS = false;
    boolean isBalancing = false;

    boolean liftOS = false;
    boolean isLifting = false;
    liftPos liftLevel = liftPos.ONE;
    
    boolean OS1 = false;
    double lrVal = 0.45 ;
    boolean OS2 = false;
    double udVal = 0.7;

    boolean phoneOS = false;
    double phonePos = 0;

    enum liftPos{
        ONE(0),TWO(-721),THREE(-1238),FOUR(-1826);

        private int val;

        liftPos(int val){
            this.val = val;
        }

        public int getVal(){
            return val;
        }
    }

    Orientation axes;

    @Override
    public void init() {
        super.init();
        if(mtrLeftDrive != null)telemetry.addData("mtrLeft", mtrLeftDrive.getMode());

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

    }

    @Override
    public void init_loop() {
//        addIMUSensor("imu");
    }

    @Override
    public void loop() {
        axes = (IMUnav != null) ? IMUnav.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES) : null;

        ////////////////LEVELING CONTROL////////////////////////

        double pos = Math.abs(((get_encoder_count(mtrArmFlip) * 360 / 1700.)) / 360.);
        pos += adjustment;
        pos = pos >= 1 ? 1 : pos;

        if (adjOS) {
            adjustment += gamepad2.y ? 15. / 270 : gamepad2.b ? -15. / 270 : 0;
            adjOS = false;
        }
        if (!adjOS && !gamepad2.y && !gamepad2.b) adjOS = true;

        if (gamepad2.guide) adjustment = 0.0;

        ////////////////////DRIVE//////////////////////

        speedRatio = gamepad1.right_trigger > .5 ^ gamepad1.right_bumper ? gamepad1.right_bumper ? .25 : 1. : .5;

        set_power(mtrLeftDrive, gamepad1.left_stick_y * speedRatio);
        set_power(mtrRightDrive, gamepad1.right_stick_y * speedRatio);

        ////////////////ARM MOTION/////////////////

        set_position(srvLevel, pos);

        set_position(srvClaw, (gamepad2.left_trigger > 0.5) ? 0.0 : 1.0);

        set_power(mtrArmSpin, (Math.abs(gamepad2.left_stick_x) <= .2) ? 0 : gamepad2.left_stick_x * .5);

        set_power(mtrArmFlip, ((Math.abs(gamepad2.left_stick_y) <= .2)) ? 0 : gamepad2.left_stick_y * .2);

        set_position(srvExtend, gamepad2.dpad_up ? 1 : gamepad2.dpad_down ? -1 : 0);


        /////////////////AUTOBALANCE//////////////////////

        if(gamepad1.guide && gamepad1.a && !gamepad1.start && !balanceOS) {
            balanceOS = true;
            isBalancing = !isBalancing;
        } else if(!gamepad1.guide && !gamepad1.a)balanceOS = false;

        if(isBalancing) {
            if(axes.secondAngle > 3) {
                mtrLeftDrive.setPower(0.2);
                mtrRightDrive.setPower(0.2);
            } else if(axes.secondAngle < -3) {
                mtrLeftDrive.setPower(-0.2);
                mtrRightDrive.setPower(-0.2);
            } else {
                isBalancing = false;
            }

        }

        ////////////////GLYPH MANIPULATOR//////////////////////

        set_position(srvGr1,(gamepad2.right_trigger > 0.5) ? GR1OPEN : GR1CLOSED);
        set_position(srvGr2,(gamepad2.right_trigger > 0.5) ? GR2OPEN : GR2CLOSED);

//        if(gamepad1.dpad_left ^ gamepad1.dpad_right && !OS1) {
//            OS1 = true;
//            lrVal += gamepad1.dpad_left ? 0.005 : -0.005;
//            lrVal = lrVal > 1 ? 1 : lrVal;
//            lrVal = lrVal < -1 ? -1 : lrVal;
//        }else if(!gamepad1.dpad_left && !gamepad1.dpad_right) OS1 = false;
//
//        if(gamepad1.dpad_down ^ gamepad1.dpad_up && !OS2) {
//            OS2 = true;
//            udVal += gamepad1.dpad_down ? 0.005 : -0.005;
//            udVal = udVal > 1 ? 1 : udVal;
//            udVal = udVal < -1 ? -1 : udVal;
//        }else if(!gamepad1.dpad_down && !gamepad1.dpad_up) OS2 = false;
//
//        if(gamepad1.guide){
//            lrVal = 0.45;
//            udVal = 0.7;
//        }
//
//        set_position(srvLR, lrVal);
//        set_position(srvUD, udVal);

        ///////////////////////PHONE///////////////////////////

//        if(gamepad1.x ^ gamepad1.b && !gamepad1.start && !phoneOS){
//            phoneOS = true;
//            phonePos += gamepad1.x ? 0.005 : -0.005;
//            phonePos = phonePos > 1 ? 1 : phonePos;
//            phonePos = phonePos < -1 ? -1 : phonePos;
//        }else if(!gamepad1.x && !gamepad1.b) phoneOS = false;
//
//        set_position(srvPhone, phonePos);

        //////////////////////LIFTING//////////////////////////

        if(!isLifting) set_power(mtrLift, .75*gamepad2.right_stick_y);

        if (((gamepad2.x ^ gamepad2.a) && !gamepad2.start) && !liftOS) {
            isLifting = true;
            liftOS = true;
            boolean up = gamepad2.x;
            switch (liftLevel) {
                case ONE:
                    liftLevel = up ? liftPos.TWO : liftPos.ONE;
                    break;
                case TWO:
                    liftLevel = up ? liftPos.THREE : liftPos.ONE;
                    break;
                case THREE:
                    liftLevel = up ? liftPos.FOUR : liftPos.TWO;
                    break;
                case FOUR:
                    liftLevel = up ? liftPos.FOUR : liftPos.THREE;
                    break;
            }
        } else if (!gamepad2.x && !gamepad2.a) liftOS = false;

        if (isLifting) {
            run_to_position(mtrLift);
            set_encoder_target(mtrLift, liftLevel.getVal());
            set_power(mtrLift, 0.5);
            if (get_encoder_count(mtrLift) > mtrLift.getTargetPosition()) {
                if (has_encoder_reached(mtrLift, liftLevel.getVal())) {
                    run_using_encoder(mtrLift);
                    isLifting = false;
                }
            } else {
                if (!has_encoder_reached(mtrLift, liftLevel.getVal())) {
                    run_using_encoder(mtrLift);
                    isLifting = false;
                }
            }
        }

        telemetry.addData("lrVal","%.3f", lrVal);
        telemetry.addData("udVal","%.3f", udVal);
        telemetry.addData("phoneVal","%.3f", phonePos);
    }
}