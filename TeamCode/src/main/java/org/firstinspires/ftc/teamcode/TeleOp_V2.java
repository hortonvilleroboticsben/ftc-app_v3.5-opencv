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

import java.util.Arrays;


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
    boolean balanceInit = true;
    int[] BalEnc = new int[2];

    boolean liftOS = false;
    boolean isLifting = false;
    liftPos liftLevel = liftPos.ONE;

    boolean OS1 = false;
    double lrVal = 0.45 ;
    boolean OS2 = false;
    double udVal = 0.7;

    boolean phoneOS = false;
    double phonePos = 0;

    long milliSeocondTimeout = 0;



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

        if(!isBalancing) {
            set_power(mtrLeftDrive, gamepad1.left_stick_y * speedRatio);
            set_power(mtrRightDrive, gamepad1.right_stick_y * speedRatio);
        }

        ////////////////ARM MOTION/////////////////

        set_position(srvLevel, pos);

        set_position(srvClaw, (gamepad2.left_trigger > 0.5) ? 0.0 : 1.0);

        if(!gamepad2.left_stick_button) {
            set_power(mtrArmSpin, (Math.abs(gamepad2.left_stick_x) <= .2) ? 0 : gamepad2.left_stick_x * .5);

            set_power(mtrArmFlip, ((Math.abs(gamepad2.left_stick_y) <= .2)) ? 0 : gamepad2.left_stick_y * .2);
        }else{
            set_power(mtrArmSpin, (Math.abs(gamepad2.left_stick_x) <= .2) ? 0 : gamepad2.left_stick_x * .1);

            set_power(mtrArmFlip, ((Math.abs(gamepad2.left_stick_y) <= .2)) ? 0 : gamepad2.left_stick_y * .1);
        }
        set_position(srvExtend, gamepad2.dpad_up ? 1 : gamepad2.dpad_down ? -1 : 0);


        /////////////////AUTOBALANCE//////////////////////

        if(gamepad1.guide && gamepad1.a && !gamepad1.start && !balanceOS) {
            balanceOS = true;
            isBalancing = !isBalancing;
            balanceInit = true;
            run_using_drive_encoders();
        } else if(!gamepad1.guide && !gamepad1.a)balanceOS = false;

        if(isBalancing) {
           run_drive_to_position();
           if(balanceInit){
               BalEnc[0] = get_encoder_count(mtrLeftDrive);
               BalEnc[1] = get_encoder_count(mtrRightDrive);
               balanceInit = false;
           }
           set_drive_target(BalEnc[0], BalEnc[1]);
           if(isWithin(get_encoder_count(mtrLeftDrive), BalEnc[0]+10, BalEnc[0]-10) &&
              isWithin(get_encoder_count(mtrRightDrive), BalEnc[1]+10, BalEnc[1]-10))
                set_drive_power(.15, .15);
           else
                set_drive_power(0,0);
        }

        ////////////////GLYPH MANIPULATOR//////////////////////

        set_position(srvGr1,(gamepad2.right_trigger > 0.5) ? GR1OPEN : GR1CLOSED);
        set_position(srvGr2,(gamepad2.right_trigger > 0.5) ? GR2OPEN : GR2CLOSED);
        if(gamepad2.right_trigger > 0.5){
            if(liftLevel.equals(liftPos.CARRY)) {
                liftLevel = liftPos.ONE;
                isLifting = true;
            }
        }else{
            if(liftLevel.equals(liftPos.ONE)) {
                liftLevel = liftPos.CARRY;
                isLifting = true;
            }
        }

//        if(gamepad1.dpad_left ^ gamepad1.dpad_right && !OS1) {
//            OS1 = true;
//            srv1Val += gamepad1.dpad_left ? 0.005 : -0.005;
//            srv1Val = srv1Val > 1 ? 1 : srv1Val;
//            srv1Val = srv1Val < -1 ? -1 : srv1Val;
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
//            srv1Val = 0.45;
//            udVal = 0.7;
//        }
//
//        set_position(srvLR, srv1Val);
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

        if(!isLifting && gamepad2.guide && gamepad2.back) reset_encoders(mtrLift);

        if(!gamepad2.right_stick_button)
            if(!isLifting) set_power(mtrLift, .75*gamepad2.right_stick_y);
        else
            if(!isLifting) set_power(mtrLift, .1*gamepad2.right_stick_y);

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
            milliSeocondTimeout = milliSeocondTimeout == 0 ? 1500 : milliSeocondTimeout + 800;
        } else if (!gamepad2.x && !gamepad2.a) liftOS = false;

        if (isLifting) {
            run_to_position(mtrLift);
            set_encoder_target(mtrLift, liftLevel.getVal());
            if (!waitHasFinished(milliSeocondTimeout)) {
                if (get_encoder_count(mtrLift) > mtrLift.getTargetPosition()) {
                    if (has_encoder_reached(mtrLift, liftLevel.getVal())) {
                        run_using_encoder(mtrLift);
                        isLifting = false;
                    } else set_power(mtrLift, .5);
                } else {
                    if (!has_encoder_reached(mtrLift, liftLevel.getVal())) {
                        run_using_encoder(mtrLift);
                        isLifting = false;
                    } else set_power(mtrLift, .5);
                }
            }else{
                run_using_encoder(mtrLift);
                isLifting = false;
            }
        }else{
            milliSeocondTimeout = 0;
            initOS = true;
        }

        telemetry.addData("isBalancing", isBalancing);
        telemetry.addData("balanceOS", balanceOS);
        telemetry.addData("balanceInit", balanceInit);
        telemetry.addData("encTarget", Arrays.toString(BalEnc));
        telemetry.addData("currEnc", Arrays.toString(new int[]{get_encoder_count(mtrLeftDrive),
                                                                       get_encoder_count(mtrRightDrive)}));
    }
}