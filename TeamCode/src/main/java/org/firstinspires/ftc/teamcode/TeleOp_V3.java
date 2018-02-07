package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "teleOp", group = "Competition")
public class TeleOp_V3 extends StateMachine_v7 {

    StateMachine_v7 relicMachine = new StateMachine_v7();
    Timer relicTimer = new Timer();

    Timer t0 = new Timer();
    Timer t1 = new Timer();

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

    boolean isGrabbingRelic = false;
    boolean grabOS = true;

    boolean isPlacingRelic = false;
    boolean placeOS = true;

    boolean phoneOS = false;
    double phonePos = 0;

    long milliSecondTimeout = 0;



    Orientation axes;

    @Override
    public void init() {
        super.init();
        if(mtrLeftDrive != null)telemetry.addData("mtrLeft", mtrLeftDrive.getMode());

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//
//        parameters.vuforiaLicenseKey = "AXlr9dj/////AAAAGRujCnGOL0aIpjtd4y5IK2sFwI4jstOeOlytTrPr3jzeQQ9tGEHgLmuGdxzuxGNY5641pyXeeyJHccL+I4QCZq8Sodm5DAUBsAQQ9ox1EY3+KNfZISN06k1IqDf7YaRXhE02j+7aE4Apnm3Hvn9V5CDKSTgOq73eJId9uzHkuNaIx+UDV4fRS1HK5L6dSGmIw3+RW7uaFdBF0E2bvWzzpZv51KFzw5oy/9qFB9r6ke5B5Gd2zw9JjafwudFSpLVCiNzIreoTaIFYVMmGMuIFIT4C6oC13EcvbNl5CFpl+irLqhSI//nlL2L2DKxKtW5UTQqNBlOSBdTxWR/kSN12edlwOu0kFgzhKBFapn3KHC0V";
//
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
//
//        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
//
//        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
//        relicTemplate = relicTrackables.get(0);
//        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
//
//        telemetry.addData(">", "Press Play to start");
//        telemetry.update();
//
//        relicTrackables.activate();

    }

    @Override
    public void init_loop() {
//        addIMUSensor("imu");
    }

    @Override
    public void loop() {
        axes = (IMUnav != null) ? IMUnav.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES) : null;

        ////////////////////DRIVE//////////////////////

        speedRatio = gamepad1.right_trigger > .5 ^ gamepad1.right_bumper ? gamepad1.right_bumper ? .25 : 1. : .5;

        if(!isBalancing) {
            set_power(mtrLeftDrive, gamepad1.left_stick_y * speedRatio);
            set_power(mtrRightDrive, gamepad1.right_stick_y * speedRatio);
        }

        ////////////////ARM MOTION/////////////////

        if(!isGrabbingRelic && !isPlacingRelic) {
            set_position(srvClaw, (gamepad2.left_trigger > 0.5) ? CLAWOPEN : CLAWCLOSED);

            set_power(mtrExtend, (!isWithin(gamepad2.left_stick_y, 0.2, 0.2)) ? 0.7 * gamepad2.left_stick_y : 0);

            set_position(srvLevel, gamepad2.left_bumper ? LEVELUP : LEVELDOWN);
        }

        ////////////////////////AUTOMATED RELIC GRABBING/////////////////////

        if(gamepad2.guide && gamepad2.y && grabOS){
            grabOS = false;
            isGrabbingRelic = !isGrabbingRelic;
        }else if(!gamepad2.y) grabOS = true;

        if(isGrabbingRelic && !isPlacingRelic){
            relicMachine.initializeMachine();
            relicMachine.AbsoluteMotorMove(mtrExtend, extendPos.PARTIAL_RETRACT.getVal(), 0.7);
            relicMachine.ServoMove(srvClaw, CLAWOPEN);
            relicMachine.ServoMove(srvLevel, LEVELDOWN);
            if(relicMachine.next_state_to_execute()){
                set_power(mtrExtend, 0.5);
                relicMachine.incrementState();
            }
            if(relicMachine.next_state_to_execute()){
                if(gamepad2.guide){
                    relicMachine.incrementState();
                }
            }
            relicMachine.Pause(300);
            if(relicMachine.next_state_to_execute()){
                set_power(mtrExtend, 0.0);
                set_position(srvClaw, CLAWCLOSED);
                relicMachine.incrementState();
            }
            relicMachine.Pause(500);
            if(relicMachine.next_state_to_execute()){
                set_power(mtrExtend, -0.2);
                relicMachine.incrementState();
            }
            if(relicMachine.next_state_to_execute()){
                if(has_encoder_reached(mtrExtend, extendPos.PARTIAL_RETRACT.getVal()) || gamepad2.guide){
                    set_position(srvLevel, LEVELUP);
                    incrementState();
                }
            }
            relicMachine.AbsoluteMotorMove(mtrExtend, extendPos.HOME.getVal(), 0.4);
            if(relicMachine.next_state_to_execute()){
                isGrabbingRelic = false;
                relicMachine.incrementState();
            }
        }else{
            relicMachine.reset();
        }

        ////////////////////////AUTOMATED RELIC GRABBING/////////////////////

        if(gamepad2.guide && gamepad2.b && placeOS){
            placeOS = false;
            isPlacingRelic = !isPlacingRelic;
        }else if(!gamepad2.b) placeOS = true;

        if(isPlacingRelic && !isGrabbingRelic){
            relicMachine.initializeMachine();
            relicMachine.ServoMove(srvClaw, CLAWCLOSED);
            relicMachine.ServoMove(srvLevel, LEVELUP);
            relicMachine.Pause(300);
            relicMachine.AbsoluteMotorMove(mtrExtend, extendPos.PARTIAL_RETRACT.getVal(), 0.5);
            relicMachine.ServoMove(srvLevel, LEVELPARTIALDOWN);
            relicMachine.AbsoluteMotorMove(mtrExtend, extendPos.FULL_EXTEND.getVal(), 0.3);
            if(relicMachine.next_state_to_execute()){
                double currVal = get_servo_position(srvLevel);
                if(relicTimer.hasWaitFinished(300)) {
                    currVal -= .003;
                    currVal = currVal < 0 ? 0 : currVal;
                    set_position(srvLevel, currVal);
                }
                if(gamepad2.guide || currVal == 0.0){
                    relicMachine.incrementState();
                }
            }
            relicMachine.ServoMove(srvClaw, CLAWOPEN);
            relicMachine.Pause(500);
            if(relicMachine.next_state_to_execute()){
                set_power(mtrExtend, -.3);
                incrementState();
            }
            relicMachine.Pause(750);
            relicMachine.ServoMove(srvClaw, CLAWCLOSED);
            relicMachine.ServoMove(srvLevel, LEVELUP);
            relicMachine.AbsoluteMotorMove(mtrExtend, extendPos.HOME.getVal(), -0.5);
            relicMachine.Pause(750);
            relicMachine.ServoMove(srvLevel, LEVELDOWN);
            if(relicMachine.next_state_to_execute()){
                isPlacingRelic = false;
                relicMachine.incrementState();
            }
        }else{
            relicMachine.reset();
        }


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

        if(((gamepad2.right_trigger > 0.5) || t0.isWaiting()) && liftLevel.equals(liftPos.CARRY)){
            milliSecondTimeout = milliSecondTimeout == 0 ? 1500 : milliSecondTimeout + 800;
            if(t0.hasWaitFinished(250)) {
                liftLevel = liftPos.ONE;
                isLifting = true;
            }
        }else if(((gamepad2.right_trigger <= 0.5) || t1.isWaiting()) && liftLevel.equals(liftPos.ONE)) {
            if (liftLevel.equals(liftPos.ONE)) {
                milliSecondTimeout = milliSecondTimeout == 0 ? 1500 : milliSecondTimeout + 800;
                if (t1.hasWaitFinished(250)) {
                    liftLevel = liftPos.CARRY;
                    isLifting = true;
                }
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
                case CARRY:
                    liftLevel = up ? liftPos.TWO : liftPos.CARRY;
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
            milliSecondTimeout = milliSecondTimeout == 0 ? 1500 : milliSecondTimeout + 800;
        } else if (!gamepad2.x && !gamepad2.a) liftOS = false;

        if (isLifting) {
            run_to_position(mtrLift);
            set_encoder_target(mtrLift, liftLevel.getVal());
            if (!waitHasFinished(milliSecondTimeout)) {
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
            milliSecondTimeout = 0;
            initOS = true;
        }

//        telemetry.addData("isBalancing", isBalancing);
//        telemetry.addData("balanceOS", balanceOS);
//        telemetry.addData("balanceInit", balanceInit);
//        telemetry.addData("encTarget", Arrays.toString(BalEnc));
//        telemetry.addData("currEnc", Arrays.toString(new int[]{get_encoder_count(mtrLeftDrive),
//                                                                       get_encoder_count(mtrRightDrive)}));
        telemetry.addData("liftPos", liftLevel);
        telemetry.addData("isLifting", isLifting);
//        telemetry.addData("isWaiting0", t0.isWaiting());
//        telemetry.addData("elapsedTime0", t0.getElapsedTime());
//        telemetry.addData("isWaiting1", t1.isWaiting());
//        telemetry.addData("elapsedTime1", t1.getElapsedTime());
        telemetry.addData("extendEnc", get_encoder_count(mtrExtend));
        telemetry.addData("Voltage",getBatteryVoltage());
        telemetry.addData("RelicMachine:",relicMachine);
    }
}