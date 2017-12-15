package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name = "TeleOp", group = "Competition")
public class TeleOp_V1 extends StateMachine_v5 {

    StateMachine_v5 robotFront = new StateMachine_v5();

    final int power = 1;
    boolean isFlipMoving = false;
    boolean isLiftMoving = false;

    final int FLIPUP = 1450;
    final int FLIPDOWN = -1450;
    final int LIFTUP = 4000;
    final int LIFTMEDIUM = 3500;
    final int LIFTDOWN = 0;



    double gr1Val = GR1CLOSED;

    double speedRatio = .5;
    double adjustment = 0.0;
    boolean adjOS = true;

    Servo srvP;
    CRServo srvC;

    boolean down = true;
    boolean flipOS = true;

    boolean balanceOS = false;
    boolean isBalancing = true;

    boolean gr1OS = false;

    int liftStage = 0;

    double GR2pos = 0;
    int GR1pos = 0;

    Orientation axes;

    @Override
    public void init() {
        super.init();
        telemetry.addData("mtrLeft", mtrLeftDrive.getMode());
        srvP = addServo(0, "srvP");
        srvC = addCRServo("srvC");
        run_using_encoder(mtrArmFlip);

    }

    @Override
    public void init_loop() {
//        addIMUSensor("imu");
    }

    @Override
    public void loop() {
        axes = IMUnav.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        double pos = Math.abs(((get_encoder_count(mtrArmFlip) * 360 / 1700.)) / 360.);
        pos += adjustment;
        pos = pos >= 1 ? 1 : pos;

        if (adjOS) {
            adjustment += gamepad2.y ? 15. / 270 : gamepad2.b ? -15. / 270 : 0;
            adjOS = false;
        }
        if (!adjOS && !gamepad2.y && !gamepad2.b) adjOS = true;

        if (gamepad2.guide) adjustment = 0.0;

        speedRatio = gamepad1.right_trigger > .5 ^ gamepad1.right_bumper ? gamepad1.right_bumper ? .25 : 1. : .5;

        set_power(mtrLeftDrive, gamepad1.left_stick_y * speedRatio);
        set_power(mtrRightDrive, gamepad1.right_stick_y * speedRatio);

        set_power(mtrArmFlip, (Math.abs(gamepad2.left_stick_y) <= .2 && get_encoder_count(mtrArmFlip) > 0) ? 0 : gamepad2.left_stick_y * .2);

        if(flipOS) set_power(mtrLift, (Math.abs(gamepad2.right_stick_y) <= .3) ? 0 : -gamepad2.right_stick_y * .8);

        set_position(srvShift, (Math.abs(gamepad2.right_stick_x) <= .3) ? 0 : gamepad2.right_stick_x);

        set_position(srvLevel, pos);

        set_position(srvClaw, (gamepad2.left_trigger > 0.5) ? 1 : 0.1);

        set_power(mtrArmSpin, (Math.abs(gamepad2.left_stick_x) <= .2) ? 0 : gamepad2.left_stick_x * .5);

        set_position(srvExtend, gamepad2.dpad_up ? 1 : gamepad2.dpad_down ? -1 : 0);

        snsColorLeft.enableLed(!gamepad1.guide);
        snsColorRight.enableLed(!gamepad1.guide);

        set_position(srvGr1, (gamepad2.right_trigger >= .5) ? GR1OPEN : GR1CLOSED);
//        if(gamepad2.right_bumper && !gr1OS) {
//            gr1Val = (GR1pos == 0) ? GR1OPEN : GR1CLOSED;
//            GR1pos = (GR1pos == 0) ? 1 : 0;
//            gr1OS = true;
//        }
//        if(!gamepad2.right_bumper) gr1OS = false;
        set_position(srvGr2, (gamepad2.right_bumper) ? GR2OPEN : GR2CLOSED);

        if (gamepad1.x && flipOS) {
            flipOS = false;
        }

        if (!flipOS) {
            initializeMachine(robotFront);
            if (!down) {
                if (next_state_to_execute(robotFront)) {
                    run_using_encoder(mtrLift);
                    set_encoder_target(mtrLift, LIFTUP);
                    set_power(mtrLift, 0.8);

                    if (Math.abs(get_encoder_count(mtrLift)) + 20 >= mtrLift.getTargetPosition()) {
                        set_power(mtrLift, 0);
                        incrementState(robotFront);

                    }
                }

                if (next_state_to_execute(robotFront)) {
                    run_to_position(mtrFlip);
                    set_encoder_target(mtrFlip, FLIPUP);
                    set_power(mtrFlip, 0.8);

                    if (has_encoder_reached(mtrFlip, FLIPUP)) {
                        reset_encoders(mtrFlip);
                        set_power(mtrFlip, 0);
                        incrementState(robotFront);

                    }
                }

                if (next_state_to_execute(robotFront)) {
                    run_using_encoder(mtrLift);
                    set_encoder_target(mtrLift, LIFTDOWN);
                    set_power(mtrLift, -0.8);

                    if (get_encoder_count(mtrLift) - 20 <= mtrLift.getTargetPosition()) {
                        set_power(mtrLift, 0);
                        incrementState(robotFront);

                    }
                }



                if (next_state_to_execute(robotFront)) {
                    resetMachine(robotFront);
                    run_using_encoder(mtrLift);
                    down = true;
                    flipOS = true;

                }
            } else {
                if(next_state_to_execute(robotFront)) {
                    run_using_encoder(mtrLift);
                    set_encoder_target(mtrLift, LIFTUP);
                    set_power(mtrLift, 0.8);

                    if (get_encoder_count(mtrLift) + 20 >= LIFTUP) {
                        set_encoder_target(mtrLift, 0);
                        set_power(mtrLift, 0);
                        incrementState(robotFront);

                    }
                }

                if (next_state_to_execute(robotFront)) {
                    run_to_position(mtrFlip);
                    set_encoder_target(mtrFlip, FLIPDOWN);
                    set_power(mtrFlip, -0.7);
                    srvGr1.setPosition(GR1CLOSED);
                    srvGr2.setPosition(GR2CLOSED);
                    if (has_encoder_reached(mtrFlip, FLIPDOWN)) {
                        reset_encoders(mtrFlip);
                        set_power(mtrFlip, 0);
                        incrementState(robotFront);
                        srvGr1.setPosition(GR1CLOSED);
                    }
                }

                if (next_state_to_execute(robotFront)) {
                    run_using_encoder(mtrLift);
                    set_encoder_target(mtrLift, LIFTMEDIUM);
                    set_power(mtrLift, -0.8);
                    srvGr1.setPosition(GR1CLOSED);
                    if (get_encoder_count(mtrLift) - 30 <= LIFTMEDIUM) {
                        set_power(mtrLift, 0);
                        incrementState(robotFront);
                        srvGr1.setPosition(GR1CLOSED);
                    }
                }

                if (next_state_to_execute(robotFront)) {
                    resetMachine(robotFront);
                    run_using_encoder(mtrLift);
                    down = false;
                    flipOS = true;
                    srvGr1.setPosition(GR1CLOSED);
                }
            }
        }
        if(gamepad1.guide && gamepad1.a && !balanceOS) {
            balanceOS = true;
            isBalancing = !isBalancing;
        }else balanceOS = false;

        if(isBalancing) {
            if(axes.secondAngle > 3) {
                mtrLeftDrive.setPower(0.2);
                mtrRightDrive.setPower(0.2);
            } else if(axes.secondAngle < -3) {
                mtrLeftDrive.setPower(-0.2);
                mtrRightDrive.setPower(-0.2);
            } else isBalancing = false;
        }
        //srvGr1.setPosition(gr1Val);
//        telemetry.addData("GR1pos", GR1pos);
//        telemetry.addData("ArmFlipEnc", get_encoder_count(mtrArmFlip));
//        telemetry.addData("LiftEnc", get_encoder_count(mtrLift));
//        telemetry.addData("LiftTarget", mtrLift.getTargetPosition());
//        telemetry.addData("FlipEnc", get_encoder_count(mtrArmFlip));
//        telemetry.addData("SrvLevel", get_servo_position(srvLevel));
        telemetry.addData("rightblue", snsColorRight.blue());
        telemetry.addData("rightargb",snsColorRight.argb());
        telemetry.addData("rightred", snsColorRight.red());
        telemetry.addData("leftblue",snsColorLeft.blue());
        telemetry.addData("leftargb",snsColorLeft.argb());
        telemetry.addData("leftred",snsColorLeft.red());
    }
}