package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="HoldPosTest")
public class HoldTest extends StateMachine_v6 {
    static DcMotor mtrHold = null;

    boolean toggle = false;
    boolean toggleOS = true;
    int holdPos = 0;

    @Override
    public void init() {
        super.init();
        mtrHold = addMotor(MotorLocation.SHIFT, "mtrHold");
    }

    @Override
    public void loop() {
        if(gamepad1.guide && toggleOS){
            toggle = !toggle;
            toggleOS = false;
            holdPos = get_encoder_count(mtrHold);
            run_using_encoder(mtrHold);
        }else if(!gamepad1.guide) toggleOS = true;

        if(toggle){
            run_to_position(mtrHold);
            set_encoder_target(mtrHold, holdPos);
            set_power(mtrHold, .2);
        }else{
            set_power(mtrHold, gamepad1.left_stick_y);
        }
        telemetry.addData("holdPos", holdPos);
        telemetry.addData("isHolding", toggle);
        telemetry.addData("currEnc", get_encoder_count(mtrHold));
    }
}
