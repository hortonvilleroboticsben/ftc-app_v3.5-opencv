package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by kungl on 1/21/2018.
 */
@Deprecated
//@TeleOp(name = "BenVisio", group = "Competition")
public class BenVision extends StateMachine_v6 {

    @Override
    public void init(){
        super.init();
    }

    @Override
    public void loop(){
            set_drive_power(gamepad1.left_stick_y, gamepad1.right_stick_y);
            set_drive_power(.1, .1);
            set_drive_target(0, 0);
            run_drive_to_position();
            if (get_encoder_count(mtrLeftDrive) >= -10 && get_encoder_count(mtrLeftDrive) <= 10)
                set_drive_power(0, 0);
           // if(set_drive_target(0,1000)> );
            telemetry.addData("Left Encoder", get_encoder_count(mtrLeftDrive));
            telemetry.addData("Right Enconder", get_encoder_count(mtrRightDrive));
    }

}
