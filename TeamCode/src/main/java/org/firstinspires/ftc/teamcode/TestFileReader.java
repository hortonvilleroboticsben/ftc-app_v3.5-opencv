package org.firstinspires.ftc.teamcode;


import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;
@Deprecated
//@TeleOp(name = "TestFileReader")
public class TestFileReader extends StateMachine_v6 {
    TextCodeLink link;
    static StateMachine_v6 runner = new StateMachine_v6();
    StateMachine_v6 drive = new StateMachine_v6(),
            arm = new StateMachine_v6(),
            glyph = new StateMachine_v6(),
            vision = new StateMachine_v6();

    @Override
    public void init() {
        super.init();
        link = new TextCodeLink(this.getClass(), new File(Environment.getExternalStorageDirectory()+"/Code"), "test.code");

        link.parseFile();
    }

    @Override
    public void loop() {
        runner.initializeMachine();
        vision.initializeMachine();
        drive.initializeMachine();
        arm.initializeMachine();
        glyph.initializeMachine();

        drive.ServoMove(srvPhone, CAM_VUMARK);
        drive.Pause(1000);
        drive.SetFlag(vision, "Read Relic");

        vision.WaitForFlag("Read Relic");
        vision.ProcessRelic();
        vision.SetFlag(drive, "Relic Read");

        drive.WaitForFlag("Relic Read");
        drive.ServoMove(srvPhone, CAM_JEWELS);
        drive.Pause(1000);
        drive.SetFlag(vision, "Read Jewels");

        vision.WaitForFlag("Read Jewels");
        vision.ProcessJewels(vuforiaFrameToMat());
        vision.SetFlag(arm, "Jewels Read");

        arm.WaitForFlag("Jewels Read");
        arm.ServoMove(srvLR, LR_CENTER);
        arm.ServoMove(srvUD, UD_DOWN + .04);
        arm.Pause(800);
        arm.ServoMove(srvUD, UD_DOWN);
        arm.Pause(500);
        if (arm.next_state_to_execute()) {
            if (ballArray[0] == StateMachine_v6.BallColor.BLUE) set_position(srvLR, LR_LEFT);
            else set_position(srvLR, LR_RIGHT);
            arm.incrementState();
        }
        arm.Pause(500);
        arm.SetFlag(drive, "Jewels Hit");
        arm.ServoMove(srvUD, UD_UP);
        arm.ServoMove(srvLR, LR_HOME);

        runner.WaitForFlag("Jewels Hit");

        try {
            link.parseFile();
            Log.println(Log.ASSERT, "link", link.keys.toString());
            Log.println(Log.ASSERT, "content", link.fileContent.toString());
            link.runFile();
            telemetry.addData("runner", runner.toString());
        }catch(Exception e){
            e.printStackTrace();
        }
    }

    @TextLink(key="Drive")
    public static void drive(double distance, double power){
        runner.Drive(distance, power);
    }

    @TextLink(key = "Turn")
    public static void turn(double degrees, double power){
        runner.Turn(degrees, power);
    }

    @TextLink(key="OWTurn")
    public static void owturn(double degrees, double power){
        runner.OWTurn(degrees, power);
    }
}