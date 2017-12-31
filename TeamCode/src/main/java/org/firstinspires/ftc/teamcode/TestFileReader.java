package org.firstinspires.ftc.teamcode;


import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;

@TeleOp(name = "TestFileReader")
public class TestFileReader extends StateMachine_v5 {
    TextCodeLink link;
    static StateMachine_v5 runner = new StateMachine_v5();

    @Override
    public void init() {
        super.init();
        link = new TextCodeLink(this.getClass(), new File(Environment.getExternalStorageDirectory()+"/Code"), "test.code");

        link.parseFile();
    }

    @Override
    public void loop() {
        initializeMachine(runner);
        try {
            link.parseFile();
            Log.println(Log.ASSERT, "link", link.keys.toString());
            link.runFile();
            telemetry.addData("runner", runner.toString());
        }catch(Exception e){
            e.printStackTrace();
        }
    }

    @TextLink(key="Drive")
    public static void drive(int distance, double power){
        runner.Drive(runner, distance, power);
    }
}