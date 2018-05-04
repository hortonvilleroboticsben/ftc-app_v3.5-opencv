package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by sam on 4/12/2018.
 */

@TeleOp(name = "VisionTesty", group = "Testing")
public class VisionTesty extends StateMachine_v8 {

    StateMachine_v8 st;

    ColorBlobDetector blobbyFinder;

    public void init() {
        super.init();
        st = new StateMachine_v8();
        st.initializeMachine();

    }

    public void loop() {
        Log.println(Log.ASSERT,"Loop Start","");
        //st.Drive(10,0.5);

        st.runVisionStuff();
        telemetry.addData("Hello, World",""+new java.util.Date().getTime()+"");
        telemetry.addData("State ",st.current_number);
        Log.println(Log.ASSERT,"Loop End","");
        st.current_number = 0;
    }

}
