package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by sam on 4/12/2018.
 */

@TeleOp(name = "BlobbyFinder", group = "Testing")
public class VisionTesty extends StateMachine_v8 {

    ColorBlobDetector blobbyFinder;

    public void init() {
        super.init();
        blobbyFinder = new ColorBlobDetector();
    }

    public void loop() {
        blobbyFinder.processLines(mRgba);
        blobbyFinder.showLines(mRgba);
    }

}
