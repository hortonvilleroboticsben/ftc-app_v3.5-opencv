package org.firstinspires.ftc.teamcode;

/**
 * Created by sam on 3/24/2018.
 */

public abstract class TestableVisionOpMode extends VisionOpMode {

    /**
     * Creates the Testable OpMode.
     */
    public TestableVisionOpMode() {
        super(false); //disable OpenCV core functions
    }

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        super.loop();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
