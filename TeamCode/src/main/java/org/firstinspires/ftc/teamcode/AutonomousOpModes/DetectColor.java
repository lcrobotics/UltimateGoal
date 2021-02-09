package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.VuforiaFrameGetter;

import java.util.Locale;

@Autonomous
public class DetectColor extends OpMode {
    private final static String VUFORIA_KEY = "ARgYuCf/////AAABmUYfc1+dVEQsgUBCPA2kCAFRmuTRB/XUfAJzLsRyFDRg6uMMjj6EXM8YNiY5l3oTw83H+PKgfF46gctdzrln2nnVXMebpgN9ULy1cOfdSsPk0hwSZqzcY0LWCj+rPPrZ3JyQT7gf2aw7bo8ZvWedWB7skuGIjg+9cyTJdDyXmXrQ8Bo4r4siTFNTVFxg21OH/Gd8wrVJF4RqjE+kcez3MzcnE2EPCqWTNixSge5yLg+tN87/R/dMPzqHWvmjE6F6J/7/sahPt7FQ9G6tYWnV1impzZsH7T/JT6pGr2SALwHdaNjBGbYY76ZfvAxixEdob9g6qMBhKOyLg6HTP9VzRZ06ksUhErmR2K2LSkyjxBBz";

    private VuforiaLocalizer vuforia;

    private VuforiaFrameGetter frameGetter = null;

    @Override
    public void init() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.

        // This is necessary for getting pixels (integral image goal detection, etc)
        boolean[] results = vuforia.enableConvertFrameToFormat(PIXEL_FORMAT.RGB565, PIXEL_FORMAT.YUV);
        if (!results[0]) { // Failed to get Vuforia to convert to RGB565.
            throw new RuntimeException("Unable to convince Vuforia to generate RGB565 frames!");
        }
        vuforia.setFrameQueueCapacity(1);
        frameGetter = new VuforiaFrameGetter(vuforia.getFrameQueue());
    }

    @Override
    public void loop() {
        frameGetter.updateFrame();
        telemetry.addData("image dims", String.format(Locale.US, "%d %d", frameGetter.imgWidth, frameGetter.imgHeight));
        /*
        for (int h = 0; h < 5; h++) {
            for (int j = 0; j < 5; j++) {
                int red = frameGetter.rgbValues[0][h][j];
                int green = frameGetter.rgbValues[1][h][j];
                int blue = frameGetter.rgbValues[2][h][j];
                telemetry.addData(String.format(Locale.US, "pixel %d %d", h, j),
                        String.format(Locale.US, "red: %d green: %d blue: %d", red, green, blue));
            }
        }
         */
        int w = 100, h = 100;
        frameGetter.updateMaxRect(0, w, h);
        telemetry.addData(
                String.format(Locale.US, "Max %dx%d rect", w, h),
                String.format(Locale.US, "%d %d", frameGetter.xMax, frameGetter.yMax));
        //telemetry.addData("Rect sum", frameGetter.sumOfRect(0, 0, 0, 200, 200));
        telemetry.update();
    }
}

