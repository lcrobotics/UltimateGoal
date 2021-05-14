package org.firstinspires.ftc.teamcode.TeleopOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp
public class ColorSensorNormalizedTest extends OpMode {
    // declare new color sensor
    NormalizedColorSensor colorSensor;

    public void init() {
        // initialize color sensor
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
    }

    @Override
    public void loop() {
        // Get the normalized colors from the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        // print rbg values as telemetry
        telemetry.addLine()
                .addData("Red", colors.red)
                .addData("Green", colors.green)
                .addData("Blue", colors.blue);
        telemetry.update();
    }
}
