package org.firstinspires.ftc.teamcode.TeleopOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;


public class ColorSensorTest extends OpMode {
    ColorSensor colorSensor;

    public void init() {
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
    }

    public void loop() {
        telemetry.addLine()
                .addData("red", colorSensor.red())
                .addData("green", colorSensor.green())
                .addData("blue", colorSensor.blue())
                .addData("alpha", colorSensor.alpha());
        telemetry.update();
    }
}
