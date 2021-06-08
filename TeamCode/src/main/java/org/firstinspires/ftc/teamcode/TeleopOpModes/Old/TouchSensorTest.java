package org.firstinspires.ftc.teamcode.TeleopOpModes.Old;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

//@TeleOp
public class TouchSensorTest extends OpMode {
    // declare touch sensor
    TouchSensor touch;

    public void init() {
        // initialize touch sensor
        touch = hardwareMap.get(TouchSensor.class, "touch_sensor");
    }


    @Override
    public void loop() {
        // send the info back to driver station using telemetry function.
        // if the digital channel returns true it's HIGH and the button is unpressed.
        if (touch.isPressed()) {
            telemetry.addData("Touch", "Is Pressed");
        } else {
            telemetry.addData("Touch", "Is Not Pressed");
        }

        telemetry.update();
    }
}
