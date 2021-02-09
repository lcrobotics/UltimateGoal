package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.AutonomousOpModes.VuforiaSuperOp;

import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@Autonomous
public class VuforiaSuperOpTest extends VuforiaSuperOp {
    public void loop(){

        frameGetter.updateFrame();
        telemetry.addData("image dims", String.format(Locale.US, "%d %d", frameGetter.imgWidth, frameGetter.imgHeight));
        int w = 100, h = 100;
        frameGetter.updateMaxRect(0, w, h);
        telemetry.addData(
                String.format(Locale.US, "Max %dx%d rect", w, h),
                String.format(Locale.US, "%d %d", frameGetter.xMax, frameGetter.yMax));

        objectLocator.updateRobotLocation();
        lastLocation = objectLocator.lastLocation;
        // Provide feedback as to where the robot is located (if we know).
        if (objectLocator.targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        }
        else {
            telemetry.addData("Visible Target", "none");
        }

        telemetry.update();
    }
}
