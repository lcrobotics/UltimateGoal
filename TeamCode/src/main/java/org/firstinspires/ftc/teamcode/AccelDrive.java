package org.firstinspires.ftc.teamcode;

import android.telephony.TelephonyManager;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AccelDrive {
    private DcMotor motor;

    private ElapsedTime time;
    private double totalTime;
    private double percentSustain;
    private double maxPower;
    private boolean finished;

    // These two variables are not necessary, but are used for convenience
    double timeAccelEnd;
    double timeDeccelStart;


    AccelDrive(DcMotor motor) {
        this.motor = motor;
        finished = true;
        time = new ElapsedTime();
    }

    public void set(double totalTime, double percentSustain, double maxPower) {
        finished = false;
        time.reset();
        this.totalTime = totalTime;
        this.maxPower = maxPower;
        this.percentSustain = percentSustain;

        timeAccelEnd = (totalTime * (1 - percentSustain) / 2);
        timeDeccelStart = totalTime * percentSustain + timeAccelEnd;
    }


    double tempPower = 0;
    double currentTime;
    /*
                y
        ________________
       /                \
   x1 /                  \ x2
     /                    \
    /                      \
     Power / Time graph
     x1 = x2
     current time is c
     */


    public void update(){
        currentTime = time.milliseconds();
        // when c < x1 (power increasing)
        if (currentTime < timeAccelEnd) {
            tempPower = (currentTime / timeAccelEnd) * maxPower;
        // when x1 < c < x2 (power constant)
        } else if (currentTime < timeDeccelStart) {
            tempPower = maxPower;
        /// when x1 + y <  c < x1 + x2 + y  (power decreasing)
        } else if (currentTime < totalTime) {
            // the inverse ration of current time to descent time multiplied by the max power.
            tempPower = ((timeAccelEnd - (currentTime - timeDeccelStart)) / timeAccelEnd) * maxPower;
        // edge case (just in case)
        } else {
            tempPower = 0;
        }


        motor.setPower(tempPower);
    }
}
