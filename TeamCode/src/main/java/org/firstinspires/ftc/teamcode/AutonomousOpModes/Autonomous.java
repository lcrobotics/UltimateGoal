package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import org.firstinspires.ftc.teamcode.SuperOp;

public class Autonomous extends SuperOp {

    int auto = 0;

    final int DRIVE = 0;
    final int CLOCKWISE = 1;
    final int DELTA = 2;
    final int COUNTERCLOCKWISE = 3;
    final int SHOOT = 4;

    boolean picture;

    @Override
    public void init() {
        super.init();
    }

    public void loop() {
        //drive forward
        switch (auto) {

            case DRIVE:
                //drive forward
                auto = CLOCKWISE;
                break;

            case CLOCKWISE:
                //Clockwise rotation

                if (picture) {
                    auto = DELTA;
                } else {
                    auto = COUNTERCLOCKWISE;
                }
                break;

            case DELTA:
                //find traslational delta

                //find rotational delta

                //if deltas in threshold, go to SHOOT

                //if not, go to DELTA
                break;

            case COUNTERCLOCKWISE:
                //Counterclockwise rotation

                if (picture) {
                    auto = DELTA;
                } else {
                    auto = CLOCKWISE;
                }
                break;

            case SHOOT:
                //shoot
                break;
        }
    }

    public boolean pictureFound() {
            return false;
    }
}
