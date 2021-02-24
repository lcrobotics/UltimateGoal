package org.firstinspires.ftc.teamcode.AutonomousOpModes;

public enum AutoState {
    DRIVE(0),
    CLOCKWISE(1),
    ANGLE(2),
    SIDEWAYS(3),
    BACK(4),
    COUNTERCLOCKWISE(5),
    SHOOT(6),
    PARK(7),
    FAIL(8),
    STOPCHECK(9);

    int val;

    AutoState(int value) {
        val = value;
    }
}
