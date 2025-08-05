package frc.robot;

import frc.robot.utils.TunableNumber;

public class RobotConstants {
    public static final boolean TUNING = true;

    public static class TankPID {
        public static final TunableNumber kP = new TunableNumber("Tank_PID/KP", 0.1);
        public static final TunableNumber kI = new TunableNumber("Tank_PID/KI", 0);
        public static final TunableNumber kD = new TunableNumber("Tank_PID/KD", 0);
    }
}
