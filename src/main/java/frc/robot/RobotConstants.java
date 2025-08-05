package frc.robot;

import frc.robot.utils.TunableNumber;

public class RobotConstants {
    public static final boolean TUNING = true;

    public static class TankConstants {
        public static double MAX_SPEED_METERS_PER_SECOND = 3.5;
        public static double GEAR_RATIO = 12.0;

        public static class TankPID {
            public static final TunableNumber kP = new TunableNumber("TANK_PID/KP", 0.1);
            public static final TunableNumber kI = new TunableNumber("TANK_PID/KI", 0);
            public static final TunableNumber kD = new TunableNumber("TANK_PID/KD", 0);
        }
    }

}