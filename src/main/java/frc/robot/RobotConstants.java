package frc.robot;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.utils.TunableNumber;

import static edu.wpi.first.units.Units.*;

public class RobotConstants {
    public static final boolean TUNING = true;

    public static class TankConstants {
        public static LinearVelocity MAX_SPEED = MetersPerSecond.of(3.5);
        public static AngularVelocity MAX_ANGULAR_SPEED = RadiansPerSecond.of(Math.PI);// rad/s
        public static double GEAR_RATIO = 12.0;
        public static Distance WHEEL_TRACK = Meters.of(0.548005);// m
        public static Distance WHEEL_RADIUS = Inches.of(3);// m

        public static class TankPID {
            public static final TunableNumber KP = new TunableNumber("TANK_PID/KP", 0.1);
            public static final TunableNumber KI = new TunableNumber("TANK_PID/KI", 0);
            public static final TunableNumber KD = new TunableNumber("TANK_PID/KD", 0);
        }
    }

}