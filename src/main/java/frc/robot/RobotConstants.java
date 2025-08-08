package frc.robot;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.utils.TunableNumber;

import static edu.wpi.first.units.Units.*;

public class RobotConstants {
    public static final boolean TUNING = true;

    public static class TankConstants {
        public static LinearVelocity MAX_SPEED = MetersPerSecond.of(2);
        public static AngularVelocity MAX_ANGULAR = RadiansPerSecond.of(0.5 * Math.PI);
        public static double GEAR_RATIO = 12.0;
        public static Distance WHEEL_TRACK = Meters.of(0.548005);
        public static Distance WHEEL_RADIUS = Inches.of(3);


        public static class TankPID {
            public static final TunableNumber kP = new TunableNumber("Tank_PID/KP", 0.4);
            public static final TunableNumber kI = new TunableNumber("Tank_PID/KI", 0);
            public static final TunableNumber kD = new TunableNumber("Tank_PID/KD", 0);
        }
    }

    public static class ShooterConstants {
        public static final TunableNumber shootVoltage = new TunableNumber("Shooter/ShootVoltage", 3);

        public static class ShooterPID {
            public static final TunableNumber kP = new TunableNumber("Shooter_PID/KP", 0.3);
            public static final TunableNumber kI = new TunableNumber("Shooter_PID/KI", 0);
            public static final TunableNumber kD = new TunableNumber("Shooter_PID/KD", 0);
            public static final TunableNumber kA = new TunableNumber("Shooter_PID/KA", 0);
            public static final TunableNumber kV = new TunableNumber("Shooter_PID/KV", 0);
            public static final TunableNumber kS = new TunableNumber("Shooter_PID/KS", 0);
        }
    }
}
