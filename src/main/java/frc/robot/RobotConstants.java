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
        public static AngularVelocity MAX_ANGULAR_SPEED = RadiansPerSecond.of(Math.PI);
        public static double GEAR_RATIO = 12.0;
        public static Distance WHEEL_TRACK = Meters.of(0.548005);
        public static Distance WHEEL_RADIUS = Inches.of(3);

        public static class TankPID {
            public static final TunableNumber kP = new TunableNumber("Tank_PID/KP", 0.1);
            public static final TunableNumber kI = new TunableNumber("Tank_PID/KI", 0);
            public static final TunableNumber kD = new TunableNumber("Tank_PID/KD", 0);
        }
    }

    public static class ShooterConstants {
        public static final TunableNumber shootVoltage = new TunableNumber("Shooter/ShootVoltage", 6);

        public static class ShooterPID {
            public static final TunableNumber kP = new TunableNumber("Shooter_PID/KP", 10);
            public static final TunableNumber kI = new TunableNumber("Shooter_PID/KI", 0);
            public static final TunableNumber kD = new TunableNumber("Shooter_PID/KD", 0);
            public static final TunableNumber kA = new TunableNumber("Shooter_PID/KA", 0);
            public static final TunableNumber kV = new TunableNumber("Shooter_PID/KV", 0);
            public static final TunableNumber kS = new TunableNumber("Shooter_PID/KS", 0);
        }
    }

    public static class IntakeConstants {
        public static final int PIVOT_MOTOR_ID = 0;
        public static final int ROLLER_MOTOR_ID = 1;
        public static final int CAN_CODER_ID = 8;
        public static final TunableNumber INTAKE_POSITION_DEGREES = new TunableNumber("Intake/INTAKE_POSITION_DEGREES", 0);
        public static final TunableNumber SHOOT_POSITION_DEGREES = new TunableNumber("Intake/SHOOT_POSITION_DEGREES", 30);
        public static final TunableNumber ELEVATE_POSITION_DEGREES = new TunableNumber("Intake/ELEVATE_POSITION_DEGREES", 45);
        public static final TunableNumber INTAKE_VOLTAGE = new TunableNumber("Intake/INTAKE_VOLTAGE", 6);
        public static final TunableNumber SHOOT_VOLTAGE = new TunableNumber("Intake/SHOOT_VOLTAGE", -6);
        public static final TunableNumber HOLD_VOLTAGE = new TunableNumber("Intake/HOLD_VOLTAGE", 2);


        public static class IntakePivotPID {
            public static final TunableNumber kP = new TunableNumber("Intake_PID/Pivot/KP", 1);
            public static final TunableNumber kI = new TunableNumber("Intake_PID/Pivot/KI", 0);
            public static final TunableNumber kD = new TunableNumber("Intake_PID/Pivot/KD", 0);
            public static final TunableNumber kA = new TunableNumber("Intake_PID/Pivot/KA", 0);
            public static final TunableNumber kV = new TunableNumber("Intake_PID/Pivot/KV", 0);
            public static final TunableNumber kS = new TunableNumber("Intake_PID/Pivot/KS", 0);
            public static final TunableNumber kG = new TunableNumber("Intake_PID/Pivot/KG", 0);
        }

        public static class IntakeRollerPID {
            public static final TunableNumber kP = new TunableNumber("Intake_PID/Roller/KP", 1);
            public static final TunableNumber kI = new TunableNumber("Intake_PID/Roller/KI", 0);
            public static final TunableNumber kD = new TunableNumber("Intake_PID/Roller/KD", 0);
            public static final TunableNumber kA = new TunableNumber("Intake_PID/Roller/KA", 0);
            public static final TunableNumber kV = new TunableNumber("Intake_PID/Roller/KV", 0);
            public static final TunableNumber kS = new TunableNumber("Intake_PID/Roller/KS", 0);
        }
    }

}