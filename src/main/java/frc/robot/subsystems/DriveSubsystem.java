// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.RobotConstants.TankPID;

public class DriveSubsystem extends SubsystemBase {
    /**
     * Creates motors
     */
    TalonFX motorRight = new TalonFX(1, "rio");
    TalonFX motorLeft = new TalonFX(0, "rio");

    TalonFXConfigurator motorRightConfigurator = motorRight.getConfigurator();
    TalonFXConfigurator motorLeftConfigurator = motorLeft.getConfigurator();

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        motorLeftConfigurator.apply(motorOutputConfigs);
        motorRightConfigurator.apply(motorOutputConfigs);
    }

    /**
     * Run motor according to joystick input values.
     */
    public void setSpeeds(double leftSpeeds, double rightSpeeds) {
        motorLeft.set(leftSpeeds);
        motorRight.set(-rightSpeeds);
    }

    public void setRPS(double leftRPS, double rightRPS) {
        SmartDashboard.putNumber("DriveSubSystem/leftRPS", leftRPS);
        SmartDashboard.putNumber("DriveSubSystem/rightRPS", -rightRPS);

        motorLeft.setControl(new VelocityVoltage(leftRPS));
        motorRight.setControl(new VelocityVoltage(-rightRPS));
    }

    /**
     * @param forwardPercentage: m/s
     * @param turningDifference: rad/s
     */

    public void setArcadeSpeed(double forwardPercentage, double turningDifference) {
        double leftDutycycle = forwardPercentage - turningDifference;
        double rightDutycycle = forwardPercentage + turningDifference;

        SmartDashboard.putNumber("DriveSubSystem/leftRPS", forwardPercentage);
        SmartDashboard.putNumber("DriveSubSystem/rightRPS", turningDifference);

        setRPS(leftDutycycle * 4, rightDutycycle * 4);
    }

    @Override
    public void periodic() {
        motorLeft.getConfigurator().apply(new Slot0Configs()
                .withKP(TankPID.kP.get())
                .withKI(TankPID.kI.get())
                .withKD(TankPID.kD.get()));

        motorRight.getConfigurator().apply(new Slot0Configs()
                .withKP(TankPID.kP.get())
                .withKI(TankPID.kI.get())
                .withKD(TankPID.kD.get()));
        // This method will be called once per scheduler run
    }
}