// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

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

    // Run arcade drive based on setSpeeds
    public void setArcadeSpeed(double forwardSpeed, double turningSpeed) {
        double leftSpeed = forwardSpeed + turningSpeed;
        double rightSpeed = forwardSpeed - turningSpeed;

        setSpeeds(leftSpeed, rightSpeed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
