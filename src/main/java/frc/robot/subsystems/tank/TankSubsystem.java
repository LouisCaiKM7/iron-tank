// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tank;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotConstants.TankConstants.WHEEL_RADIUS;
import static frc.robot.RobotConstants.TankConstants.WHEEL_TRACK;

public class TankSubsystem extends SubsystemBase {

    private final TankIO io;
    private final TankIOInputsAutoLogged inputs = new TankIOInputsAutoLogged();

    DifferentialDriveKinematics kinematics =
            new DifferentialDriveKinematics(WHEEL_TRACK.in(Units.Meters));

    public TankSubsystem(TankIO io) {
        this.io = io;
    }

    /**
     * @param forwardSpeed: m/s
     * @param turningSpeed: rad/s
     */
    public void setArcadeSpeed(LinearVelocity forwardSpeed, AngularVelocity turningSpeed) {
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(forwardSpeed, MetersPerSecond.of(0), turningSpeed));

        SmartDashboard.putNumber("TankSubsystem/forwardSpeed", forwardSpeed.in(MetersPerSecond));
        SmartDashboard.putNumber("TankSubsystem/turningSpeed", forwardSpeed.in(MetersPerSecond));

        io.setRPS(
                RotationsPerSecond.of(wheelSpeeds.leftMetersPerSecond / WHEEL_RADIUS.in(Meters) * 2 * Math.PI),
                RotationsPerSecond.of(wheelSpeeds.rightMetersPerSecond / WHEEL_RADIUS.in(Meters) * 2 * Math.PI));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Tank", inputs);
    }
}
