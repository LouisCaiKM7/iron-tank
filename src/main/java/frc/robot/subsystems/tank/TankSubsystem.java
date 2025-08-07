// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tank;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotConstants.TankConstants.*;

public class TankSubsystem extends SubsystemBase {

    private final TankIO io;
    private final TankIOInputsAutoLogged inputs = new TankIOInputsAutoLogged();

    public TankSubsystem(TankIO io) {
        this.io = io;
    }

    /**
     * @param forwardSpeed: m/s
     * @param turningSpeed: rad/s
     */

    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(WHEEL_TRACK);

    public void setArcadeSpeed(LinearVelocity forwardSpeed, AngularVelocity turningSpeed) {
        DifferentialDriveWheelSpeeds wheelSpeeds =
                kinematics.toWheelSpeeds(
                    new ChassisSpeeds(forwardSpeed, MetersPerSecond.of(0),turningSpeed));

//        AngularVelocity forwardRPS = RotationsPerSecond.of(forwardSpeed.div(WHEEL_RADIUS));

        Logger.recordOutput("TankSubsystem/targetForwardSpeed", forwardSpeed.in(MetersPerSecond));
        Logger.recordOutput("TankSubsystem/targetTurningSpeed", turningSpeed.in(RadiansPerSecond));

        io.setRPS(chassisSpeedToMotorRPS(wheelSpeeds.leftMetersPerSecond), chassisSpeedToMotorRPS(wheelSpeeds.rightMetersPerSecond));
    }

    public AngularVelocity chassisSpeedToMotorRPS(double chassisSpeedMetersPerSecond){
        return RotationsPerSecond.of(chassisSpeedMetersPerSecond/(WHEEL_RADIUS.in(Meters))* 2* Math.PI).times(GEAR_RATIO);
    }

//    public double forwardRPS = forwardSpeed / 2 / Math.PI / WHEEL_RADIUS * GEAR_RATIO;
//    public double turningRPS = turningSpeed * WHEEL_TRACK / 2 * GEAR_RATIO;

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("TANK",inputs);
    }
}
