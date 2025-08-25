// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tank;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotConstants.TankConstants.*;

public class TankSubsystem extends SubsystemBase {

    private final TankIO io;
    private final TankIOInputsAutoLogged inputs = new TankIOInputsAutoLogged();
    public PigeonIMU pigeonIMU = new PigeonIMU(3);
    public DifferentialDriveOdometry tankOdometry =
            new DifferentialDriveOdometry(new Rotation2d(), 0, 0, new Pose2d());
    Pose2d robotPose = new Pose2d();
    // 添加基于 RPS 的位置计算变量
    private double lastLeftRPS = 0.0;
    private double lastRightRPS = 0.0;
    private double currentX = 0.0;
    private double currentY = 0.0;
    private Angle currentAngle = Degrees.of(0);
    private double lastTime = 0.0;
    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(WHEEL_TRACK);

    public TankSubsystem(TankIO io) {
        this.io = io;
    }

    /**
     * @param forwardSpeed: m/s
     * @param turningSpeed: rad/s
     */
    public void setArcadeSpeed(LinearVelocity forwardSpeed, AngularVelocity turningSpeed) {
        DifferentialDriveWheelSpeeds wheelSpeeds =
                kinematics.toWheelSpeeds(
                        new ChassisSpeeds(forwardSpeed, MetersPerSecond.of(0), turningSpeed));

        SmartDashboard.putNumber("TankSubsystem/forwardSpeed", forwardSpeed.in(MetersPerSecond));
        SmartDashboard.putNumber("TankSubsystem/turningSpeed", turningSpeed.in(RadiansPerSecond));

        io.setRPS(chassisSpeedToMotorRPS(wheelSpeeds.leftMetersPerSecond), chassisSpeedToMotorRPS(wheelSpeeds.rightMetersPerSecond));
    }

    public void resetGyro() {
        pigeonIMU.setYaw(0);
    }

    public void resetOdometry() {
        tankOdometry.resetPose(new Pose2d());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Tank", inputs);
        updatePoseFromRPS();

        Logger.recordOutput("Tank/RobotPose", robotPose);
        Logger.recordOutput("Tank/Yaw", pigeonIMU.getYaw());
        Logger.recordOutput("Tank/measuredDistance", getRobotPose().getY());
    }

    private AngularVelocity chassisSpeedToMotorRPS(double chassisSpeedMetersPerSecond) {
        return RotationsPerSecond.of(chassisSpeedMetersPerSecond / (WHEEL_RADIUS.in(Meters) * 2 * Math.PI)).times(GEAR_RATIO);
    }

    public Pose2d getRobotPose() {
        return robotPose;
    }

    private void updatePoseFromRPS() {
        double currentTime = System.currentTimeMillis() / 1000.0; // 转换为秒
        double deltaTime = currentTime - lastTime;

        if (lastTime == 0.0) {
            lastTime = currentTime;
            lastLeftRPS = inputs.leftMotorVelocityRotPerSec;
            lastRightRPS = inputs.rightMotorVelocityRotPerSec;
            return;
        }


        // 计算左右轮的平均 RPS
        double avgLeftRPS = (inputs.leftMotorVelocityRotPerSec + lastLeftRPS) / 2.0;
        double avgRightRPS = (inputs.rightMotorVelocityRotPerSec + lastRightRPS) / 2.0;


        double leftDistance = -inputs.leftPosition / GEAR_RATIO * (WHEEL_RADIUS.in(Meters) * 2 * Math.PI);
        double rightDistance = inputs.rightPosition / GEAR_RATIO * (WHEEL_RADIUS.in(Meters) * 2 * Math.PI);

        currentAngle = Degrees.of(pigeonIMU.getYaw());

        robotPose = tankOdometry.update(new Rotation2d(currentAngle), leftDistance, rightDistance);

        SmartDashboard.putNumber("TankSubsystem/leftDistance", leftDistance);
        SmartDashboard.putNumber("TankSubsystem/rightDistance", rightDistance); // 保存当前值作为下次计算的 last 值
        lastLeftRPS = inputs.leftMotorVelocityRotPerSec;
        lastRightRPS = inputs.rightMotorVelocityRotPerSec;
        lastTime = currentTime;
    }

}