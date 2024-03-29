// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.SPI;

public class Robot extends TimedRobot {
  public static Drivetrain m_swerve;
  public static Gyro m_gyro;

  private static XboxController swerveController;
  private static XboxController alternateController;
  
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private static SlewRateLimiter m_xspeedLimiter;
  private static SlewRateLimiter m_yspeedLimiter;
  private static SlewRateLimiter m_rotLimiter;

  private boolean teleopStarted;

  @Override
  public void robotInit() {
    m_gyro = new Gyro(90);
    m_swerve = new Drivetrain();

    swerveController = new XboxController(Constants.swerveControllerPort);
    alternateController = new XboxController(Constants.alternateControllerPort);

    m_xspeedLimiter = new SlewRateLimiter(Constants.xSlewRateLimiter);
    m_yspeedLimiter = new SlewRateLimiter(Constants.ySlewRateLimiter);
    m_rotLimiter = new SlewRateLimiter(Constants.yawSlewRateLimiter);

    teleopStarted = false;
  }

  @Override
  public void autonomousInit() {
    teleopStarted = false;
  }

  @Override
  public void autonomousPeriodic() {
    m_swerve.updateOdometry();
  }
  
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();
    teleopStarted = false;

  }

  boolean fieldRelative = false;
  @Override
  public void teleopPeriodic() {
    if (swerveController.getBackButtonPressed() == true) {
      fieldRelative = !fieldRelative;
    }
    if (swerveController.getStartButton() == true) {
      teleopStarted = true;
    }
    if (teleopStarted == true) {
      driveWithJoystick(fieldRelative);
    }
    if (swerveController.getAButtonPressed() == true) {
      m_gyro.calibrateGyro();
    }
    SmartDashboard.putBoolean("Field Relative", fieldRelative);
  }

  public void disabledInit() {
    teleopStarted = false;
  }

  private void driveWithJoystick(boolean fieldRelative) {
    double xSpeed = 0;
    double ySpeed = 0;
    double yaw = 0;
    boolean aboveDeadband = false;
 
    /* Converts the cartesian X and Y axes of the left stick of the swerve controller to the polar coordinate "r" */
    /* This is the distance from (0, 0) to the cartesian coordinate of the left stick output */
    /* If this absolute "r" value is less than the left stick deadband (set in Constants), the xSpeed and ySpeed are not calculated */
    if (Math.abs(Math.sqrt(Math.pow(swerveController.getLeftX(), 2) + Math.pow(swerveController.getLeftY(), 2))) > Constants.swerveControllerLeftStickDeadband) {
      aboveDeadband = true;
      if (fieldRelative == true) {
        ySpeed = -1 * (m_yspeedLimiter.calculate(swerveController.getLeftY() * Math.cos(Math.toRadians(m_gyro.getTotalAngleDegrees())) - (-1 * swerveController.getLeftX()) * Math.sin(Math.toRadians(m_gyro.getTotalAngleDegrees()))) * Drivetrain.kMaxVoltage);
        xSpeed = m_xspeedLimiter.calculate(swerveController.getLeftY() * Math.sin(Math.toRadians(m_gyro.getTotalAngleDegrees())) + (-1 * swerveController.getLeftX()) * Math.cos(Math.toRadians(m_gyro.getTotalAngleDegrees()))) * Drivetrain.kMaxVoltage;
      }
      else {
        ySpeed = -1 * m_yspeedLimiter.calculate(swerveController.getLeftX()) * Drivetrain.kMaxVoltage;
        xSpeed = m_xspeedLimiter.calculate(swerveController.getLeftY()) * Drivetrain.kMaxVoltage;
      }
    }
    else {
      if (aboveDeadband == true) {
        aboveDeadband = false;
        ySpeed = 0.1;
      }
      ySpeed = 0;
      xSpeed = 0;
    }
    yaw = -1 * m_rotLimiter.calculate(MathUtil.applyDeadband(swerveController.getRightX(), Constants.swerveControllerRightXDeadband)) * Drivetrain.kMaxAngularSpeed;

    SmartDashboard.putNumber("xSpeed ", xSpeed);
    SmartDashboard.putNumber("ySpeed ", ySpeed);
    SmartDashboard.putNumber("yaw ", yaw);
    SmartDashboard.putNumber("gyro angle ", m_gyro.getTotalAngleDegrees());

    m_swerve.drive(xSpeed, ySpeed, yaw);
  }
}