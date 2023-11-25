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
    m_swerve = new Drivetrain();
    m_gyro = new Gyro(90);

    swerveController = new XboxController(Constants.swerveControllerPort);
    alternateController = new XboxController(Constants.alternateControllerPort);

    m_xspeedLimiter = new SlewRateLimiter(Constants.xSlewRateLimiter);
    m_yspeedLimiter = new SlewRateLimiter(Constants.ySlewRateLimiter);
    m_rotLimiter = new SlewRateLimiter(Constants.yawSlewRateLimiter);

    m_gyro.calibrateGyro();

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

  @Override
  public void teleopPeriodic() {
    if (swerveController.getBackButton() == true) {
      m_swerve.resetModules();
    }
    if (swerveController.getStartButton() == true) {
      teleopStarted = true;
    }
    if (teleopStarted == true) {
      driveWithJoystick(false);
    }
  }

  public void disabledInit() {
    teleopStarted = false;
  }


  /* DO NOT ATTEMPT TO CHANGE SWERVE CODE; INVERTING A SINGLE VALUE SCREWS WITH EVERYTHING (IDK HOW) */
  /* Wanna change the x and y axes? Or invert the one of the drive directions? Good luck!!! (pain) */
  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward. 
    final var xSpeed =
        m_xspeedLimiter.calculate(MathUtil.applyDeadband(-swerveController.getLeftY(), Constants.controllerLeftXDeadband))
            * Drivetrain.kMaxVoltage;
            SmartDashboard.putNumber("xSpeed ", xSpeed);

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default. 
    final var ySpeed =
        m_yspeedLimiter.calculate(MathUtil.applyDeadband(swerveController.getLeftX(), Constants.controllerLeftYDeadband))
            * Drivetrain.kMaxVoltage;
            SmartDashboard.putNumber("ySpeed ", ySpeed);

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var yaw =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(swerveController.getRightX(), Constants.controllerRightXDeadband))
            * Drivetrain.kMaxAngularSpeed;
            SmartDashboard.putNumber("yaw ", yaw);

    m_swerve.drive(-xSpeed, -ySpeed, yaw, fieldRelative);
  }
}