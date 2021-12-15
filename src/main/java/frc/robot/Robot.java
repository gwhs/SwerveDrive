/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final SwerveDriveSubsystem m_swerve = new SwerveDriveSubsystem();

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    m_swerve.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
  }

  public void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    double xSpeed = -m_controller.getY(GenericHID.Hand.kLeft) * SwerveDriveSubsystem.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    double ySpeed = -m_controller.getX(GenericHID.Hand.kLeft) * SwerveDriveSubsystem.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    double rot = -m_controller.getX(GenericHID.Hand.kRight) * SwerveDriveSubsystem.kMaxAngularSpeed;


		xSpeed = MathUtils.deadband(xSpeed, 0.175);
		ySpeed = MathUtils.deadband(ySpeed, 0.175);
		rot = MathUtils.deadband(rot, 0.1);
    
    m_swerve.holonomicDrive(xSpeed, ySpeed, rot);
  }
  @Override
  public void robotInit() {

  }
  @Override
  public void teleopInit(){

  }
  @Override
  public void disabledInit(){

  }
}
