// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class DrivetrainCommand extends CommandBase {
  /** Creates a new DrivetrainCommand. */
  private Drivetrain m_subsystem;
  private XboxController m_xbox;
  public DrivetrainCommand(Drivetrain subsystem, XboxController xbox) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
		this.m_xbox = xbox;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double strafe = mXboxController.getX(Hand.kLeft); //real: pos
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrivetrain.stopDriveMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
