// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.MathUtils;
import frc.robot.Subsystems.SwerveDriveSubsystem;


public class SwerveDriveCommand extends CommandBase {
  /** Creates a new DrivetrainCommand. */
  private SwerveDriveSubsystem mDrivetrain;
  private XboxController mXbox;

  public SwerveDriveCommand(SwerveDriveSubsystem subsystem, XboxController xbox) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mDrivetrain = subsystem;
    this.mXbox = xbox;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
