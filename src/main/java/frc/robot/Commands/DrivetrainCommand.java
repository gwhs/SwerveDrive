// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.MathUtils;

public class DrivetrainCommand extends CommandBase {
  /** Creates a new DrivetrainCommand. */
  private Drivetrain mDrivetrain;
  private XboxController mXbox;
  public DrivetrainCommand(Drivetrain subsystem, XboxController xbox) {
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
    double forward = mXbox.getY(Hand.kLeft); //real: positive
		double rotation = mXbox.getTriggerAxis(Hand.kLeft) 
			- mXbox.getTriggerAxis(Hand.kRight); //trigger values are between 0 and 1, left is -1 and right is +1
		double strafe = mXbox.getX(Hand.kLeft); //real: pos

		forward = MathUtils.deadband(forward, 0.175);
		strafe = MathUtils.deadband(strafe, 0.175);
		rotation = MathUtils.deadband(rotation, 0.1);

		mDrivetrain.holonomicDrive(forward, -strafe, rotation);
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
