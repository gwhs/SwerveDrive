// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.MathUtils;
import frc.robot.subsystems.SwerveDriveSubsystem;


public class AdjustedFieldOrientedCommand extends CommandBase {
  /** Creates a new DrivetrainCommand. */
  private SwerveDriveSubsystem mDrivetrain;
  private boolean mIncreaseAngle;
  
    public AdjustedFieldOrientedCommand(SwerveDriveSubsystem drivetrain, boolean increaseAngle) {
      mDrivetrain = drivetrain;
      mIncreaseAngle = increaseAngle;
    }
  
    @Override
    public void execute() {
      // if (mIncreaseAngle) {
      //   mDrivetrain.setAdjustmentAngle(mDrivetrain.getAdjustmentAngle()); // + ADJUSTEMENT_AMOUNT
      // } else {
      //   mDrivetrain.setAdjustmentAngle(mDrivetrain.getAdjustmentAngle()); // - ADJUSTEMENT_AMOUNT
      // }
    }
  
    @Override
    public boolean isFinished() {
      return true;
    }
}
