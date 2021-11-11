/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import com.ctre.motorcontrol.can.WPI_TalonFX;




public final class Container {
  private Container() {

    SwerveModule m0 = new SwerveModule(new TalonFX(), TalonFX());
    SwerveModule m1 = new SwerveModule(new TalonFX(), TalonFX());
    SwerveModule m2 = new SwerveModule(new TalonFX(), TalonFX());
    SwerveModule m3 = new SwerveModule(new TalonFX(), TalonFX());
   
  }

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
