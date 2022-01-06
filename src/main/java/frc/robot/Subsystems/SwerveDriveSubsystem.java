/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Represents a swerve drive style drivetrain.
 */
public class SwerveDriveSubsystem extends SubsystemBase {
  public static final double kMaxSpeed = 0.5; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  private static final double WHEELBASE = 18.625;
  private static final double TRACKWIDTH = 18.5;

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModuleCANCoder m_frontLeft = new SwerveModuleCANCoder(Constants.frontLeftDrive, Constants.frontLeftTurn, 0, Constants.frontLeftEncoder);
  private final SwerveModuleCANCoder m_frontRight = new SwerveModuleCANCoder(Constants.frontRightDrive, Constants.frontRightTurn, 0, Constants.frontRightEncoder);
  private final SwerveModuleCANCoder m_backLeft = new SwerveModuleCANCoder(Constants.backLeftDrive, Constants.backLeftTurn, 0, Constants.backLeftEncoder);
  private final SwerveModuleCANCoder m_backRight = new SwerveModuleCANCoder(Constants.backRightDrive, Constants.backRightTurn, 0, Constants.backRightEncoder);

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
  );

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getAngle());

  public SwerveDriveSubsystem() {
    m_gyro.reset();
  }

  /**
   * Returns the angle of the robot as a Rotation2d.
   *
   * @return The angle of the robot.
   */
  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rot, getAngle())
            : new ChassisSpeeds(xSpeed, ySpeed, rot)
    );
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);
    // m_frontLeft.setDesiredState(swerveModuleStates[0]);
    // m_frontRight.setDesiredState(swerveModuleStates[1]);
    // m_backLeft.setDesiredState(swerveModuleStates[2]);
    // m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void stopDriveMotors() {
    m_frontLeft.setDriveSpeed(0);
    m_frontRight.setDriveSpeed(0);
    m_backRight.setDriveSpeed(0);
    m_backLeft.setDriveSpeed(0);
  }

  public void holonomicDrive(double forward, double strafe, double rotation) {
    m_frontLeft.setDriveSpeed(forward);
    m_frontRight.setDriveSpeed(forward);
    m_backRight.setDriveSpeed(forward);
    m_backLeft.setDriveSpeed(forward);

    m_frontLeft.setTurnSpeed(rotation);
    m_frontRight.setTurnSpeed(rotation);
    m_backRight.setTurnSpeed(rotation);
    m_backLeft.setTurnSpeed(rotation);
  }
  
  public void m1holonomicDrive(double forward, double strafe, double rotation) {
    m_frontLeft.setDriveSpeed(forward);
    m_frontLeft.setTurnSpeed(rotation);
  }
  public void m2holonomicDrive(double forward, double strafe, double rotation) {
    m_frontRight.setDriveSpeed(forward);
    m_frontRight.setTurnSpeed(rotation);
  }

  public void m2Align(double angle){
    m_frontRight.goToPosition(angle);
  } 

   public void m3holonomicDrive(double forward, double strafe, double rotation) {
    m_backRight.setDriveSpeed(forward);
    m_backRight.setTurnSpeed(rotation);
  }
  public void m4holonomicDrive(double forward, double strafe, double rotation) {
    m_backLeft.setDriveSpeed(forward);
    m_frontLeft.setTurnSpeed(rotation);
  }

  public void setStrafe(double forward, double strafe, double rotation) {
        
    double a = strafe - rotation * (WHEELBASE / TRACKWIDTH);
    double b = strafe + rotation * (WHEELBASE / TRACKWIDTH);
    double c = forward - rotation * (TRACKWIDTH / WHEELBASE);
    double d = forward + rotation * (TRACKWIDTH / WHEELBASE);

    double[] angles = new double[]{
      Math.atan2(b, c) * 180 / Math.PI,
      Math.atan2(b, d) * 180 / Math.PI,
      Math.atan2(a, d) * 180 / Math.PI,
      Math.atan2(a, c) * 180 / Math.PI
    };

    double[] speeds = new double[]{
      Math.sqrt(b * b + c * c),
      Math.sqrt(b * b + d * d),
      Math.sqrt(a * a + d * d),
      Math.sqrt(a * a + c * c)
    };

    // SmartDashboard.putNumber("Module 0 Ticks", mSwerveModules[0].getPosition());
    // SmartDashboard.putNumber("Module 1 Ticks", mSwerveModules[1].getPosition());
    // SmartDashboard.putNumber("Module 2 Ticks", mSwerveModules[2].getPosition());
    // SmartDashboard.putNumber("Module 3 Ticks", mSwerveModules[3].getPosition());
    // SmartDashboard.putNumber("Module 3 get current angle", mSwerveModules[3].getCurrentAngle());


    SwerveModuleCANCoder[] modules = { m_frontLeft, m_frontRight, m_backRight, m_backLeft};
    for (int i = 0; i < 4; i++) {
      if (Math.abs(forward) > 0.05 ||
        Math.abs(strafe) > 0.05 ||
        Math.abs(rotation) > 0.05) {
        modules[i].setTargetAngle(angles[i] + 180);
      } 
      else {
        modules[i].setTargetAngle(modules[i].getTargetAngle());
      }
        modules[i].setDriveSpeed(speeds[i]);
    }
 }

  public void alignWheels() {
    m_frontLeft.goToPosition(m_frontLeft.getOffsetAngle());
    m_frontRight.goToPosition(m_frontRight.getOffsetAngle());
    m_backRight.goToPosition(m_backRight.getOffsetAngle());
    m_backLeft.goToPosition(m_backLeft.getOffsetAngle());
  }

  /**
   * Updates the field relative position of the robot.
   */
  public void updateOdometry() {
    m_odometry.update(
        getAngle() //,
        // m_frontLeft.getState(),
        // m_frontRight.getState(),
        // m_backLeft.getState(),
        // m_backRight.getState()
    );
  }
}
