package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;



public class SwerveModuleCANCoder{
    private static final double kWheelRadius = 0.0508;
    private static final int kEncoderResolution = 4096;

    private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration
        = 2 * Math.PI; // radians per second squared
    private final CANEncoder m1_Encoder = new CANEncoder(Constants.frontLeftEncoder);
    private final CANEncoder m2_Encoder = new CANEncoder(Constants.frontRightEncoder);
    private final CANEncoder m3_Encoder = new CANEncoder(Constants.backRightEncoder);
    private final CANEncoder m4_Encoder = new CANEncoder(Constants.backLeftEncoder);

    //private final PIDController m_drivePIDController = new PIDController(1, 3, 5, 7);

    // private final ProfiledPIDController m_turningPIDController
    // = new ProfiledPIDController(1, 0, 0,
    // new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));
    

    // public SwerveModuleCANCoder(int driveMotor, int turnMotor) {

    //     m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);
    //     m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    // }
    // public SwerveModuleState getState() {
    //     return new SwerveModuleState(m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.get()));
    //   }
    // public void setDesiredState(SwerveModuleState state) {
    //     // Calculate the drive output from the drive PID controller.
    //     final var driveOutput = m_drivePIDController.calculate(
    //         m_driveEncoder.getRate(), state.speedMetersPerSecond);
    
    //     // Calculate the turning motor output from the turning PID controller.
    //     final var turnOutput = m_turningPIDController.calculate(
    //         m_turningEncoder.get(), state.angle.getRadians()
    //     );
}