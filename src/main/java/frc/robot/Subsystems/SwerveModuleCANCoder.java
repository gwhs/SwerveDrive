package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PWMTalonFX;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;




public class SwerveModuleCANCoder extends SubsystemBase{
    private static final double kWheelRadius = 0.0508;
    private static final int kEncoderResolution = 4096;

    private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration
        = 2 * Math.PI; // radians per second squared
    private final CANCoder m1_Encoder = new CANCoder(Constants.frontLeftEncoder);
    private final CANCoder m2_Encoder = new CANCoder(Constants.frontRightEncoder);
    private final CANCoder m3_Encoder = new CANCoder(Constants.backRightEncoder);
    private final CANCoder m4_Encoder = new CANCoder(Constants.backLeftEncoder);
    private TalonFX m_drive;
    private TalonFX m_turn;
    private TalonFXConfiguration config = new TalonFXConfiguration();


    //private final PIDController m_drivePIDController = new PIDController(1, 3, 5, 7);

    // private final ProfiledPIDController m_turningPIDController
    // = new ProfiledPIDController(1, 0, 0,
    // new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));
    

     public SwerveModuleCANCoder(int driveMotor, int turnMotor) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        // config.remoteFilter0.remoteSensorDeviceID = _canifier.getDeviceID();
        // config.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANifier_PWMInput1;
        // config.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;

         m_drive = new TalonFX(driveMotor);
         m_turn = new TalonFX(turnMotor);
        m_drive.configFactoryDefault();
        m_drive.setInverted(true);
        m_drive.setSensorPhase(true);
        m_drive.configAllSettings(config);

        m_turn.configFactoryDefault();
        m_turn.setInverted(true);
        m_turn.setSensorPhase(true);
        m_turn.configAllSettings(config);

//     //     m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);
//     //     m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
//      }
    //  public SwerveModuleState getState() {
    //     return new SwerveModuleState(m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.get()));
    //  }
//     public void setDesiredState(SwerveModuleState state) {
//        // Calculate the drive output from the drive PID controller.
//        final var driveOutput = m_drivePIDController.calculate(
//         m_driveEncoder.getRate(), state.speedMetersPerSecond);
    
//     //     // Calculate the turning motor output from the turning PID controller.
//     //    final var turnOutput = m_turningPIDController.calculate(
//     //    m_turningEncoder.get(), state.angle.getRadians()
//     //  );
//     }
// }
     }
    
     public void setDriveSpeed(double speed) {
         m_drive.set(ControlMode.PercentOutput, speed);
     }
     public void setPIDSlot(final int slot) {
         m_drive.selectProfileSlot(slot, 0);
     }
     public void setDrivePIDSlot(final int slot){
         m_turn.selectProfileSlot(slot, 0);
     }
}