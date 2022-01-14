package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;




public class SwerveModuleCANCoder extends SubsystemBase{
    private static final double kWheelRadius = 0.0508;
    private static final int kEncoderResolution = 4096;


    private static final double kModuleMaxAngularVelocity = SwerveDriveSubsystem.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration
        = 2 * Math.PI; // radians per second squared
    private CANCoder m_Encoder;
    private TalonFX m_drive;
    private TalonFX m_turn;
    private TalonFXConfiguration config = new TalonFXConfiguration();
    private double mZeroOffSet;
    private double mLastTargetAngle = 0;

    //private final PIDController m_drivePIDController = new PIDController(1, 3, 5, 7);

    // private final ProfiledPIDController m_turningPIDController
    // = new ProfiledPIDController(1, 0, 0,
    // new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));
    

     public SwerveModuleCANCoder(int driveMotor, int turnMotor, double zeroOffSet, int encoderDeviceNum) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        // config.remoteFilter0.remoteSensorDeviceID = _canifier.getDeviceID();
        // config.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANifier_PWMInput1;
        // config.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        
        m_Encoder = new CANCoder(encoderDeviceNum);
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
        
        mZeroOffSet = zeroOffSet;

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

     public double getOffsetAngle() {
         return mZeroOffSet;
     }
    
     public void setDriveSpeed(double speed) {
         m_drive.set(ControlMode.PercentOutput, speed);
     }

     public double getTargetAngle() {
         return mLastTargetAngle;
     }

     public void setTargetAngle(double targetAngle) {
         mLastTargetAngle = targetAngle;
         targetAngle %= 360;
         final double currentAngle = m_Encoder.getPosition() * (360.0 / 1024.0);
    
         double currentAngleMod = currentAngle % 360;
         double delta = currentAngleMod - targetAngle;
        if (delta > 180) {
            targetAngle += 360;
        }
        else if (delta < -180) {
            targetAngle -= 360;
        }
        delta = currentAngleMod - targetAngle;
        if (delta > 90 || delta < -90) {
            if (delta > 90) { 
                targetAngle += 180;
            }
            else if (delta < -90); {
                targetAngle -= 180;
            }
            m_drive.setInverted(true);
        }
        targetAngle += currentAngle - currentAngleMod;
        targetAngle *= 1024.0 / 360.0;
      //  System.out.println("target angle: " + targetAngle);
        m_turn.set(ControlMode.Position, targetAngle);
        }

     public void setTurnSpeed(double speed) {
         m_turn.set(ControlMode.PercentOutput, speed);
         
     }

     public void goToPosition(double position) {
         m_turn.set(ControlMode.Position, position);
     }
    //  public void setPIDSlot(final int slot) {
    //      m_drive.selectProfileSlot(slot, 0);
    //  }
    //  public void swapPIDSlot(int slot){
    //     for(int i = 0; i < 4; i++)
    //        {
    //         [i].setPIDSlot(slot);
    //         mSwerveModules[i].setPIDSlot(slot);
    //         mSwerveModules[i].setPIDSlot(slot);
    //         mSwerveModules[i].setPIDSlot(slot);
    //        }
    // }
     public void setDrivePIDSlot(final int slot){
         m_turn.selectProfileSlot(slot, 0);
     }
     
    //  public void swapDrivePIDSlot(int slot){
    //      for(int i = 0; i < 4; i++)
    //      {
    //          mSwerveModules[i].setDrivePIDSlot(slot);
    //      }
    //  }
}