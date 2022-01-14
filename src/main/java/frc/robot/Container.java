package frc.robot;

//import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.SwerveDriveCommand;
import frc.robot.Subsystems.SwerveDriveSubsystem;
import frc.robot.Robot;

public class Container {
    // private final XboxController mXboxController;
    private final XboxController mxbox;
    private final SwerveDriveSubsystem msubsystem;
    public Container() {
        msubsystem = new SwerveDriveSubsystem();
        mxbox = new XboxController(0);
        msubsystem.setDefaultCommand(new SwerveDriveCommand(msubsystem, mxbox));
    }
    private void configureButtonBindings() {
        //JoystickButton getX = new JoystickButton(xbox, XboxController.Jo.kY.value);
        // JoystickButton getY = new JoystickButton(xbox, XboxController.Button.kA.value);
    
        //buttonY.toggleWhenPressed(new firstGearCommand(m_PneumaticSubsystem));
       // JoystickButton buttonA = new JoystickButton(xbox, XboxController.Button.kA.value);
        //buttonA.whenPressed(new secondGearCommand(m_PneumaticSubsystem, xbox));
    
      }
}

