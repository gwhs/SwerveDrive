package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Container {
    // private final XboxController mXboxController;

    public Container() {
        SwerveModule m0 = new SwerveModule(1,2);
        SwerveModule m1 = new SwerveModule(3,4);
        SwerveModule m2 = new SwerveModule(5,6);
        SwerveModule m3 = new SwerveModule(7,8);
    }
    private void configureButtonBindings() {
        // JoystickButton getX = new JoystickButton(xbox, XboxController.Jo.kY.value);
        // JoystickButton getY = new JoystickButton(xbox, XboxController.Button.kA.value);
    
        //buttonY.toggleWhenPressed(new firstGearCommand(m_PneumaticSubsystem));
       // JoystickButton buttonA = new JoystickButton(xbox, XboxController.Button.kA.value);
        //buttonA.whenPressed(new secondGearCommand(m_PneumaticSubsystem, xbox));
    
      }
}