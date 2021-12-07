package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Container {
    private final XboxController mXboxController;

    public Container() {
        SwerveModule m0 = new SwerveModule();
        SwerveModule m1 = new SwerveModule();
        SwerveModule m2 = new SwerveModule();
        SwerveModule m3 = new SwerveModule();
    }
}
