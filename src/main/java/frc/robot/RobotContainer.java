package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriverControl;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public final XboxController m_Controller = new XboxController(0);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new DriverControl(swerveSubsystem,
      () -> -m_Controller.getLeftY(), 
      () -> -m_Controller.getLeftX(), 
      () -> -m_Controller.getRightX(), 
      () -> m_Controller.getLeftBumper()));
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(m_Controller, 1).onTrue(new InstantCommand(() -> swerveSubsystem.resetNavx()));
    
  }


  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
