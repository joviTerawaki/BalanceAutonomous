package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

public class TestingAutoCommandGroup extends CommandBase {
  private final SwerveSubsystem swerve; 

  public TestingAutoCommandGroup(SwerveSubsystem newSwerve) {
    swerve = newSwerve; 

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    swerve.resetEnc();
    swerve.resetNavx();
  }

  @Override
  public void execute() {
    new SequentialCommandGroup(
      new DriveForwardRoll(swerve),
      new PIDBalanceCommand(swerve)
    );
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
