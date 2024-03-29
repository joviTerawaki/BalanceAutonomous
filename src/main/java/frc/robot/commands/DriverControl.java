package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConsts;
import frc.robot.subsystems.SwerveSubsystem;

public class DriverControl extends CommandBase{
    private SwerveSubsystem swerveSubsystem;
    private DoubleSupplier xSupplier, ySupplier, zSupplier;
    private BooleanSupplier fieldOriented;
    // private SlewRateLimiter xLimiter, yLimiter, zLimiter;

    public DriverControl(SwerveSubsystem subs, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier zSupplier,
        BooleanSupplier fieldOriented) {
            swerveSubsystem = subs;
            this.xSupplier = xSupplier;
            this.ySupplier = ySupplier;
            this.zSupplier = zSupplier;
            this.fieldOriented = fieldOriented;


            // this.xLimiter = new SlewRateLimiter(0.5);
            // this.yLimiter = new SlewRateLimiter(0.5);
            // this.zLimiter = new SlewRateLimiter(0.5);

            addRequirements(subs);
    }

    public double deadzone(double num){
        return Math.abs(num) > 0.01 ? num : 0;
    }

    private static double modifyAxis(double num) {
        // Square the axis
        num = Math.copySign(num * num, num);
    
        return num;
      }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Get values from joysticks
        double xSpeed = xSupplier.getAsDouble();
        double ySpeed = ySupplier.getAsDouble();
        double zSpeed = zSupplier.getAsDouble();

        // Deadzone
        xSpeed = deadzone(xSpeed);
        ySpeed = deadzone(ySpeed);
        zSpeed = deadzone(zSpeed);

        // Smoother acceleration
        xSpeed = modifyAxis(xSpeed); //xLimiter.calculate(xSpeed) * SwerveConsts.maxSpeed_mps;
        ySpeed = modifyAxis(ySpeed); //yLimiter.calculate(ySpeed) * SwerveConsts.maxSpeed_mps;
        zSpeed = modifyAxis(zSpeed); //zLimiter.calculate(zSpeed) * SwerveConsts.maxRotation;

        // Chassis Speeds
        ChassisSpeeds chassisSpeeds;
        if(fieldOriented.getAsBoolean()) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, swerveSubsystem.getRotation2d());
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, zSpeed);
        }

        // Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = SwerveConsts.driveKinematics.toSwerveModuleStates(chassisSpeeds);

        // Set each module state to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
