package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConsts;

public class SwerveSubsystem extends SubsystemBase {
    private SwerveModule frontLeft;
    private SwerveModule backLeft;
    private SwerveModule backRight;
    private SwerveModule frontRight;

    private AHRS navx;


    public SwerveSubsystem() {
        frontLeft = new SwerveModule(SwerveConsts.FL_turningMotorPort, SwerveConsts.FL_driveMotorPort, 
            SwerveConsts.FL_absoluteEncoderPort, SwerveConsts.FL_offset , false, true);
        
        backLeft = new SwerveModule(SwerveConsts.BL_turningMotorPort, SwerveConsts.BL_driveMotorPort, 
            SwerveConsts.BL_absoluteEncoderPort, SwerveConsts.BL_offset , false, true);

        backRight = new SwerveModule(SwerveConsts.BR_turningMotorPort, SwerveConsts.BR_driveMotorPort, 
            SwerveConsts.BR_absoluteEncoderPort, SwerveConsts.BR_offset , false, true);

        frontRight = new SwerveModule(SwerveConsts.FR_turningMotorPort, SwerveConsts.FR_driveMotorPort, 
            SwerveConsts.FR_absoluteEncoderPort, SwerveConsts.FR_offset , false, true);

        navx = new AHRS(SPI.Port.kMXP);;
    }

    public void resetNavx() {
        navx.zeroYaw();
    }

    public double getYawAngle(){
        return ( /*navx.getYaw()*/ navx.getAngle() % 360 );
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getYawAngle());
    }

    public void stopModules() {
        frontLeft.stop();
        backLeft.stop();
        backRight.stop();
        frontRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConsts.maxSpeed_mps);
        frontLeft.setDesiredState(desiredStates[0]);
        backLeft.setDesiredState(desiredStates[1]);
        backRight.setDesiredState(desiredStates[2]);
        frontRight.setDesiredState(desiredStates[3]);
    }

    // PERIODIC - runs repeatedly (like periodic from timed robot)
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Yaw", getYawAngle());
    }

}
