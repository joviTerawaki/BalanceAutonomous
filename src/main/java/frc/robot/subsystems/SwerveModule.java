package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class SwerveModule extends SubsystemBase{
    private CANSparkMax turningMotor;
    private WPI_TalonFX drivingMotor;

    private CANCoder absoluteEncoder;
    private RelativeEncoder turningEnc;
    private TalonFXSensorCollection drivingEnc;

    private PIDController turningPID;

    private double encoderOffset;
    private boolean encoderReversed;

    private double kp, ki, kd;

    public SwerveModule(int neoPort, int talonPort, int cancoderPort, double encoderOffset, boolean encoderReversed, boolean driveReversed){
        turningMotor = new CANSparkMax(neoPort, MotorType.kBrushless);
        drivingMotor = new WPI_TalonFX(talonPort);

        absoluteEncoder = new CANCoder(cancoderPort);
        turningEnc = turningMotor.getEncoder();
        drivingEnc = new TalonFXSensorCollection(drivingMotor);

        turningPID = new PIDController(SwerveConsts.kp, SwerveConsts.ki, SwerveConsts.kd);
        turningPID.enableContinuousInput(-Math.PI, Math.PI); // System is circular;  Goes from -Math.PI to 0 to Math.PI

        this.encoderOffset = encoderOffset;
        this.encoderReversed = encoderReversed;

        turningEnc.setPositionConversionFactor(SwerveConsts.turningEncoderRotationConversion);
        turningEnc.setVelocityConversionFactor(SwerveConsts.turningEncoderSpeedConversion);

        drivingMotor.setInverted(driveReversed);

        resetEncoders();

    }

    /* * * ENCODER VALUES * * */

    public double getDrivePosition(){
        return drivingEnc.getIntegratedSensorPosition();
    }

    public double getTurningPosition(){
        return turningEnc.getPosition();
    }

    public double getDriveSpeed(){
        return drivingEnc.getIntegratedSensorVelocity();
    }

    public double getTurningSpeed(){
        return turningEnc.getVelocity();
    }

    public double getAbsoluteEncoder(){
        double angle = (absoluteEncoder.getAbsolutePosition() * 2.0 * Math.PI) - encoderOffset; // in radians
        if(encoderReversed){
            return (angle * -1);
        } else {
            return angle;
        }
    }

    // set turning enc to value of absolute encoder
    public void resetEncoders(){
        drivingEnc.setIntegratedSensorPosition(0, 0);
        turningEnc.setPosition(getAbsoluteEncoder());
    }

    public SwerveModuleState getState(){
        // Rotation2d = rotation represented by a point on the unit circle
        // Rotation2d(double) => constructs a Rotation2d given the angle in radians
        
        // SwerveModuleState = state of a swerve module
        // SwerveModuleState(speed (in meters per second), angle of module (Using Rotation2d))
        return new SwerveModuleState(getDriveSpeed(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state){
        // To make keep robot from going back to 0 position
        if(Math.abs(state.speedMetersPerSecond) < 0.01){
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);

        // set speed
        drivingMotor.set(state.speedMetersPerSecond / SwerveConsts.maxSpeed_mps); 
        turningMotor.set(turningPID.calculate(getTurningPosition(), state.angle.getRadians()));

        // Print to SmartDashboard
        SmartDashboard.putString("Swerve["+absoluteEncoder.getDeviceID()+"] state", state.toString());
        // SmartDashboard.putNumber("Swerve["+absoluteEncoder.getDeviceID()+"] turning enc", getTurningPosition());
        // SmartDashboard.putNumber("Swerve["+absoluteEncoder.getDeviceID()+"] CANCoder", getAbsoluteEncoder());
        // SmartDashboard.putNumber("Swerve["+absoluteEncoder.getDeviceID()+"] Drive Speed", getDriveSpeed());
        // SmartDashboard.putNumber("Swerve["+absoluteEncoder.getDeviceID()+"] NEO Output", drivingMotor.getMotorOutputPercent());
    }

    public void stop(){
        drivingMotor.set(0);
        turningMotor.set(0);
    }
    
    public void driveNeo(){
        turningMotor.set(0.1);
    }

    @Override
    public void periodic(){
        kp = SmartDashboard.getNumber("kP", 0);
        SmartDashboard.putNumber("kP", kp);
        ki = SmartDashboard.getNumber("kI", 0);
        SmartDashboard.putNumber("kI", ki);
        kd = SmartDashboard.getNumber("kD", 0);
        SmartDashboard.putNumber("kD", kd);

        turningPID.setPID(kp, ki, kd);
    }

    // constructor
    // double solenoid, neo550 (CANSparkMax), relative encoder
    // methods to close, open
    // rotate clockwise rotate counter clockwise

}
