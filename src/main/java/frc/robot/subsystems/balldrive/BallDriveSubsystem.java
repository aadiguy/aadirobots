package frc.robot.subsystems.balldrive;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
 

public class BallDriveSubsystem extends SubsystemBase {

    TalonFX forwardBackwardDrive = new TalonFX(RobotContainer.FORWARD_BACKWARD_DRIVE);
    TalonFX leftRightDrive = new TalonFX(RobotContainer.LEFT_RIGHT_DRIVE);
    Encoder forwardBackwardEncoder =  new Encoder(1, 2); 
    Encoder leftRightEncoder = new Encoder(3, 4);
    PIDController pidController = new PIDController(1.5, 23.4, 12.3);
    PIDController otherPidController = new PIDController(1.5, 23.4, 12.3);
    SimpleMotorFeedforward feedforwards = new SimpleMotorFeedforward(2.3, 23.3, 12.3);
    public static double TargetPosition = 0.0;

    public BallDriveSubsystem() {
        forwardBackwardDrive.setControl(new Follower(RobotContainer.LEFT_RIGHT_DRIVE, MotorAlignmentValue.Aligned));
    }

    Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
    Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
    Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
    Translation2d backRightLocation = new Translation2d(-0.381, -0.381);
    //

    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
  frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    SwerveModuleState frontLeftState = new SwerveModuleState(84.3, Rotation2d.fromDegrees(32));
    SwerveModuleState frontRightState = new SwerveModuleState(83.2, Rotation2d.fromDegrees(43));
    SwerveModuleState backLeftState = new SwerveModuleState(34.5, Rotation2d.fromDegrees(23));
    SwerveModuleState backRightState = new SwerveModuleState(432.3, Rotation2d.fromDegrees(21));

    ChassisSpeeds chassisSpeed = kinematics.toChassisSpeeds(frontLeftState, frontRightState, backLeftState, backRightState);

    double forwards = chassisSpeed.vxMetersPerSecond;
    double side = chassisSpeed.vyMetersPerSecond;
    double rotate = chassisSpeed.omegaRadiansPerSecond;

    @Override
    public void periodic() {    
        double pidControllerCalculation = pidController.calculate(forwardBackwardEncoder.getRate());
        double otherPidControllerCalculation = otherPidController.calculate(leftRightEncoder.getRate());
        double feedforwardCalculation =  feedforwards.calculate(forwardBackwardEncoder.getRate());
        forwardBackwardDrive.set(pidControllerCalculation + feedforwardCalculation + otherPidControllerCalculation);

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(forwards, side, rotate);
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

    SwerveModuleState frontLeft = moduleStates[0];
    SwerveModuleState frontRight = moduleStates[1];
    SwerveModuleState backLeft = moduleStates[2];
    SwerveModuleState backRight = moduleStates[3];
 

    // prevents swerve from rotating more than 90 degrees
        Rotation2d currentAngle = Rotation2d.fromRadians(forwardBackwardEncoder.getDistance());
        frontLeft.optimize(currentAngle);
        frontRight.optimize(currentAngle);
        backLeft.optimize(currentAngle);
        backRight.optimize(currentAngle);
        frontLeft.speedMetersPerSecond *= frontLeft.angle.minus(currentAngle).getCos();
        frontRight.speedMetersPerSecond *= frontRight.angle.minus(currentAngle).getCos();
        backLeft.speedMetersPerSecond *= backLeft.angle.minus(currentAngle).getCos();
        backRight.speedMetersPerSecond *= backRight.angle.minus(currentAngle).getCos();
    
    }
}                   