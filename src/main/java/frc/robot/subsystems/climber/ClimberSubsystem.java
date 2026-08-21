package frc.robot.subsystems.climber;
 
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
 
public class ClimberSubsystem extends SubsystemBase {

    TalonFX rightClimberMotor = new TalonFX(RobotContainer.RIGHT_CLIMBER_MOTOR);
    TalonFX leftClimberMotor = new TalonFX(RobotContainer.LEFT_CLIMBER_MOTOR);
    DigitalInput climberBeamBrake = new DigitalInput(0);
    private ClimberSpeeds currentClimberSpeeds = ClimberSpeeds.IDLE;
    private ClimberDistance currentClimberDistance = ClimberDistance.DOWN;
    Encoder climberEncoder = new Encoder(0, 1);
    PIDController gamePiecePIDController = new PIDController(12.0, 13.2, 16.9);
    PIDController emptyPIDController = new PIDController(13.45, 14.857, 21.34);
    private final double targetPosition = 5.0;
    private Distance distanceFudge = Inches.of(12.0);
    private double distanceManager = distanceFudge.in(Inches);


    public ClimberSubsystem() {
        leftClimberMotor.setControl(new Follower(RobotContainer.RIGHT_CLIMBER_MOTOR, MotorAlignmentValue.Aligned));
    }

    public boolean detectGamePiece() {
       return climberBeamBrake.get();
    }

    public void adjustArm() {
        if (detectGamePiece()) {
             double pidOutput = gamePiecePIDController.calculate(climberEncoder.getDistance(), targetPosition);
             rightClimberMotor.set(pidOutput);
        } else {
            double pidOutput = emptyPIDController.calculate(climberEncoder.getDistance(), targetPosition);
             rightClimberMotor.set(pidOutput);
        }
    }

    public void spinClimber() {
        rightClimberMotor.setControl(ClimberSpeeds.SPIN.controlRequest);
    }

    public void idleClimber() {
        rightClimberMotor.setControl(ClimberSpeeds.IDLE.controlRequest);
    }

    public void jerkUporDown() {

        if (distanceManager > ClimberConstants.MAXIMUM_HEIGHT || distanceManager < ClimberConstants.MINIMUM_HEIGHT) {
             rightClimberMotor.setControl(ClimberDistance.SAFE.controlRequest);
        }
    }

    public void moveUpContinously() {
         rightClimberMotor.set(0.5);
    }

    public void moveDownContinously() {
         rightClimberMotor.set(-0.5);
    }

    public void increaseDistanceFudge(Distance distance) {
        distanceFudge = distanceFudge.plus(distance);
        updateDistanceFudge();
    }

    public void descreaseDistanceFudge(Distance distance) {
        distanceFudge = distanceFudge.minus(distance);
        updateDistanceFudge();
    }

    public void updateDistanceFudge() {
        SmartDashboard.putNumber("Distance of the climber", distanceFudge.in(Inches));
        distanceManager = distanceFudge.in(Inches);
    }

    enum ClimberSpeeds {
        SPIN(new MotionMagicVelocityVoltage(ClimberConstants.MOTOR_ROTATION), ClimberConstants.MOTOR_ROTATION),
        IDLE(new MotionMagicVelocityVoltage(ClimberConstants.IDLE_ROTATION), ClimberConstants.IDLE_ROTATION);

        final ControlRequest controlRequest;
        final AngularVelocity angularVelocity;

        ClimberSpeeds(ControlRequest controlRequest, AngularVelocity angularVelocity) {
            this.controlRequest = controlRequest;
            this.angularVelocity = angularVelocity;
        }
    }

    public void setClimberSpeeds(ClimberSpeeds climberSpeeds) {
        currentClimberSpeeds = climberSpeeds;
        ControlRequest controlRequest = currentClimberSpeeds.controlRequest;
        rightClimberMotor.setControl(controlRequest);
    }

    public ClimberSpeeds getClimberSpeeds() {
        return currentClimberSpeeds; 
    }

    enum ClimberDistance {
        DOWN(new MotionMagicVoltage(ClimberConstants.MINIMUM_HEIGHT), ClimberConstants.MINIMUM_HEIGHT),
        UP(new MotionMagicVoltage(ClimberConstants.MAXIMUM_HEIGHT), ClimberConstants.MAXIMUM_HEIGHT ),
        SAFE(new MotionMagicVoltage(ClimberConstants.SAFE_HEIGHT), ClimberConstants.SAFE_HEIGHT);

        final ControlRequest controlRequest;
        final double dooble;

        ClimberDistance(ControlRequest controlRequest, double dooble) {
            this.controlRequest = controlRequest;
            this.dooble = dooble;
        }
    }

    public void setClimberDistance(ClimberDistance climberdistance) {
        currentClimberDistance = climberdistance;
        ControlRequest controlRequest = currentClimberDistance.controlRequest;
        rightClimberMotor.setControl(controlRequest);
    }

    public ClimberDistance getClimberDistance() {
        return currentClimberDistance;
    }
}