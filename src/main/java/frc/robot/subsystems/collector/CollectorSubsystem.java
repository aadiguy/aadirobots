package frc.robot.commands.collector;
 
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotContainer;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import edu.wpi.first.units.measure.AngularVelocity;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.units.measure.Angle;
 
public class CollectorSubsystem extends SubsystemBase {

    private final TalonFX rightDeployMotor = new TalonFX(RobotContainer.RIGHT_DEPLOY_MOTOR);
    private final TalonFX leftDeployMotor = new TalonFX(RobotContainer.LEFT_DEPLOY_MOTOR);
    private final TalonFX intakeMotor = new TalonFX(RobotContainer.INTAKE_MOTOR);
    private final DigitalInput beamBrake = new DigitalInput(RobotContainer.BEAM_BRAKE);
    private CollectorStateSpeed currentCollectorStateSpeed = CollectorStateSpeed.IDLE; 
    private CollectorStateAngles currentCollectorStateAngles = CollectorStateAngles.IDLE;

   public CollectorSubsystem() {
            leftDeployMotor.setControl(new Follower(RobotContainer.RIGHT_DEPLOY_MOTOR, MotorAlignmentValue.Aligned));
    }

     void deployCollector() {
        rightDeployMotor.setControl(CollectorStateAngles.DEPLOY.controlRequest);
    }

    void stop() {
        rightDeployMotor.setControl(CollectorStateAngles.IDLE.controlRequest);
    }

    void stowCollector() {
        rightDeployMotor.setControl(CollectorStateAngles.STOW.controlRequest);
    }

    boolean isGamePieceDetected() {
         return beamBrake.get();
    }

void gamePieceDetetction() {
    if (isGamePieceDetected()) {
        intakeMotor.setControl(CollectorStateSpeed.INTAKE.controlRequest);
        rightDeployMotor.setControl(CollectorStateAngles.STOW.controlRequest);
        

    } else {
        intakeMotor.setControl(CollectorStateSpeed.IDLE.controlRequest);
        }
    } 

enum CollectorStateSpeed {
    INTAKE(new MotionMagicVelocityVoltage(CollectorConstants.INTAKE_SPEED), CollectorConstants.INTAKE_SPEED),
    IDLE(new MotionMagicVelocityVoltage(CollectorConstants.IDLE_POSITION), CollectorConstants.IDLE_POSITION);

    final AngularVelocity angularVelocity;
    final ControlRequest controlRequest;

    CollectorStateSpeed(ControlRequest controlRequest, AngularVelocity angularVelocity) {
        this.angularVelocity = angularVelocity;
        this.controlRequest = controlRequest;
    }
}

enum CollectorStateAngles {
    DEPLOY(new MotionMagicVoltage(CollectorConstants.DEPLOY_ANGLE), CollectorConstants.DEPLOY_ANGLE),
    IDLE(new MotionMagicVoltage(CollectorConstants.IDLE_ANGLE), CollectorConstants.IDLE_ANGLE ),
    STOW(new MotionMagicVoltage(CollectorConstants.STOW_ANGLE), CollectorConstants.STOW_ANGLE);

    final Angle angle;
    final ControlRequest controlRequest;

    CollectorStateAngles(ControlRequest controlRequest, Angle angle) {
        this.controlRequest = controlRequest;
        this.angle = angle;
        }
    }

void setCollectorAngles(CollectorStateAngles collectorStateAngles) {
    currentCollectorStateAngles = collectorStateAngles;
    ControlRequest controlRequest = collectorStateAngles.controlRequest;
    rightDeployMotor.setControl(controlRequest);
}

void setCollectorSpeed(CollectorStateSpeed collectorStateSpeed) {
    currentCollectorStateSpeed = collectorStateSpeed;
    ControlRequest controlRequest = collectorStateSpeed.controlRequest;
    intakeMotor.setControl(controlRequest);
    }

 CollectorStateAngles getCollectorAngles() {
    return currentCollectorStateAngles;
    }

CollectorStateSpeed getCollectorStateSpeed() {
    return currentCollectorStateSpeed;
    }
}