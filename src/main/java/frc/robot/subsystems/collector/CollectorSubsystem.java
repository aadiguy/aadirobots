package frc.robot.subsystems.collector;
 
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class CollectorSubsystem extends SubsystemBase {

    private final TalonFX rightDeployMotor = new TalonFX(RobotContainer.RIGHT_DEPLOY_MOTOR);
    private final TalonFX leftDeployMotor = new TalonFX(RobotContainer.LEFT_DEPLOY_MOTOR);
    private final TalonFX intakeMotor = new TalonFX(RobotContainer.INTAKE_MOTOR);
    private final DigitalInput beamBrake = new DigitalInput(RobotContainer.BEAM_BRAKE);
    private CollectorStateSpeed currentCollectorStateSpeed = CollectorStateSpeed.IDLE_POSE; 
    private CollectorStateAngles currentCollectorStateAngles = CollectorStateAngles.IDLE;
    private boolean isStopped = false;

   public CollectorSubsystem() {
            leftDeployMotor.setControl(new Follower(RobotContainer.RIGHT_DEPLOY_MOTOR, MotorAlignmentValue.Aligned));
    }

    @SuppressWarnings("UnnecessaryReturnStatement")
    public void deployCollector() {
        if (stop()) {
            return;
        } else {
            rightDeployMotor.setControl(CollectorStateAngles.DEPLOY.controlRequest);
        }
    }

    boolean stop() {
        rightDeployMotor.setControl(CollectorStateAngles.IDLE.controlRequest);
        isStopped = true;
        return isStopped;
    }

    public void stowCollector() {
        rightDeployMotor.setControl(CollectorStateAngles.STOW.controlRequest);
    }

    public boolean isGamePieceDetected() {
         return beamBrake.get();
    }

    public void gamePieceDetetction() {
    if (isGamePieceDetected()) {
        intakeMotor.setControl(CollectorStateSpeed.INTAKE.controlRequest);
        rightDeployMotor.setControl(CollectorStateAngles.STOW.controlRequest);
    } else {
        intakeMotor.setControl(CollectorStateSpeed.IDLE_POSE.controlRequest);
        }
    } 

public enum CollectorStateSpeed {
    INTAKE(new MotionMagicVelocityVoltage(CollectorConstants.INTAKE_SPEED), CollectorConstants.INTAKE_SPEED),
    IDLE_POSE(new NeutralOut(), RotationsPerSecond.of(0));
    
    final ControlRequest controlRequest;

    CollectorStateSpeed(ControlRequest controlRequest, AngularVelocity angularVelocity) {
        this.controlRequest = controlRequest;
    }
}

public enum CollectorStateAngles {
    DEPLOY(new MotionMagicVoltage(CollectorConstants.DEPLOY_ANGLE), CollectorConstants.DEPLOY_ANGLE),
    IDLE(new MotionMagicVoltage(CollectorConstants.IDLE_ANGLE), CollectorConstants.IDLE_ANGLE),
    STOW(new MotionMagicVoltage(CollectorConstants.STOW_ANGLE), CollectorConstants.STOW_ANGLE);

    final ControlRequest controlRequest;

    CollectorStateAngles(ControlRequest controlRequest, Angle angle) {
        this.controlRequest = controlRequest;
        }
    }

public void setCollectorAngles(CollectorStateAngles collectorStateAngles) {

    currentCollectorStateAngles = collectorStateAngles;
    ControlRequest controlRequest = collectorStateAngles.controlRequest;
    rightDeployMotor.setControl(controlRequest);
}

public void setCollectorSpeed(CollectorStateSpeed collectorStateSpeed) {
    
    currentCollectorStateSpeed = collectorStateSpeed;
    ControlRequest controlRequest = collectorStateSpeed.controlRequest;
    intakeMotor.setControl(controlRequest);
    }

 public CollectorStateAngles getCollectorAngles() {
    return currentCollectorStateAngles;
    }

public CollectorStateSpeed getCollectorStateSpeed() {
    return currentCollectorStateSpeed;
    }
}