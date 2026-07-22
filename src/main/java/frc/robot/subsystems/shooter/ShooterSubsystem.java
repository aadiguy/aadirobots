package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX flywheelMotor = new TalonFX(RobotContainer.FLYWHEEL_MOTOR);
    private final TalonFX pivotMotor = new TalonFX(RobotContainer.PIVOT_MOTOR);
    private final TalonFX passthroughMotor = new TalonFX(RobotContainer.PASSTHROUGH_MOTOR);
    private ShooterStateAngles currentShooterStateAngles = ShooterStateAngles.LOW_SHOT;
    private ShooterStateSpeed currentShooterStateSpeed = ShooterStateSpeed.ZERO_SPEED;
    
    public void setFlywheelSpeed() {
        flywheelMotor.setControl(ShooterStateSpeed.FLYWHEEL_SPEED.controlRequest);

        var configuratorPassthroughMotor = passthroughMotor.getConfigurator();
        MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
        motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        configuratorPassthroughMotor.apply(motorConfigs);
    }

    public void restrictUpwardandDownwardMovement(Angle angle) {

        Angle maxAngle = ShooterStateAngles.HIGH_SHOT.angle;
        double maxAngleValue = maxAngle.in(Degrees);

        Angle minAngle = ShooterStateAngles.LOW_SHOT.angle;
        double minAngleValue = minAngle.in(Degrees);
        
        double angleValue = angle.in(Degrees);

        if (angleValue > maxAngleValue) {
            angle = maxAngle;
        } 
        
        else if  (angleValue < minAngleValue) {
            pivotMotor.setControl(ShooterStateAngles.LOW_SHOT.controlRequest);
            angle = minAngle;
        }
        pivotMotor.setControl(new MotionMagicVoltage(angle));
    }

    public void descreaseDistanceFudge(Angle angle) {
        angle = angle.minus(angle);
        pivotMotor.setControl(new MotionMagicVoltage(angle));
    }

    public void increaseDistanceFudge(Angle angle) {
        angle = angle.plus(angle);
        pivotMotor.setControl(new MotionMagicVoltage(angle)); 
    }
 
    @Override
    public void periodic() {
        Angle shooterAngle = pivotMotor.getPosition().getValue();
        SmartDashboard.putNumber("Shooter Angle", shooterAngle.in(Degrees));
    }

   public enum ShooterStateAngles {
        LOW_SHOT(new MotionMagicVoltage(ShooterConstants.MINIMUM_SHOOT_ANGLE), ShooterConstants.MINIMUM_SHOOT_ANGLE),
        HIGH_SHOT(new MotionMagicVoltage(ShooterConstants.MAX_SHOOTING_ANGLE), ShooterConstants.MAX_SHOOTING_ANGLE);

        public final ControlRequest controlRequest;
        public final Angle angle;

        ShooterStateAngles(ControlRequest controlRequest, Angle angle) {
            this.controlRequest = controlRequest;
            this.angle = angle; 
        }
    }
    
     public void setShooterAngles(ShooterStateAngles shooterStateAngles) {
            currentShooterStateAngles = shooterStateAngles;
            ControlRequest controlRequest = currentShooterStateAngles.controlRequest;
            pivotMotor.setControl(controlRequest);
        }

        public ShooterStateAngles getCurrentShooterStateAngles() {
            return currentShooterStateAngles;
        }

    public enum ShooterStateSpeed {
        FLYWHEEL_SPEED(new MotionMagicVelocityVoltage(ShooterConstants.FLYWHEEL_SPEED), ShooterConstants.FLYWHEEL_SPEED),
        ZERO_SPEED(new NeutralOut(), RotationsPerSecond.of(0));

        public final ControlRequest controlRequest;
        public final AngularVelocity speed;

        ShooterStateSpeed(ControlRequest controlRequest, AngularVelocity speed) {
            this.controlRequest = controlRequest;
            this.speed = speed;
        }
    }

    public void setShooterSpeed(ShooterStateSpeed shooterStateSpeed) {
        currentShooterStateSpeed = shooterStateSpeed;
        ControlRequest controlRequest = currentShooterStateSpeed.controlRequest;
        flywheelMotor.setControl(controlRequest);
    }

    public ShooterStateSpeed getCurrentShooterStateSpeed() {
        return currentShooterStateSpeed;
    }
}