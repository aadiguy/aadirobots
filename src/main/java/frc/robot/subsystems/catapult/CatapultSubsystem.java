package frc.robot.subsystems.catapult;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import static edu.wpi.first.units.Units.Degrees;

public class CatapultSubsystem extends SubsystemBase {

    TalonFX armMotor = new TalonFX(RobotContainer.ARM_MOTOR);
    TalonFX releaseMotor = new TalonFX(RobotContainer.RELEASE_MOTOR);
    CatapultStateAngles currentCatapultStateAngles = CatapultStateAngles.DOWN;
    Encoder encoder = new Encoder(1, 0);
    ArmFeedforward armFeedforward = new ArmFeedforward(0, 0, 0);
    PIDController pidController = new PIDController(0, 0, 0);
    public static double TargetPosition = 0.0;

    @Override
    public void periodic() {
        double pidControllerCalculation = pidController.calculate(encoder.getDistance(), TargetPosition);
        double feedForwardCalculation = armFeedforward.calculate(encoder.getDistance(), TargetPosition);
        double totalCalculation = pidControllerCalculation + feedForwardCalculation;
        armMotor.setVoltage(totalCalculation);
    }
    
    public void resitrictPosition(Angle angle) {
        
        Angle maxAngle = CatapultStateAngles.UP.angle;
        double maxAngleValue = maxAngle.in(Degrees);

        Angle minAngle = CatapultStateAngles.DOWN.angle;
        double minAngleValue = minAngle.in(Degrees);
        
        double angleValue = angle.in(Degrees);

        if (angleValue > maxAngleValue) {
            angle = maxAngle;
        } 
        
        else if  (angleValue < minAngleValue) {
            angle = minAngle;
        }
        armMotor.setControl(new MotionMagicVoltage(angle));
    }

    public void moveToWithdraw() {
        armMotor.setControl(CatapultStateAngles.ROTATE.controlRequest);
        releaseMotor.setControl(CatapultStateAngles.ROTATE.controlRequest);
        
        var configuratorArmMotor = armMotor.getConfigurator();
        MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
        motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        configuratorArmMotor.apply(motorConfigs);
        
        if (armMotor.getPosition().getValueAsDouble() > 90){
            armMotor.setControl(CatapultStateAngles.ROTATE.controlRequest);
        }
    }

    public void releaseMotorBack() {
        releaseMotor.setControl(CatapultStateAngles.DOWN.controlRequest);
    }

    public enum CatapultStateAngles {
        
        UP(new MotionMagicVoltage(CatapultConstants.MAX_DEGREES), CatapultConstants.MAX_DEGREES),
        DOWN(new MotionMagicVoltage(CatapultConstants.MIN_DEGREES), CatapultConstants.MIN_DEGREES),
        ROTATE(new MotionMagicVoltage(CatapultConstants.DEGREE_ROTATION), CatapultConstants.DEGREE_ROTATION);

        ControlRequest controlRequest;
        Angle angle;

        CatapultStateAngles(ControlRequest controlRequest, Angle angle) {
                    this.controlRequest = controlRequest;
                    this.angle = angle;
        }
    }

    public void setCatapultStateAngles(CatapultStateAngles catapultAngles) {
        currentCatapultStateAngles = catapultAngles;
        ControlRequest controlRequest = catapultAngles.controlRequest;
        armMotor.setControl(controlRequest);
    }

    public CatapultStateAngles getCatapultStateAngles() {
        return currentCatapultStateAngles;
    }
}