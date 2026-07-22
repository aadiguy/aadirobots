package frc.robot.subsystems.collector;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class CollectorConstants {

    public static final Angle DEPLOY_ANGLE  = Degrees.of(120);
    public static final Angle IDLE_ANGLE = Degrees.of(0);
    public static final Angle STOW_ANGLE  = Degrees.of(100);
    
    public static final AngularVelocity INTAKE_SPEED = RotationsPerSecond.of(10);
    public static final AngularVelocity IDLE_POSITION = RotationsPerSecond.of(0);
}