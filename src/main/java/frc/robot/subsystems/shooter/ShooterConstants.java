package frc.robot.subsystems.shooter;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterConstants {
    public static final Angle MINIMUM_SHOOT_ANGLE = Degrees.of(45);
    public static final Angle MAX_SHOOTING_ANGLE = Degrees.of(135);
    public static final Angle ZERO_HEIGHT_ANGLE = Degrees.of(0);
    public static final AngularVelocity FLYWHEEL_SPEED = RotationsPerSecond.of(300);
    public static final AngularVelocity ZERO_SPEED = RotationsPerSecond.of(0);
}
