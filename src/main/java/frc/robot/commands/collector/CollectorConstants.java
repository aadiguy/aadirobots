package frc.robot.commands.collector;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import static edu.wpi.first.units.Units.*;

public class CollectorConstants {

    static final Angle DEPLOY_ANGLE  = Degrees.of(120);
    static final Angle IDLE_ANGLE = Degrees.of(0);
    static final Angle STOW_ANGLE  = Degrees.of(100);
    
    static final AngularVelocity INTAKE_SPEED = RotationsPerSecond.of(10);
    static final AngularVelocity IDLE_POSITION = RotationsPerSecond.of(0);
}