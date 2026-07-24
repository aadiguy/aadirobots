package frc.robot.subsystems.climber;

import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
 
public class ClimberConstants {

    public static final AngularVelocity MOTOR_ROTATION = RotationsPerSecond.of(0.441421356);
    public static final AngularVelocity IDLE_ROTATION = RotationsPerSecond.of(0);
    public static final double MINIMUM_HEIGHT = Units.feetToMeters(1);
    public static final double MAXIMUM_HEIGHT = Units.feetToMeters(9.0);

}