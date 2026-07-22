package frc.robot.subsystems.shooter;


import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterStateAngles;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterStateSpeed;
 

public class Shooter {

    private final ShooterSubsystem shooter;
    private final Trigger isShooting;
    private final Trigger isIdle;
    private final Trigger goHigh;
    private final Trigger goLow;
     
    public Shooter(ShooterSubsystem shooter) {
        this.shooter = shooter;

        isShooting = new Trigger(() -> shooter.getCurrentShooterStateSpeed().equals(ShooterStateSpeed.FLYWHEEL_SPEED));
        isIdle = new Trigger(() -> shooter.getCurrentShooterStateSpeed().equals(ShooterStateSpeed.ZERO_SPEED));
        goHigh = new Trigger(() -> shooter.getCurrentShooterStateAngles().equals(ShooterStateAngles.HIGH_SHOT));
        goLow = new Trigger (() -> shooter.getCurrentShooterStateAngles().equals(ShooterStateAngles.LOW_SHOT));
    }

    public Command increaseDistanceFudge() {
        return shooter.runOnce(() -> shooter.increaseDistanceFudge(Degrees.of(5)));
    }

    public Command decreaseDistanceFudge() {
        return shooter.runOnce(() -> shooter.descreaseDistanceFudge(Degrees.of(-5)));
    }

    public Command shoot() {
        return shooter.runOnce(() -> shooter.setShooterSpeed(ShooterStateSpeed.FLYWHEEL_SPEED));
    }

    public Command idle() {
        return shooter.runOnce(() -> shooter.setShooterSpeed(ShooterStateSpeed.ZERO_SPEED));
    }

    public Command high() {
        return shooter.runOnce(() -> shooter.setShooterAngles(ShooterStateAngles.HIGH_SHOT));
    }

    public Command low() {
        return shooter.runOnce(() -> shooter.setShooterAngles(ShooterStateAngles.LOW_SHOT));
    }

    public Trigger isShooting() {
        return isShooting;
    }

    public Trigger isIdle() {
        return isIdle;
    }

    public Trigger goHigh() {
        return goHigh;
    }

    public Trigger goLow() {
        return goLow;
    }
}