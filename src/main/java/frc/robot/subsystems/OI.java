package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.shooter.Shooter;


public class OI {
    
    CommandXboxController driverController = new CommandXboxController(0);
    private Collector collector;
    private Shooter shooter;

    public void configureBindings() {
        driverController.leftBumper().onTrue(
            Commands.either(
        collector.deployCollector(),
        collector.stowCollector(),
        collector.isStowing()
        ));

        driverController.leftTrigger().onTrue(collector.intakeCollector());
        driverController.a().onTrue(collector.idleCollector());
        driverController.povUp().onTrue(shooter.increaseDistanceFudge());
        driverController.povDownLeft().onTrue(shooter.decreaseDistanceFudge());
        driverController.x().onTrue(shooter.high());
        driverController.y().onTrue(shooter.low());
        driverController.b().onTrue(shooter.idle());
        driverController.leftTrigger().onTrue(shooter.shoot());
    }
}
