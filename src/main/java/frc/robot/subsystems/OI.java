package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.collector.Collector;


public class OI {
    
    CommandXboxController driverController = new CommandXboxController(0);
    private Collector collector;

    public void configureBindings() {
        driverController.leftBumper().onTrue(
            Commands.either(
        collector.deployCollector(),
        collector.stowCollector(),
        collector.isStowing()
        ));

        driverController.leftTrigger().onTrue(collector.intakeCollector());
        driverController.a().onTrue(collector.idleCollector());
    }
}
