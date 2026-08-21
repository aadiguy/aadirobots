package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.catapult.Catapult;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.shooter.Shooter;

public class OI {
    
    CommandXboxController driverController = new CommandXboxController(0);
    CommandXboxController climberController = new CommandXboxController(1);

    private Collector collector;
    private Shooter shooter;
    private Climber climber;
    private Catapult catapult;

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
        driverController.rightBumper().onTrue(catapult.catapultUp());
        driverController.leftBumper().onTrue(catapult.catapultDown());
        driverController.rightTrigger().onTrue(catapult.catapultRotate());
         
        climberController.y().onTrue(climber.goUp());
        climberController.a().onTrue(climber.goDown());
        climberController.b().onTrue(climber.goStill());
        climberController.x().onTrue(climber.goSpin());
        climberController.povUp().onTrue(climber.increaseDistanceFudge());
        climberController.povDown().onTrue(climber.descreaseDistanceFudge());

    }
}
