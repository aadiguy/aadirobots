package frc.robot.subsystems.collector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.collector.CollectorSubsystem.CollectorStateAngles;
import frc.robot.subsystems.collector.CollectorSubsystem.CollectorStateSpeed;
  
public class Collector {

    private final CollectorSubsystem collector;
    private final Trigger isCollecting;
    private final Trigger isDeploying;
    private final Trigger isIdle;
    private final Trigger isStowing;

    public Collector(CollectorSubsystem collector) {
        this.collector = collector;

        isCollecting = new Trigger(() -> collector.getCollectorStateSpeed().equals(CollectorStateSpeed.INTAKE));
        isDeploying = new Trigger (() -> collector.getCollectorAngles().equals(CollectorStateAngles.DEPLOY));
        isIdle = new Trigger (() -> collector.getCollectorStateSpeed().equals(CollectorStateSpeed.IDLE));
        isStowing = new Trigger (() -> collector.getCollectorAngles().equals(CollectorStateAngles.STOW));
    }

   public Command deployCollector() {
        return collector.runOnce(() -> collector.deployCollector());
    }

    public Command intakeCollector() {
        return collector.run(() -> collector.gamePieceDetetction());
    }

    public Command idleCollector() {
        return collector.runOnce(() -> collector.stop());
    }

    void cancelDeploy() {
    if (idleCollector().isScheduled()) {
            deployCollector().cancel();
        }
    }

    public Command stowCollector() {
        return collector.runOnce(() -> collector.stowCollector());
    }

    public Trigger isCollecting() {
        return isCollecting;
    }

    public Trigger isDeploying() {
        return isDeploying;
    }

    public Trigger isIdle() {
        return isIdle;
    }

    public Trigger isStowing() {
        return isStowing;
    }
}