package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Inches;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.climber.ClimberSubsystem.ClimberDistance;
import frc.robot.subsystems.climber.ClimberSubsystem.ClimberSpeeds;
 
public class Climber {

    private ClimberSubsystem climber;
    private final Trigger isUp;
    private final Trigger isDown;
    private final Trigger isIdle;
    private final Trigger isSpinning;

    public Climber() {
        this.climber = climber;

        isUp = new Trigger(() -> climber.getClimberDistance().equals(ClimberDistance.UP));
        isDown = new Trigger(() -> climber.getClimberDistance().equals(ClimberDistance.DOWN));
        isIdle = new Trigger (() -> climber.getClimberSpeeds().equals(ClimberSpeeds.IDLE));
        isSpinning = new Trigger(() -> climber.getClimberSpeeds().equals(ClimberSpeeds.SPIN));
    }

    public Command goUp() {
        return climber.runOnce(() -> climber.setClimberDistance(ClimberDistance.UP));
    }

    public Command goDown() {
        return climber.runOnce(() -> climber.setClimberDistance(ClimberDistance.DOWN));
    }

    public Command goStill() {
        return climber.runOnce(() -> climber.setClimberSpeeds(ClimberSpeeds.IDLE));
    }

     public Command goSpin() {
        return climber.runOnce(() -> climber.setClimberSpeeds(ClimberSpeeds.SPIN));
    }

    public Command increaseDistanceFudge() {
        return climber.runOnce(() -> climber.increaseDistanceFudge(Inches.of(6)));
    }

    public Command descreaseDistanceFudge() {
        return climber.runOnce(() -> climber.descreaseDistanceFudge(Inches.of(-6)));
    }

    public Trigger isUp() {
        return isUp;
    }

    public Trigger isDown() {
        return isDown;
    }

    public Trigger isIdle() {
        return isIdle;
    }

    public Trigger isSpinning() {
        return isSpinning;
    }
}