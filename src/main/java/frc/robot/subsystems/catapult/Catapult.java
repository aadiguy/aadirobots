package frc.robot.subsystems.catapult;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.catapult.CatapultSubsystem.CatapultStateAngles;

public class Catapult {

    private CatapultSubsystem catapult;
    private Trigger isUp;
    private Trigger isDown;
    private Trigger isRotating;

    public Catapult() {
         
        isUp = new Trigger(() -> catapult.getCatapultStateAngles().equals(CatapultStateAngles.UP));
        isDown = new Trigger(() -> catapult.getCatapultStateAngles().equals(CatapultStateAngles.DOWN));
        isDown = new Trigger(() -> catapult.getCatapultStateAngles().equals(CatapultStateAngles.ROTATE));
    }

    public Command catapultUp() {
        return catapult.runOnce(() -> catapult.setCatapultStateAngles(CatapultStateAngles.UP));
    }

    public Command catapultDown() {
        return catapult.runOnce(() -> catapult.setCatapultStateAngles(CatapultStateAngles.DOWN));
    }

    public Command catapultRotate() {
        return catapult.runOnce(catapult::moveToWithdraw).andThen(catapult::releaseMotorBack);
    }

    public Trigger isUp() {
        return isUp;
    }

    public Trigger isDown() {
        return isDown;
    }

    public Trigger isRotating() {
        return isRotating;
    }
}
