package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class TeleopWrist extends CommandBase {
    private Wrist s_Wrist;
    private DoubleSupplier moveVal;

    public TeleopWrist(Wrist s_Wrist, DoubleSupplier moveVal) {
        this.s_Wrist = s_Wrist;
        this.moveVal = moveVal;

        this.addRequirements(s_Wrist);
    }

    @Override
    public void initialize() {
        // System.out.println("Teleop Wrist Started");
    }

    @Override
    public void execute() {
        // System.out.println("Teleop Wrist Running");
        double deadbandController = Math.abs(moveVal.getAsDouble()) > .1 ? -moveVal.getAsDouble() : 0;
        s_Wrist.setPosition((s_Wrist.getPosition() + deadbandController * .05) % (Math.PI * 2));
    }
}
