package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmCommand extends Command {
    private final Arm arm;
    private final DoubleSupplier doubleSupplier;
    public ArmCommand(Arm arm, DoubleSupplier doubleSupplier) {
        this.arm = arm;
        this.doubleSupplier = doubleSupplier;
        addRequirements(arm);
    }
 // Called when the command is initially scheduled.
 @Override
 public void initialize() {}

 // Called every time the scheduler runs while the command is scheduled.
 @Override
 public void execute() {
    arm.set_angle(Rotation2d.fromDegrees(doubleSupplier.getAsDouble()));
 }

 // Called once the command ends or is interrupted.
 @Override
 public void end(boolean interrupted) {
    arm.stop_arm();
 }
}
