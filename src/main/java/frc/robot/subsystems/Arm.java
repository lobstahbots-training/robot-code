package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private final CANSparkMax left_arm_motor;
    private final CANSparkMax right_arm_motor;
    private final PIDController controller;
    private final DutyCycleEncoder arm_encoder;
    private final ArmFeedforward feedforward;
    public Arm(int left_arm_ID, int right_arm_ID) {
        this.left_arm_motor = new CANSparkMax(left_arm_ID, MotorType.kBrushless);
        this.right_arm_motor = new CANSparkMax(right_arm_ID, MotorType.kBrushless);
        this.controller = new PIDController(0.1, 0.1, 0.1);
        this.arm_encoder = new DutyCycleEncoder(0);
        this.feedforward = new ArmFeedforward(0, 0, 0, 0);
        arm_encoder.setDistancePerRotation(360);
        left_arm_motor.setSmartCurrentLimit(40,20);
        right_arm_motor.setSmartCurrentLimit(40,20);
    }
    public void set_voltage(double left_volts, double right_volts) {
        left_arm_motor.setVoltage(left_volts);
        right_arm_motor.setVoltage(right_volts);
    }
    public void set_angle(Rotation2d rotation2d) {
        controller.setSetpoint(rotation2d.getDegrees()); //goal position
        double PIDoutput = controller.calculate(arm_encoder.getAbsolutePosition()); // current position
        double feedforward_output = feedforward.calculate(rotation2d.getDegrees(), 0);
        PIDoutput = PIDoutput + feedforward_output;
        set_voltage(PIDoutput, PIDoutput);
    }
    public void reset_error() {
        controller.reset();
    }
    public void stop_arm() {
        left_arm_motor.stopMotor();
        right_arm_motor.stopMotor();
    }
}
