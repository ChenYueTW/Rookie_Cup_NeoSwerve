package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.encoder.RevEncoder;
import frc.robot.lib.motor.SparkModule;
import frc.robot.lib.motor.TalonModule;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkModule intake = new SparkModule(0, false, false);
    private final TalonModule lifter = new TalonModule(0, false, true);
    private final RevEncoder encoder = new RevEncoder(0);
    
    
}
