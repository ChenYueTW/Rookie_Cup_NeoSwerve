package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveConstants;
import frc.robot.lib.helpers.IDashboardProvider;
import frc.robot.lib.motor.SwerveSpark;
import frc.robot.lib.swerve.DriveEncoder;
import frc.robot.lib.swerve.TurnEncoder;

public class SwerveModule implements IDashboardProvider {
    private final SwerveSpark driveMotor;
    private final SwerveSpark turnMotor;

    private final DriveEncoder driveEncoder;
    private final TurnEncoder turnEncoder;

    private final PIDController turnPid;

    private final String motorName;
    private double driveOutput;
    private double turnOutput;

    /**
     * Constructs a SwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * Swerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    public SwerveModule(
        int driveMotorPort, int turnMotorPort, int turnEncoderPort,
        boolean driveMotorReverse, boolean turnMotorReverse, boolean driveEncoderReverse,
        String motorName
    ){
        this.registerDashboard();

        this.driveMotor = new SwerveSpark(driveMotorPort, driveMotorReverse);
        this.turnMotor = new SwerveSpark(turnMotorPort, turnMotorReverse);

        this.driveEncoder = new DriveEncoder(this.driveMotor.getEncoder(), driveEncoderReverse);
        this.turnEncoder = new TurnEncoder(turnEncoderPort);

        // Convert motor angle to wheel angle in rotations.
        this.driveEncoder.setPositionConversionFactor(SwerveConstants.DRIVE_POSITION_CONVERSION_FACTOR);
        // Convert the number of motor turns to the number of wheel turns in m/s.
        this.driveEncoder.setVelocityConversionFactor(SwerveConstants.DRIVE_VELOCITY_CONVERSION_FACTOR);

        this.turnPid = new PIDController(0.0065, 0.00005, 0.0); // TODO
        // Angle use -0.5 to 0.5, Radian use -Math.PI to Math.PI.
        this.turnPid.enableContinuousInput(-180, 180);

        this.motorName = motorName;
    }
    
    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            this.driveEncoder.getVelocity(),
            Rotation2d.fromDegrees(this.turnEncoder.getAbsolutePositionDegrees())
        );
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            this.driveEncoder.getPosition(),
            Rotation2d.fromDegrees(this.turnEncoder.getAbsolutePositionDegrees())
        );
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.01) {
            this.stop();
            return;
        }
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, this.getState().angle);

        this.driveOutput = state.speedMetersPerSecond / SwerveConstants.MAX_SPEED_METERS_PER_SECOND;
        this.turnOutput = this.turnPid.calculate(
            this.getState().angle.getDegrees(), state.angle.getDegrees());

        this.driveMotor.set(this.driveOutput);
        this.turnMotor.set(this.turnOutput);
    }

    // Put Drive Velocity and Turn Position to SmartDashboard.
    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("SwerveState/" + this.motorName + " DriveVel", this.driveEncoder.getVelocity());
        SmartDashboard.putNumber("SwerveState/" + this.motorName + " TurnPos", this.turnEncoder.getAbsolutePositionDegrees());
    }

    // Stop module.
    public void stop() {
        this.driveMotor.set(0.0);
        this.turnMotor.set(0.0);
    }
}
