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

        this.driveEncoder.setPositionConversionFactor(SwerveConstants.DRIVE_POSITION_CONVERSION_FACTOR);
        this.driveEncoder.setVelocityConversionFactor(SwerveConstants.DRIVE_VELOCITY_CONVERSION_FACTOR);

        this.turnPid = new PIDController(0.0073, 0., 0.0); // TODO
        this.turnPid.enableContinuousInput(-180, 180);

        this.motorName = motorName;
    }
    
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            this.driveEncoder.getVelocity(),
            Rotation2d.fromDegrees(this.turnEncoder.getAbsolutePositionDegrees())
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            this.driveEncoder.getPosition(),
            Rotation2d.fromDegrees(this.turnEncoder.getAbsolutePositionDegrees())
        );
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, this.getPosition().angle);

        this.driveOutput = state.speedMetersPerSecond / SwerveConstants.MAX_SPEED_METERS_PER_SECOND;
        this.turnOutput = this.turnPid.calculate(this.getState().angle.getDegrees(), state.angle.getDegrees());

        this.driveMotor.set(this.driveOutput);
        this.turnMotor.set(this.turnOutput);
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("SwerveState/" + this.motorName + " DriveVel", this.driveEncoder.getVelocity());
        SmartDashboard.putNumber("SwerveState/" + this.motorName + " TurnPos", this.turnEncoder.getAbsolutePositionDegrees());
    }

    public void stop() {
        this.driveMotor.set(0.0);
        this.turnMotor.set(0.0);
    }
}
