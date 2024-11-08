package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveConstants;
import frc.robot.lib.helpers.IDashboardProvider;
import frc.robot.lib.math.MathHelper;
import frc.robot.lib.motor.SwerveSpark;

public class SwerveModule implements IDashboardProvider{
    private final SwerveSpark driveMotor;
    private final SwerveSpark turnMotor;

    private final RelativeEncoder driveEncoder;
    private final CANcoder turnEncoder;

    private final PIDController drivePid;
    private final PIDController turnPid;

    private final String motorName;
    private double driveOutput;
    private double turnOutput;

    private final boolean driveEncoderReversed;

    public SwerveModule(
        int driveMotorPort, int turnMotorPort, int turnEncoderPort,
        boolean driveMotorReverse, boolean turnMotorReverse, boolean driveEncoderReverse,
        String motorName
    ){
        this.registerDashboard();

        this.driveMotor = new SwerveSpark(driveMotorPort, driveMotorReverse);
        this.turnMotor = new SwerveSpark(turnMotorPort, turnMotorReverse);

        this.driveEncoder = this.driveMotor.getEncoder();
        this.turnEncoder = new CANcoder(turnEncoderPort);

        this.driveEncoderReversed = driveEncoderReverse;
        this.driveEncoder.setPositionConversionFactor(SwerveConstants.DRIVE_POSITION_CONVERSION_FACTOR);
        this.driveEncoder.setVelocityConversionFactor(SwerveConstants.DRIVE_VELOCITY_CONVERSION_FACTOR);

        this.drivePid = new PIDController(0.0, 0.0, 0.0);

        this.turnPid = new PIDController(0.0065, 0.00005, 0.0);
        this.turnPid.enableContinuousInput(-180, 180);

        this.motorName = motorName;
    }

    public double getDriveEncoderPosition() {
        return this.driveEncoder.getPosition() * (this.driveEncoderReversed ? 1 : -1);
    }

    public double getDriveEncoderVelocity() {
        return this.driveEncoder.getVelocity() * (this.driveEncoderReversed ? 1 : -1);
    }
    
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            this.getDriveEncoderVelocity(),
            Rotation2d.fromDegrees(this.getTurningEncoderPosition())
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            this.getDriveEncoderPosition(),
            Rotation2d.fromDegrees(this.getTurningEncoderPosition())
        );
    }

    public double getTurningEncoderPosition() {
        double value = Units.rotationsToDegrees(this.turnEncoder.getAbsolutePosition().getValue());
        value %= 360.0;
        return value > 180 ? value - 360 : value;
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, this.getState().angle);

        this.driveOutput = this.drivePid.calculate(MathHelper.rpmToMpS(this.driveEncoder.getVelocity()), state.speedMetersPerSecond);
        this.turnOutput = this.turnPid.calculate(this.getState().angle.getDegrees(), state.angle.getDegrees());

        this.driveMotor.set(this.driveOutput);
        this.turnMotor.set(this.turnOutput);
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("SwerveState/" + this.motorName + " DriveVel", this.getState().speedMetersPerSecond);
        SmartDashboard.putNumber("SwerveState/" + this.motorName + " TurnPos", this.getTurningEncoderPosition());
    }

    public void stop() {
        this.driveMotor.set(0.0);
        this.turnMotor.set(0.0);
    }
}
