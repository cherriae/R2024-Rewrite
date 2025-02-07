package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;

public class Elevator extends AdvancedSubsystem {
  private final Mechanism2d _mech = new Mechanism2d(1.35, 2);
  private final MechanismRoot2d _root = _mech.getRoot("Elevator Mount", 1, 0.1);

  private final MechanismLigament2d _elevator =
      _root.append(new MechanismLigament2d("elevator", 0.1, 90, 3, new Color8Bit(Color.kCyan)));

  private final MechanismLigament2d _shooter =
      _elevator.append(
          new MechanismLigament2d("shooter", 0.3, 15, 3, new Color8Bit(Color.kAliceBlue)));

  private final TalonFX _leftElevatorMotor = new TalonFX(ElevatorConstants.elevatorLeftPort, "rio");
  private final TalonFX _rightElevatorMotor =
      new TalonFX(ElevatorConstants.elevatorRightPort, "rio");

  private final TalonFX _leftShooterMotor = new TalonFX(ShooterConstants.shooterLeftPort, "rio");
  private final TalonFX _rightShooterMotor = new TalonFX(ShooterConstants.shooterRightPort, "rio");

  // Rights follow Left
  private final StatusSignal<Angle> _heightGetter = _leftElevatorMotor.getPosition();
  private final StatusSignal<Angle> _angleGetter = _leftShooterMotor.getPosition();

  private final DynamicMotionMagicVoltage _heightSetter = new DynamicMotionMagicVoltage(0, 0, 0, 0);
  private final DynamicMotionMagicVoltage _angleSetter = new DynamicMotionMagicVoltage(0, 0, 0, 0);

  private final StatusSignal<AngularVelocity> _elevatorVelocityGetter =
      _leftElevatorMotor.getVelocity();
  private final StatusSignal<AngularVelocity> _shooterVelocityGetter =
      _leftShooterMotor.getVelocity();

  private final VelocityVoltage _elevatorVelocitySetter = new VelocityVoltage(0);
  private final VelocityVoltage _shooterVelocitySetter = new VelocityVoltage(0);

  // add later
  private final Constraints _elevatorMaxConstraints =
      new Constraints(
          ElevatorConstants.maxElevatorSpeed.in(RadiansPerSecond),
          ElevatorConstants.maxElevatorAcceleration.in(RadiansPerSecondPerSecond));

  private final Constraints _shooterMaxConstraints =
      new Constraints(
          ShooterConstants.maxShooterSpeed.in(RadiansPerSecond),
          ShooterConstants.maxShooterAcceleration.in(RadiansPerSecondPerSecond));

  private final TrapezoidProfile _elevatorMaxProfile =
      new TrapezoidProfile(_elevatorMaxConstraints);

  private final TrapezoidProfile _shooterMaxProfile = new TrapezoidProfile(_shooterMaxConstraints);

  private ElevatorSim _elevatorSim;
  private FlywheelSim _shooterSim;

  private double _lastSimTime;

  private Notifier _simNotifier;

  public Elevator() {
    var leftElevatorMotorConfigs = new TalonFXConfiguration();
    var rightElevatorMotorConfigs = new TalonFXConfiguration();
    var leftShooterMotorConfigs = new TalonFXConfiguration();
    var rightShooterMotorConfigs = new TalonFXConfiguration();

    leftElevatorMotorConfigs.Slot0.kV =
        ElevatorConstants.elevatorkV.in(Volts.per(RotationsPerSecond));
    leftElevatorMotorConfigs.Slot0.kA =
        ElevatorConstants.elevatorkA.in(Volts.per(RotationsPerSecondPerSecond));

    leftElevatorMotorConfigs.Feedback.SensorToMechanismRatio = ElevatorConstants.elevatorGearRatio;

    leftShooterMotorConfigs.Slot0.kV = ShooterConstants.shooterkV.in(Volts.per(RotationsPerSecond));
    leftShooterMotorConfigs.Slot0.kA =
        ShooterConstants.shooterkA.in(Volts.per(RotationsPerSecondPerSecond));

    leftShooterMotorConfigs.Feedback.SensorToMechanismRatio = ShooterConstants.shooterGearRatio;

    CTREUtil.attempt(
        () -> _leftElevatorMotor.getConfigurator().apply(leftElevatorMotorConfigs),
        _leftElevatorMotor);
    CTREUtil.attempt(
        () -> _rightElevatorMotor.getConfigurator().apply(rightElevatorMotorConfigs),
        _rightElevatorMotor);
    CTREUtil.attempt(
        () -> _leftShooterMotor.getConfigurator().apply(leftShooterMotorConfigs),
        _leftShooterMotor);
    CTREUtil.attempt(
        () -> _rightShooterMotor.getConfigurator().apply(rightShooterMotorConfigs),
        _rightShooterMotor);

    _rightElevatorMotor.setControl(new Follower(ElevatorConstants.elevatorLeftPort, true));
    _rightShooterMotor.setControl(new Follower(ShooterConstants.shooterLeftPort, true));

    FaultLogger.register(_leftElevatorMotor);
    FaultLogger.register(_rightElevatorMotor);
    FaultLogger.register(_leftShooterMotor);
    FaultLogger.register(_rightShooterMotor);

    if (Robot.isSimulation()) {
      _elevatorSim =
          new ElevatorSim(
              DCMotor.getKrakenX60(2),
              ElevatorConstants.elevatorGearRatio,
              Units.lbsToKilograms(9.398), // change later
              ElevatorConstants.drumRadius.in(Meters),
              0,
              ElevatorConstants.maxElevatorHeight.in(Rotations)
                  * ElevatorConstants.drumCircumference.in(Meters),
              false,
              0);

      _shooterSim =
          new FlywheelSim(
              LinearSystemId.createFlywheelSystem(
                  DCMotor.getKrakenX60(1), 0.001, ShooterConstants.shooterGearRatio),
              DCMotor.getKrakenX60(2));

      startSimThread();
    }
  }

  private void startSimThread() {
    _lastSimTime = Utils.getCurrentTimeSeconds();

    _simNotifier =
        new Notifier(
            () -> {
              final double batteryVolts = RobotController.getBatteryVoltage();

              final double currentTime = Utils.getCurrentTimeSeconds();
              final double deltaTime = currentTime - _lastSimTime;

              var leftElevatorMotorSimState = _leftElevatorMotor.getSimState();
              var rightElevatorMotorSimState = _rightElevatorMotor.getSimState();
              var leftShooterSimState = _leftShooterMotor.getSimState();
              var rightShooterSimState = _rightShooterMotor.getSimState();

              leftElevatorMotorSimState.setSupplyVoltage(batteryVolts);
              rightElevatorMotorSimState.setSupplyVoltage(batteryVolts);
              leftShooterSimState.setSupplyVoltage(batteryVolts);
              rightShooterSimState.setSupplyVoltage(batteryVolts);

              _elevatorSim.setInputVoltage(
                  leftElevatorMotorSimState.getMotorVoltageMeasure().in(Volts));
              _shooterSim.setInputVoltage(leftShooterSimState.getMotorVoltageMeasure().in(Volts));

              _elevatorSim.update(deltaTime);
              _shooterSim.update(deltaTime);

              // raw rotor positions
              leftElevatorMotorSimState.setRawRotorPosition(
                  _elevatorSim.getPositionMeters()
                      / ElevatorConstants.drumCircumference.in(Meters)
                      * ElevatorConstants.elevatorGearRatio);
              rightElevatorMotorSimState.setRawRotorPosition(
                  -_elevatorSim.getPositionMeters()
                      / ElevatorConstants.drumCircumference.in(Meters)
                      * ElevatorConstants.elevatorGearRatio);

              // raw rotor velocities
              leftElevatorMotorSimState.setRotorVelocity(
                  _elevatorSim.getVelocityMetersPerSecond()
                      / ElevatorConstants.drumCircumference.in(Meters)
                      * ElevatorConstants.elevatorGearRatio);
              rightElevatorMotorSimState.setRotorVelocity(
                  -_elevatorSim.getVelocityMetersPerSecond()
                      / ElevatorConstants.drumCircumference.in(Meters)
                      * ElevatorConstants.elevatorGearRatio);

              leftShooterSimState.setRotorVelocity(
                  _shooterSim.getAngularVelocity().in(RotationsPerSecond)
                      * ShooterConstants.shooterGearRatio);
              rightShooterSimState.setRotorVelocity(
                  _shooterSim.getAngularVelocity().in(RotationsPerSecond)
                      * ShooterConstants.shooterGearRatio);

              _lastSimTime = currentTime;
            });

    _simNotifier.setName("Elevator/Shooter Sim Thread");
    _simNotifier.startPeriodic(1 / Constants.simUpdateFrequency.in(Hertz));
  }

  @Logged(name = "Elevator Velocity")
  public double getElevatorVelocity() {
    return _elevatorVelocityGetter.refresh().getValue().in(RadiansPerSecond);
  }

  @Logged(name = "Shooter Velocity")
  public double getWristVelocity() {
    return _shooterVelocityGetter.refresh().getValue().in(RadiansPerSecond);
  }

  @Logged(name = "Elevator Height")
  public double getHeight() {
    return _heightGetter.refresh().getValue().in(Radians);
  }

  @Logged(name = "Shooter Angle")
  public double getAngle() {
    return _angleGetter.refresh().getValue().in(Radians);
  }

  public Command setElevatorSpeed(DoubleSupplier elevatorSpeed) {
    return run(
        () -> {
          _leftElevatorMotor.setControl(
              _elevatorVelocitySetter.withVelocity(
                  elevatorSpeed.getAsDouble()));
        }).withName("Set Elevator Spped");
  }

  public Command setShooterSpeed(DoubleSupplier shooterSpeed) {
    return run(
        () -> {
          _leftShooterMotor.setControl(
              _shooterVelocitySetter.withVelocity(
                  shooterSpeed.getAsDouble()));
        }).withName("Set Shooter Speed");
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();
    _elevator.setLength(
        Units.radiansToRotations(getHeight()) * ElevatorConstants.drumCircumference.in(Meters));
    _shooter.setAngle(Math.toDegrees(getAngle()) - 90);
    SmartDashboard.putData("Shootelevator Visualizer", _mech);
  }

  @Override
  public void close() {}
}
