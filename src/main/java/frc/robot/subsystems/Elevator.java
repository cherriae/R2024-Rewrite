package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
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
import frc.robot.Constants.WristConstants;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;

public class Elevator extends AdvancedSubsystem {
  private final Mechanism2d _mech = new Mechanism2d(1.35, 2);
  private final MechanismRoot2d _root = _mech.getRoot("Elevator Mount", 1, 0.1);

  private final MechanismLigament2d _elevator =
      _root.append(new MechanismLigament2d("elevator", 0.1, 90, 3, new Color8Bit(Color.kCyan)));

  private final MechanismLigament2d _wrist =
      _elevator.append(new MechanismLigament2d("wrist", 0.3, 15, 3, new Color8Bit(Color.kPurple)));

  private final TalonFX _leftElevatorMotor = new TalonFX(ElevatorConstants.elevatorLeftPort, "rio");
  private final TalonFX _rightElevatorMotor =
      new TalonFX(ElevatorConstants.elevatorRightPort, "rio");
  private final TalonFX _wristMotor =
      new TalonFX(Constants.IntakeConstants.intakeAcutatorPort, "rio");

  // Rights follow Left for elevator
  private final StatusSignal<Angle> _heightGetter = _leftElevatorMotor.getPosition();
  private final StatusSignal<Angle> _angleGetter = _wristMotor.getPosition();

  private final DynamicMotionMagicVoltage _heightSetter = new DynamicMotionMagicVoltage(0, 0, 0, 0);
  private final DynamicMotionMagicVoltage _angleSetter = new DynamicMotionMagicVoltage(0, 0, 0, 0);

  private final StatusSignal<AngularVelocity> _elevatorVelocityGetter =
      _leftElevatorMotor.getVelocity();
  private final StatusSignal<AngularVelocity> _wristVelocityGetter = _wristMotor.getVelocity();

  private final VelocityVoltage _elevatorVelocitySetter = new VelocityVoltage(0);
  private final VelocityVoltage _wristVelocitySetter = new VelocityVoltage(0);

  private final Constraints _elevatorMaxConstraints =
      new Constraints(
          ElevatorConstants.maxElevatorSpeed.in(RadiansPerSecond),
          ElevatorConstants.maxElevatorAcceleration.in(RadiansPerSecondPerSecond));

  private final TrapezoidProfile _elevatorMaxProfile =
      new TrapezoidProfile(_elevatorMaxConstraints);

  private SingleJointedArmSim _wristSim;
  private ElevatorSim _elevatorSim;
  private double _lastSimTime;
  private Notifier _simNotifier;

  public Elevator() {

    setDefaultCommand(idle());
    var leftElevatorMotorConfigs = new TalonFXConfiguration();
    var rightElevatorMotorConfigs = new TalonFXConfiguration();
    var wristMotorConfigs = new TalonFXConfiguration();

    leftElevatorMotorConfigs.Slot0.kV =
        ElevatorConstants.elevatorkV.in(Volts.per(RotationsPerSecond));
    leftElevatorMotorConfigs.Slot0.kA =
        ElevatorConstants.elevatorkA.in(Volts.per(RotationsPerSecondPerSecond));
    leftElevatorMotorConfigs.Slot0.kS = ElevatorConstants.elevatorkS.in(Volts);
    leftElevatorMotorConfigs.Feedback.SensorToMechanismRatio = ElevatorConstants.elevatorGearRatio;
    leftElevatorMotorConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    leftElevatorMotorConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 100;
    leftElevatorMotorConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    leftElevatorMotorConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    wristMotorConfigs.Slot0.kV = WristConstants.wristkV.in(Volts.per(RotationsPerSecond));
    wristMotorConfigs.Slot0.kA = WristConstants.wristkA.in(Volts.per(RotationsPerSecondPerSecond));
    wristMotorConfigs.Feedback.SensorToMechanismRatio = WristConstants.wristGearRatio;
    wristMotorConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    wristMotorConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(90);
    wristMotorConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    wristMotorConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(0);

    CTREUtil.attempt(
        () -> _leftElevatorMotor.getConfigurator().apply(leftElevatorMotorConfigs),
        _leftElevatorMotor);
    CTREUtil.attempt(
        () -> _rightElevatorMotor.getConfigurator().apply(rightElevatorMotorConfigs),
        _rightElevatorMotor);
    CTREUtil.attempt(() -> _wristMotor.getConfigurator().apply(wristMotorConfigs), _wristMotor);

    _rightElevatorMotor.setControl(new Follower(ElevatorConstants.elevatorLeftPort, true));

    FaultLogger.register(_leftElevatorMotor);
    FaultLogger.register(_rightElevatorMotor);
    FaultLogger.register(_wristMotor);

    if (Robot.isSimulation()) {
      _elevatorSim =
          new ElevatorSim(
              DCMotor.getKrakenX60(2),
              ElevatorConstants.elevatorGearRatio,
              Units.lbsToKilograms(9.398),
              ElevatorConstants.drumRadius.in(Meters),
              0,
              ElevatorConstants.maxElevatorHeight.in(Rotations)
                  * ElevatorConstants.drumCircumference.in(Meters),
              false,
              0);

      // Add wrist simulation
      _wristSim =
          new SingleJointedArmSim(
              DCMotor.getKrakenX60(1),
              WristConstants.wristGearRatio,
              SingleJointedArmSim.estimateMOI(
                  WristConstants.shooterLength.in(Meters), Units.lbsToKilograms(8.155)),
              WristConstants.shooterLength.in(Meters),
              WristConstants.minWristAngle.in(Radians),
              WristConstants.maxWristAngle.in(Radians),
              false,
              0);

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
              var wristSimState = _wristMotor.getSimState();

              leftElevatorMotorSimState.setSupplyVoltage(batteryVolts);
              rightElevatorMotorSimState.setSupplyVoltage(batteryVolts);
              wristSimState.setSupplyVoltage(batteryVolts);

              // Update elevator simulation
              _elevatorSim.setInputVoltage(
                  leftElevatorMotorSimState.getMotorVoltageMeasure().in(Volts));
              _elevatorSim.update(deltaTime);

              // Update wrist simulation
              _wristSim.setInputVoltage(wristSimState.getMotorVoltageMeasure().in(Volts));
              _wristSim.update(deltaTime);

              // Elevator position and velocity updates
              leftElevatorMotorSimState.setRawRotorPosition(
                  _elevatorSim.getPositionMeters()
                      / ElevatorConstants.drumCircumference.in(Meters)
                      * ElevatorConstants.elevatorGearRatio);
              rightElevatorMotorSimState.setRawRotorPosition(
                  -_elevatorSim.getPositionMeters()
                      / ElevatorConstants.drumCircumference.in(Meters)
                      * ElevatorConstants.elevatorGearRatio);

              leftElevatorMotorSimState.setRotorVelocity(
                  _elevatorSim.getVelocityMetersPerSecond()
                      / ElevatorConstants.drumCircumference.in(Meters)
                      * ElevatorConstants.elevatorGearRatio);
              rightElevatorMotorSimState.setRotorVelocity(
                  -_elevatorSim.getVelocityMetersPerSecond()
                      / ElevatorConstants.drumCircumference.in(Meters)
                      * ElevatorConstants.elevatorGearRatio);

              // Wrist position and velocity updates
              wristSimState.setRawRotorPosition(
                  Units.radiansToRotations(
                      _wristSim.getAngleRads() * WristConstants.wristGearRatio));
              wristSimState.setRotorVelocity(
                  Units.radiansToRotations(
                      _wristSim.getVelocityRadPerSec() * WristConstants.wristGearRatio));

              _lastSimTime = currentTime;
            });

    _simNotifier.setName("Elevator/Wrist Sim Thread");
    _simNotifier.startPeriodic(1 / Constants.simUpdateFrequency.in(Hertz));
  }

  @Logged(name = "Elevator Velocity")
  public double getElevatorVelocity() {
    return _elevatorVelocityGetter.refresh().getValue().in(RadiansPerSecond);
  }

  @Logged(name = "Wrist Velocity")
  public double getWristVelocity() {
    return _wristVelocityGetter.refresh().getValue().in(RadiansPerSecond);
  }

  @Logged(name = "Elevator Height")
  public double getHeight() {
    return _heightGetter.refresh().getValue().in(Radians);
  }

  @Logged(name = "Wrist Angle")
  public double getAngle() {
    return _angleGetter.refresh().getValue().in(Radians);
  }

  private Command idle() {
    return run(() -> {
          _leftElevatorMotor.setControl(_elevatorVelocitySetter.withVelocity(0));
          _wristMotor.setControl(_wristVelocitySetter.withVelocity(0));
        })
        .withName("Idle");
  }

  public Command setElevatorSpeed(DoubleSupplier elevatorSpeed) {
    return run(() -> {
          _leftElevatorMotor.setControl(
              _elevatorVelocitySetter.withVelocity(
                  Units.radiansToRotations(elevatorSpeed.getAsDouble())));
        })
        .withName("Set Elevator Speed");
  }

  public Command setWristAngle(DoubleSupplier angle) {
    return run(() -> {
          _wristMotor.setControl(
              _wristVelocitySetter.withVelocity(Units.radiansToRotations(angle.getAsDouble())));
        })
        .withName("Set Wrist Angle");
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
    _wrist.setAngle(Math.toDegrees(getAngle()) - 90);
    SmartDashboard.putData("Elevator Visualizer", _mech);
  }

  @Override
  public void close() {
    if (_simNotifier != null) {
      _simNotifier.close();
    }
  }
}
