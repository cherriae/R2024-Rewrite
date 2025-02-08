package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
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
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;

public class Intake extends AdvancedSubsystem {
  private final Mechanism2d _mech = new Mechanism2d(0.625, 0.067);
  private final MechanismRoot2d _root = _mech.getRoot("pivot", 0.5, 0.1);

  private final MechanismLigament2d _intake =
      _root.append(
          new MechanismLigament2d(
              "intake",
              0.5,
              IntakeConstants.actuatorStowed.in(Degrees),
              3,
              new Color8Bit(Color.kBlue)));

  private final TalonFX _actuatorMotor = new TalonFX(IntakeConstants.intakeAcutautorPort, "rio");
  private final TalonFX _feedMotor = new TalonFX(IntakeConstants.intakeFeedPort, "rio");

  private final MotionMagicVoltage _actuatorPositionSetter = new MotionMagicVoltage(0);
  private final VelocityVoltage _feedVelocitySetter = new VelocityVoltage(0);

  private final StatusSignal<Angle> _actuatorPositionGetter = _actuatorMotor.getPosition();
  private final StatusSignal<AngularVelocity> _feedVelocityGetter = _feedMotor.getVelocity();

  private SingleJointedArmSim _actuatorSim;

  private double _lastSimTime;

  private Notifier _simNotifier;

  public Intake() {
    var feedMotorConfigs = new TalonFXConfiguration();
    var actuatorMotorConfigs = new TalonFXConfiguration();

    actuatorMotorConfigs.Slot0.kV = IntakeConstants.actuatorkV.in(Volts.per(RotationsPerSecond));
    actuatorMotorConfigs.Slot0.kA =
        IntakeConstants.actuatorkA.in(Volts.per(RotationsPerSecondPerSecond));

    actuatorMotorConfigs.Feedback.SensorToMechanismRatio = IntakeConstants.actuatorGearRatio;

    actuatorMotorConfigs.MotionMagic.MotionMagicCruiseVelocity =
        IntakeConstants.actuatorVelocity.in(RotationsPerSecond);
    actuatorMotorConfigs.MotionMagic.MotionMagicAcceleration =
        IntakeConstants.actuatorAcceleration.in(RotationsPerSecondPerSecond);

    CTREUtil.attempt(() -> _feedMotor.getConfigurator().apply(feedMotorConfigs), _feedMotor);
    CTREUtil.attempt(
        () -> _actuatorMotor.getConfigurator().apply(actuatorMotorConfigs), _actuatorMotor);

    FaultLogger.register(_feedMotor);
    FaultLogger.register(_actuatorMotor);

    setDefaultCommand(stow());

    if (Robot.isSimulation()) {
      _actuatorSim =
          new SingleJointedArmSim(
              DCMotor.getKrakenX60(1),
              IntakeConstants.actuatorGearRatio,
              SingleJointedArmSim.estimateMOI(
                  IntakeConstants.intakeLength.in(Meters), Units.lbsToKilograms(12)),
              IntakeConstants.intakeLength.in(Meters),
              IntakeConstants.actuatorStowed.in(Radians),
              IntakeConstants.actuatorOut.in(Radians),
              false,
              IntakeConstants.actuatorStowed.in(Radians));

      startSimThread();
    }
  }

  private void startSimThread() {
    _lastSimTime = Utils.getCurrentTimeSeconds();

    _simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              final double deltaTime = currentTime - _lastSimTime;

              final double batteryVoltage = RobotController.getBatteryVoltage();

              var actuatorMotorSimState = _actuatorMotor.getSimState();

              actuatorMotorSimState.setSupplyVoltage(batteryVoltage);

              _actuatorSim.setInputVoltage(
                  actuatorMotorSimState.getMotorVoltageMeasure().in(Volts));

              _actuatorSim.update(deltaTime);

              actuatorMotorSimState.setRawRotorPosition(
                  Units.radiansToRotations(
                      _actuatorSim.getAngleRads() * IntakeConstants.actuatorGearRatio));

              actuatorMotorSimState.setRotorVelocity(
                  Units.radiansToRotations(
                      _actuatorSim.getVelocityRadPerSec() * IntakeConstants.actuatorGearRatio));

              _lastSimTime = currentTime;
            });

    _simNotifier.setName("Intake Sim Thread");
    _simNotifier.startPeriodic(1 / Constants.simUpdateFrequency.in(Hertz));
  }

  @Logged(name = "Actuator Angle")
  public double getAngle() {
    return _actuatorPositionGetter.refresh().getValue().in(Radians);
  }

  @Logged(name = "Feed Speed")
  public double getSpeed() {
    return _feedVelocityGetter.refresh().getValue().in(RadiansPerSecond);
  }

  // set the actuator angle and feed speed.
  private void setSpeeds(double actuatorAngle, double feedSpeed) {
    _actuatorMotor.setControl(
        _actuatorPositionSetter.withPosition(Units.radiansToRotations(actuatorAngle)));
    _feedMotor.setControl(_feedVelocitySetter.withVelocity(Units.radiansToRotations(feedSpeed)));
  }

  private Command stow() {
    return run(() -> {
          setSpeeds(IntakeConstants.actuatorStowed.in(Radians), 0);
        })
        .withName("Stow");
  }

  public Command intakeNote() {
    return run(() -> {
          setSpeeds(
              IntakeConstants.actuatorOut.in(Radians),
              IntakeConstants.feedSpeed.in(RadiansPerSecond));
        })
        .withName("Intake.");
  }

  public Command outtakeNote() {
    return run(() -> {
          setSpeeds(
              IntakeConstants.actuatorOut.in(Radians),
              -IntakeConstants.feedSpeed.in(RadiansPerSecond));
        })
        .withName("Outtake");
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    _intake.setAngle(Math.toDegrees(getAngle()));

    SmartDashboard.putData("Intake Visualizer", _mech);
  }

  @Override
  public void close() {
    _actuatorMotor.close();
    _feedMotor.close();

    _simNotifier.close();
  }
}
