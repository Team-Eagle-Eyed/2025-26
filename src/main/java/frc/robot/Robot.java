// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// WPILIB IMPORTS
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// CTRE IMPORTS
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

// REVROBOTICS IMPORTS
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Robot extends TimedRobot {
  private final TalonFX leftMoter = new TalonFX(3);
  private final TalonFX rightMoter = new TalonFX(1);

  private final PIDController turnPID = new PIDController(0.02, 0.0, 0.001);

  private final DutyCycleOut leftOut = new DutyCycleOut(0);
  private final DutyCycleOut rightOut = new DutyCycleOut(0);

  private SparkFlex IntakeMotor;
  private SparkFlexConfig IntakeRollerConfig;
  private SparkLimitSwitch IntakeForwardLimit;
  private SparkLimitSwitch IntakeBackwardsLimit;
  private RelativeEncoder IntakeEncoder;

  private SparkMax IntakeExtender;
  private SparkMaxConfig IntakeExtenderConfig;
  private SparkLimitSwitch IntakeExtenderForwardLimit;
  private SparkLimitSwitch IntakeExtenderBackwardsLimit;
  private RelativeEncoder IntakeExtenderEncoder;

  private SparkMax AgitatorMotor;
  private SparkMaxConfig AgitatorConfig;
  private SparkLimitSwitch AgitatorForwardLimit;
  private SparkLimitSwitch AgitatorBackwardsLimit;
  private RelativeEncoder AgitatorEncoder;

  private SparkMax Kicker;
  private SparkMaxConfig KickerConfig;
  private SparkLimitSwitch KickerForwardLimit;
  private SparkLimitSwitch KickerBackwardsLimit;
  private RelativeEncoder KickerEncoder;


  public Robot() {
    // Define Motor (CHANGE IDS)
    IntakeMotor =  new SparkFlex(0, MotorType.kBrushless);
    IntakeExtender = new SparkMax(0, MotorType.kBrushless);
    AgitatorMotor = new SparkMax(0, MotorType.kBrushless);
    Kicker = new SparkMax(0, MotorType.kBrushless);

    // Setup Limit Configs
    IntakeForwardLimit = IntakeMotor.getForwardLimitSwitch();
    IntakeBackwardsLimit = IntakeMotor.getReverseLimitSwitch();
    IntakeEncoder = IntakeMotor.getEncoder();

    AgitatorForwardLimit = AgitatorMotor.getForwardLimitSwitch();
    AgitatorBackwardsLimit = AgitatorMotor.getReverseLimitSwitch();
    AgitatorEncoder = AgitatorMotor.getEncoder();

    IntakeExtenderForwardLimit = IntakeExtender.getForwardLimitSwitch();
    IntakeExtenderBackwardsLimit = IntakeExtender.getReverseLimitSwitch();
    IntakeExtenderEncoder = IntakeExtender.getEncoder();

    KickerForwardLimit = Kicker.getForwardLimitSwitch();
    KickerBackwardsLimit = Kicker.getReverseLimitSwitch();
    KickerEncoder = Kicker.getEncoder();

    IntakeRollerConfig = new SparkFlexConfig();
    IntakeExtenderConfig = new SparkMaxConfig();
    AgitatorConfig = new SparkMaxConfig();
    KickerConfig = new SparkMaxConfig();

    IntakeRollerConfig.idleMode(IdleMode.kBrake);
    IntakeExtenderConfig.idleMode(IdleMode.kBrake);
    AgitatorConfig.idleMode(IdleMode.kBrake);
    KickerConfig.idleMode(IdleMode.kBrake);

    IntakeExtenderConfig.limitSwitch
    .forwardLimitSwitchType(Type.kNormallyOpen)
    .forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor)
    .reverseLimitSwitchType(Type.kNormallyOpen)
    .reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor); 
    
    IntakeRollerConfig.limitSwitch
    .forwardLimitSwitchType(Type.kNormallyOpen)
    .forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor)
    .reverseLimitSwitchType(Type.kNormallyOpen)
    .reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);

    AgitatorConfig.limitSwitch
    .forwardLimitSwitchType(Type.kNormallyOpen)
    .forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor)
    .reverseLimitSwitchType(Type.kNormallyOpen)
    .reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);

    KickerConfig.limitSwitch
    .forwardLimitSwitchType(Type.kNormallyOpen)
    .forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor)
    .reverseLimitSwitchType(Type.kNormallyOpen)
    .reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);

    IntakeExtenderConfig.softLimit
    .forwardSoftLimit(25)
    .forwardSoftLimitEnabled(true)
    .reverseSoftLimit(-25)
    .reverseSoftLimitEnabled(true);

    IntakeRollerConfig.softLimit
    .forwardSoftLimit(17)
    .forwardSoftLimitEnabled(true)
    .reverseSoftLimit(-17)
    .reverseSoftLimitEnabled(true);

    AgitatorConfig.softLimit
    .forwardSoftLimit(20)
    .forwardSoftLimitEnabled(true)
    .reverseSoftLimit(-20)
    .reverseSoftLimitEnabled(true);

    KickerConfig.softLimit
    .forwardSoftLimit(25)
    .forwardSoftLimitEnabled(true)
    .reverseSoftLimit(-25)
    .reverseSoftLimitEnabled(true);

    IntakeMotor.configure(IntakeRollerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    IntakeExtender.configure(IntakeExtenderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    AgitatorMotor.configure(AgitatorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    Kicker.configure(KickerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    IntakeEncoder.setPosition(0);
    IntakeExtenderEncoder.setPosition(0);
    AgitatorEncoder.setPosition(0);
    KickerEncoder.setPosition(0);
  }

// ADD SMARTDASHBOARD UPDATES

  @Override
  public void robotInit() {
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();

    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    
    leftMoter.getConfigurator().apply(leftConfig);
    rightMoter.getConfigurator().apply(rightConfig);

    turnPID.setTolerance(1.0);
    turnPID.enableContinuousInput(-180.0, 180.0);
  }

  public void teleoPeriodic() {
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
    double turn = turnPID.calculate(tx, 0.0);
    turn = Math.max(Math.min(turn, 0.5), -0.5);
    leftOut.Output = turn;
    rightOut.Output = turn;
    leftMoter.setControl(leftOut);
    rightMoter.setControl(rightOut);

    if (turnPID.atSetpoint()) {
        leftOut.Output = 0;
        rightOut.Output = 0;
        leftMoter.setControl(leftOut);
        rightMoter.setControl(rightOut);
    }
  }
}