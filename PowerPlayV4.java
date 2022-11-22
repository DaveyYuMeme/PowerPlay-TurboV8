package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class PowerPlayV4 extends LinearOpMode {
  // Control Hub Motors
  private DcMotorWrapper frontLeft;
  private DcMotorWrapper linSlide;

  // Control Hub Servos
  private ServoWrapper deposit;
  private ServoWrapper linearServo;
  private ServoWrapper clawServo2;
  private ServoWrapper clawServo1;
  private ServoWrapper frontArm;
  
  // Expansion Hub Motors
  private DcMotorWrapper frontRight;
  private DcMotorWrapper backRight;
  private DcMotorWrapper backLeft;
  private DcMotorWrapper turret;
  
  // Expansion Hub Servos
  private ServoWrapper rightArm;
  private ServoWrapper leftArm;

  // Expansion Hub Digital Sensors
  private TouchSensor leftTouchSensor;
  private TouchSensor rightTouchSensor;

  // Gamepad Wrappers
  private GamepadWrapper gamepad1Wrapper;

  // Meta
  private TimeManager timeManager;

  // Drivetrain Wrappers
  private MecanumWrapper mecanumWrapper;
  private OdometryWrapper odometryWrapper;

  // Control Parameters
  private double armLowerBound = 0.5;
  private double armUpperBound = 0.82;
  private double clawLowerBound = 0.5;
  private double clawUpperBound = 0.87;
  private double frontArmLowerBound = 0.5;
  private double frontArmUpperBound = 0.95;
  private int linSlideLowerBound = -0;
  private int linSlideUpperBound = -900;
  private double linSlidePower = 0.8;
  private double depositLowerBound = 0.5;
  private double depositUpperBound = 0.9;
  private int turretLowerBound = 0;
  private int turretUpperBound = 100;
  private double linearServoLowerBound = 0.1;
  private double linearServoUpperBound = 0.9;

  // Odometry Parameters
  private double inchesToTicks = 1901.86;
  private double degreesToTicks = 100;
  private double trackWidth = 11.25;
  private double forwardOffset = 5.35;
  
  // Position Parameters
  private double[] armPositions = { 0.0, 0.5, 1.0 };
  private int armPosition = 0;
  private double[] clawPositions = { 0.0, 1.0 };
  private int clawPosition = 0;
  private double[] frontArmPositions = { 0.0, 1.0 };
  private int frontArmPosition = 0;
  private double[] linSlidePositions = { 0.0, 0.33, 0.67, 1.0 };
  private int linSlidePosition = 0;
  private double[] depositPositions = { 0.0, 0.5, 1.0 };
  private int depositPosition = 0;
  private double[] linearServoPositions = { 0.0, 1.0 };
  private int linearServoPosition = 0;
  
  private void initControlHub() {
    telemetry.addData("Status", "Initializing Control Hub");
    telemetry.update();
    
    // Get motors
    this.linSlide = new DcMotorWrapper()
      .setDcMotor(hardwareMap.dcMotor.get("LinSlide"))
      .setLowerBound(this.linSlideLowerBound)
      .setUpperBound(this.linSlideUpperBound)
      .setPower(this.linSlidePower);

    // Get servos
    this.deposit     = new ServoWrapper()
      .setServo(hardwareMap.servo.get("Deposit"))
      .setLowerBound(this.depositLowerBound)
      .setUpperBound(this.depositUpperBound);
    this.linearServo = new ServoWrapper()
      .setServo(hardwareMap.servo.get("LinearServo"))
      .setLowerBound(this.linearServoLowerBound)
      .setUpperBound(this.linearServoUpperBound);
    this.clawServo1  = new ServoWrapper()
      .setServo(hardwareMap.servo.get("ClawServo1"))
      .setLowerBound(1.0 - this.clawLowerBound)
      .setUpperBound(1.0 - this.clawUpperBound);
    this.clawServo2  = new ServoWrapper()
      .setServo(hardwareMap.servo.get("ClawServo2"))
      .setLowerBound(this.clawLowerBound)
      .setUpperBound(this.clawUpperBound);
    this.frontArm    = new ServoWrapper()
      .setServo(hardwareMap.servo.get("FrontArm"))
      .setLowerBound(this.frontArmLowerBound)
      .setUpperBound(this.frontArmUpperBound);
    
    telemetry.addData("Status", "Initialized Control Hub");
    telemetry.update();
  }
  
  private void initExpansionHub() {
    telemetry.addData("Status", "Initializing Expansion Hub");
    telemetry.update();
    
    // Get motors
    this.turret     = new DcMotorWrapper()
      .setDcMotor(hardwareMap.dcMotor.get("Turret"), false);
    
    // Get servos
    this.leftArm  = new ServoWrapper()
      .setServo(hardwareMap.servo.get("LeftArm"))
      .setLowerBound(1.0 - armLowerBound)
      .setUpperBound(1.0 - armUpperBound);
    this.rightArm = new ServoWrapper()
      .setServo(hardwareMap.servo.get("RightArm"))
      .setLowerBound(armLowerBound)
      .setUpperBound(armUpperBound);
    
    // Get digital sensors
    this.leftTouchSensor  = hardwareMap.touchSensor.get("LeftTouchSensor");
    this.rightTouchSensor = hardwareMap.touchSensor.get("RightTouchSensor");
    
    telemetry.addData("Status", "Initialized Expansion Hub");
    telemetry.update();
  }

  private void initGamepads() {
    telemetry.addData("Status", "Initializing Gamepads");
    telemetry.update();
    
    // Get gamepad1
    this.gamepad1Wrapper = new GamepadWrapper()
      .setGamepad(gamepad1);
    
    this.gamepad1Wrapper.subscribeXPressedEvent(() -> {
      this.linearServoPosition = (this.linearServoPosition + 1) % this.linearServoPositions.length;
      this.linearServo.setPosition(this.linearServoPositions[this.linearServoPosition]);
      return false;
    });
    
    this.gamepad1Wrapper.subscribeYPressedEvent(() -> {
      this.depositPosition = (this.depositPosition + 1) % this.depositPositions.length;
      this.deposit.setPosition(this.depositPositions[this.depositPosition]);
      return false;
    });
    
    this.gamepad1Wrapper.subscribeAPressedEvent(() -> {
      this.armPosition = (this.armPosition + 1) % this.armPositions.length;
      this.leftArm.setPosition(this.armPositions[this.armPosition]);
      this.rightArm.setPosition(this.armPositions[this.armPosition]);
      return false;
    });
    
    this.gamepad1Wrapper.subscribeBPressedEvent(() -> {
      this.clawPosition = (this.clawPosition + 1) % this.clawPositions.length;
      this.clawServo1.setPosition(this.clawPositions[this.clawPosition]);
      this.clawServo2.setPosition(this.clawPositions[this.clawPosition]);
      return false;
    });
    
    this.gamepad1Wrapper.subscribeDPressedEvent(() -> {
      this.frontArmPosition = (this.frontArmPosition + 1) % this.frontArmPositions.length;
      this.frontArm.setPosition(this.frontArmPositions[this.frontArmPosition]);
      return false;
    });
    
    this.gamepad1Wrapper.subscribeUPressedEvent(() -> {
      this.linSlidePosition = (this.linSlidePosition + 1) % this.linSlidePositions.length;
      this.linSlide.setPosition(this.linSlidePositions[this.linSlidePosition]);
      return false;
    });

    telemetry.addData("Status", "Initialized Gamepads");
    telemetry.update();
  }

  private void initPIDs() {
    telemetry.addData("Status", "Initializing PIDs");
    telemetry.update();

    telemetry.addData("Status", "Initialized PIDs");
    telemetry.update();
  }

  private void initDrivetrain() {
    telemetry.addData("Status", "Initializing Drivetrain");
    telemetry.update();

    // Create Mecanum wrapper
    this.mecanumWrapper = new MecanumWrapper()
      .setFrontLeft(hardwareMap.dcMotor.get("FrontLeft"))
      .setFrontRight(hardwareMap.dcMotor.get("FrontRight"))
      .setBackLeft(hardwareMap.dcMotor.get("BackLeft"))
      .setBackRight(hardwareMap.dcMotor.get("BackRight"));
    
    // Reverse FrontLeft and BackLeft
    this.mecanumWrapper
      .getFrontLeft()
      .setDirection(DcMotor.Direction.REVERSE);
    this.mecanumWrapper
      .getBackLeft()
      .setDirection(DcMotor.Direction.REVERSE);
    
    // Create Odometry wrapper
    this.odometryWrapper = new OdometryWrapper()
      .setLeftEncoder(hardwareMap.dcMotor.get("BackLeft"))
      .setRightEncoder(hardwareMap.dcMotor.get("FrontRight"))
      .setFrontEncoder(hardwareMap.dcMotor.get("BackRight"))
      .setTrackWidth(this.trackWidth * inchesToTicks)
      .setForwardOffset(this.forwardOffset * inchesToTicks);
    
    // Reverse leftEncoder
    this.odometryWrapper
      .getLeftEncoder()
      .setDirection(DcMotor.Direction.REVERSE);
    
    telemetry.addData("Status", "Initialized Drivetrain");
    telemetry.update();
  }

  private void initMeta() {
    telemetry.addData("Status", "Initializing Meta");
    telemetry.update();

    // Create timeManager
    this.timeManager = new TimeManager()
      .setOpMode(this);
    
    // Set timeManager
    DcMotorWrapper.setTimeManager(this.timeManager);
    ServoWrapper.setTimeManager(this.timeManager);
    PIDWrapper.setTimeManager(this.timeManager);

    telemetry.addData("Status", "Initialized Meta");
    telemetry.update();
  }

  private void initProcesses() {
    telemetry.addData("Status", "Initializing Processes");
    telemetry.update();

    telemetry.addData("Status", "Initialized Processes");
    telemetry.update();
  }

  private void initPositions() {
    telemetry.addData("Status", "Initializing Positions");
    telemetry.update();

    // Initialize Control Hub motor positions
    this.linSlide.setPosition(this.linSlidePositions[this.linSlidePosition]);

    // Initialize Control Hub servo positions
    this.deposit.setPosition(this.depositPositions[this.depositPosition]);
    this.clawServo1.setPosition(this.clawPositions[this.clawPosition]);
    this.clawServo2.setPosition(this.clawPositions[this.clawPosition]);
    this.frontArm.setPosition(this.frontArmPositions[this.frontArmPosition]);
    
    // Initialize Expansion Hub servo positions
    this.leftArm.setPosition(this.armPositions[this.armPosition]);
    this.rightArm.setPosition(this.armPositions[this.armPosition]);

    telemetry.addData("Status", "Initialized Positions");
    telemetry.update();
  }
  
  private void initAll() {
    telemetry.addData("Status", "Initializing all");
    telemetry.update();

    // Initialize everything
    this.initControlHub();
    this.initExpansionHub();
    this.initGamepads();
    this.initPIDs();
    this.initDrivetrain();
    this.initMeta();
    this.initProcesses();
    this.initPositions();
    
    telemetry.addData("Status", "Initialized all");
    telemetry.update();
  }
  
  private void updateAll() {
    // Control Hub motors
    this.linSlide.update();
  
    // Control Hub servos
    this.deposit.update();
    this.linearServo.update();
    this.clawServo1.update();
    this.clawServo2.update();
    this.frontArm.update();
    
    // Expansion Hub motors
    this.turret.update();
    
    // Expansion Hub servos
    this.rightArm.update();
    this.leftArm.update();

    // Gamepads update
    this.gamepad1Wrapper.update();

    // Drivetrain update
    this.mecanumWrapper.update();
    this.odometryWrapper.update();

    // Meta update
    this.timeManager.update();
  }
  
  private void interact() {
    this.mecanumWrapper.setPowerX(0.7 * gamepad1.left_stick_x);
    this.mecanumWrapper.setPowerY(-0.7 * gamepad1.left_stick_y * 1.1);
    this.mecanumWrapper.setPowerR(-0.5 * gamepad1.right_stick_x);
    this.turret.getDcMotor().setPower(0.7 * (gamepad1.left_trigger - gamepad1.right_trigger));
  }

  private void displayStats() {
    telemetry.addData("left", this.odometryWrapper.getLeftEncoderPosition());
    telemetry.addData("right", this.odometryWrapper.getRightEncoderPosition());
    telemetry.addData("front", this.odometryWrapper.getFrontEncoderPosition());
    telemetry.addData("X", this.odometryWrapper.getX() / this.inchesToTicks);
    telemetry.addData("Y", this.odometryWrapper.getY() / this.inchesToTicks);
    telemetry.addData("R", this.odometryWrapper.getR() / this.degreesToTicks);
    telemetry.addData("LinSlide raw encoder position", this.linSlide.getDcMotor().getCurrentPosition());
    telemetry.update();
  }
  
  @Override
  public void runOpMode() throws InterruptedException {
    this.initAll();
    waitForStart();
    if (isStopRequested()) return;
    while (opModeIsActive()) {
      this.displayStats();
      this.interact();
      this.updateAll();
    }
  }
}