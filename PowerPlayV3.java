package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class PowerPlayV3 extends LinearOpMode {
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

  // Gamepad Wrappers
  private GamepadWrapper gamepad1Wrapper;

  // Meta
  private TimeManager timeManager;

  // Drivetrain Wrappers
  private MecanumWrapper mecanumWrapper;
  private OdometryWrapper odometryWrapper;

  // Control Constants
  private double armLowerBound = 0.0;
  private double armUpperBound = 1.0;
  private int linSlideLowerBound = -0;
  private int linSlideUpperBound = -600;
  private double linSlidePower = 0.2;
  private double depositLowerBound = 0.5;
  private double depositUpperBound = 0.85;

  // Odometry Constants
  private double inchesToTicks = 1901.86;
  private double degreesToTicks = 100;
  private double trackWidth = 11.25;
  private double forwardOffset = 5.35;
  
  // Toggling
  private double[] linSlidePositions = { 0.0, 0.33, 0.67, 1.0 };
  private int linSlidePosition = 0;
  private double[] depositPositions = { 0.0, 1.0 };
  private int depositPosition = 0;
  
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
      .setServo(hardwareMap.servo.get("LinearServo"));
    this.clawServo2  = new ServoWrapper()
      .setServo(hardwareMap.servo.get("ClawServo2"));
    this.clawServo1  = new ServoWrapper()
      .setServo(hardwareMap.servo.get("ClawServo1"));
    this.frontArm    = new ServoWrapper()
      .setServo(hardwareMap.servo.get("FrontArm"));
    
    telemetry.addData("Status", "Initialized Control Hub");
    telemetry.update();
  }
  
  private void initExpansionHub() {
    telemetry.addData("Status", "Initializing Expansion Hub");
    telemetry.update();
    
    // Get motors
    this.turret     = new DcMotorWrapper()
      .setDcMotor(hardwareMap.dcMotor.get("Turret"));
    
    // Get servos
    this.rightArm = new ServoWrapper()
      .setServo(hardwareMap.servo.get("RightArm"))
      .setLowerBound(armLowerBound)
      .setUpperBound(armUpperBound);
    this.leftArm  = new ServoWrapper()
      .setServo(hardwareMap.servo.get("LeftArm"))
      .setLowerBound(armLowerBound)
      .setUpperBound(armUpperBound);
    
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
      this.linSlidePosition = (this.linSlidePosition + 1) % this.linSlidePositions.length;
      this.linSlide.setPosition(this.linSlidePositions[this.linSlidePosition]);
      return false;
    });
    
    this.gamepad1Wrapper.subscribeYPressedEvent(() -> {
      this.depositPosition = (this.depositPosition + 1) % this.depositPositions.length;
      this.deposit.setPosition(this.depositPositions[this.depositPosition]);
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
    
    telemetry.addData("Status", "Initialized all");
    telemetry.update();
  }
  
  private void updateAll() {
    // Control Hub motors
    this.linSlide.update();
  
    // Control Hub servos
    this.deposit.update();
    this.linearServo.update();
    this.clawServo2.update();
    this.clawServo1.update();
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
    this.mecanumWrapper.setPowerX(-0.7 * gamepad1.left_stick_x);
    this.mecanumWrapper.setPowerY(-0.7 * gamepad1.left_stick_y * 1.1);
    this.mecanumWrapper.setPowerR(-0.5 * gamepad1.right_stick_x);
    // if (gamepad1.dpad_up) this.inchesToTicks += 10;
    // if (gamepad1.dpad_down) this.inchesToTicks -= 10;
    // if (this.odometryWrapper.getX() > -36.0) this.mecanumWrapper.setPowerX(-0.5);
  }

  private void displayStats() {
    telemetry.addData("left", this.odometryWrapper.getLeftEncoderPosition());
    telemetry.addData("right", this.odometryWrapper.getRightEncoderPosition());
    telemetry.addData("front", this.odometryWrapper.getFrontEncoderPosition());
    telemetry.addData("X", this.odometryWrapper.getX() / this.inchesToTicks);
    telemetry.addData("Y", this.odometryWrapper.getY() / this.inchesToTicks);
    telemetry.addData("R", this.odometryWrapper.getR() / this.degreesToTicks);
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