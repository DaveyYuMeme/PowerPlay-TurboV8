package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class PowerPlayV2 extends LinearOpMode {
  // Control Hub Motors
  private DcMotorWrapper frontLeft;
  private DcMotorWrapper linSlideUpper;
  private DcMotorWrapper linSlideLower;

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

  // PID Wrappers
  private PIDWrapper linSlidePID;

  // Meta
  private TimeManager timeManager;

  // Drivetrain Wrappers
  private MecanumWrapper mecanumWrapper;
  private OdometryWrapper odometryWrapper;

  // Control Constants
  private double armLowerBound = 0.0;
  private double armUpperBound = 1.0;
  private int linSlideLowerBound = 0;
  private int linSlideUpperBound = 500;
  private double linSlidePower = 0.2;

  // PID constants
  private double linSlideKp = 0.5;
  private double linSlideKi = 0.5;
  private double linSlideKd = 0.5;

  // Odometry Constants
  private double inchesToTicks = 2000.0;
  private double trackWidth = 11.25;
  private double forwardOffset = 5.35;

  // LinSlide toggles
  private double[] linSlidePositions = { 0.0, 1.0 };
  private int linSlidePosition;
  
  private void initControlHub() {
    telemetry.addData("Status", "Initializing Control Hub");
    telemetry.update();
    
    // Get motors
    this.linSlideUpper = new DcMotorWrapper()
      .setDcMotor(hardwareMap.dcMotor.get("LinSlideUpper"))
      .setLowerBound(this.linSlideLowerBound)
      .setUpperBound(this.linSlideUpperBound)
      .setPower(this.linSlidePower);
    this.linSlideLower = new DcMotorWrapper()
      .setDcMotor(hardwareMap.dcMotor.get("LinSlideLower"))
      .setLowerBound(-this.linSlideLowerBound)
      .setUpperBound(-this.linSlideUpperBound)
      .setPower(this.linSlidePower);

    // Get servos
    this.deposit     = new ServoWrapper()
      .setServo(hardwareMap.servo.get("Deposit"));
    this.linearServo = new ServoWrapper()
      .setServo(hardwareMap.servo.get("LinearServo"));
    this.clawServo2  = new ServoWrapper()
      .setServo(hardwareMap.servo.get("ClawServo2"));
    this.clawServo1  = new ServoWrapper()
      .setServo(hardwareMap.servo.get("ClawServo1"));
    this.frontArm    = new ServoWrapper()
      .setServo(hardwareMap.servo.get("FrontArm"));
    
    // Configure lagging motor
    this.linSlideLower
      .getDcMotor()
      .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    this.linSlideLower
      .getDcMotor()
      .setDirection(DcMotor.Direction.REVERSE);
    
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

    telemetry.addData("Status", "Initialized Gamepads");
    telemetry.update();
  }

  private void initPIDs() {
    telemetry.addData("Status", "Initializing PIDs");
    telemetry.update();

    // Create linSlidePID
    this.linSlidePID = new PIDWrapper()
      .setKp(linSlideKp)
      .setKi(linSlideKi)
      .setKd(linSlideKd)
      .setErrorFunction(() -> {
        int targetPosition = -linSlideLower.getPosition();
        int currentPosition = linSlideUpper.getPosition();
        return targetPosition - currentPosition;
      })
      .setResponseFunction((double factor) -> {
        linSlideUpper
          .getDcMotor()
          .setPower(factor);
      });

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
      .getFrontRight()
      .setDirection(DcMotor.Direction.REVERSE);
    
    // Create Odometry wrapper
    this.odometryWrapper = new OdometryWrapper()
      .setLeftEncoder(hardwareMap.dcMotor.get("BackLeft"))
      .setRightEncoder(hardwareMap.dcMotor.get("FrontRight"))
      .setFrontEncoder(hardwareMap.dcMotor.get("BackRight"))
      .setTrackWidth(this.trackWidth * inchesToTicks)
      .setForwardOffset(this.forwardOffset * inchesToTicks);
    
    // Reverse LeftEncoder
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

    this.gamepad1Wrapper.subscribeXPressedEvent(() -> {
      this.linSlidePosition = (this.linSlidePosition + 1) % this.linSlidePositions.length;
      this.linSlideLower.setPosition(this.linSlidePositions[this.linSlidePosition]);
      return false;
    });

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
    this.linSlideUpper.update();
    this.linSlideLower.update();
  
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

    // PIDs update
    this.linSlidePID.update();

    // Drivetrain update
    this.mecanumWrapper.update();
    this.odometryWrapper.update();

    // Meta update
    this.timeManager.update();
  }
  
  private void interact() {
    this.mecanumWrapper.setPowerX(-0.7 * gamepad1.left_stick_y);
    this.mecanumWrapper.setPowerY(0.7 * gamepad1.left_stick_x * 1.1);
    this.mecanumWrapper.setPowerR(0.5 * gamepad1.right_stick_x);
  }

  private void displayStats() {
    telemetry.addData("LinSlideLower position", this.linSlideLower.getPosition());
    telemetry.addData("LinSlideUpper position", this.linSlideUpper.getPosition());
  }
  
  @Override
  public void runOpMode() throws InterruptedException {
    this.initAll();
    waitForStart();

    if (isStopRequested()) return;
    
    while (opModeIsActive()) {
      this.updateAll();
      this.displayStats();
      this.interact();
      telemetry.update();
    }
  }
}