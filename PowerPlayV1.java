package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class PowerPlayV1 extends LinearOpMode {
  // Control Hub Servos
  private ServoWrapper clawServoRight;
  private ServoWrapper clawServoLeft;
  private ServoWrapper leftArm;
  private ServoWrapper frontArm;
  
  // Expansion Hub Motors
  private DcMotorWrapper linSlideUpper;
  private DcMotorWrapper linSlideLower;
  
  // Expansion Hub Servos
  private ServoWrapper linearServo;
  private ServoWrapper rightArm;

  // Gamepad Wrappers
  private GamepadWrapper gamepad1Wrapper;

  // Meta
  private TimeManager timeManager;

  // Movement Wrappers
  private MecanumWrapper mecanumWrapper;
  private OdometryWrapper odometryWrapper;
  
  // Claw control limits
  private double clawLowerBound = 0.1;
  private double clawUpperBound = 0.5;
  
  // Arm control limits
  private double armLowerBound = 0.15;
  private double armUpperBound = 0.5;
  
  // Front Arm control limits
  private double frontArmLowerBound = 0.35;
  private double frontArmUpperBound = 0.8;
  
  // Linear Slide control limits
  private int linSlideLowerBound = 0;
  private int linSlideUpperBound = 930;
  private double linSlideExtendingPower = 0.9;
  private double linSlideReturningPower = 0.3;
  
  // Linear Servo control limits
  private double linearServoLowerBound = 0.1;
  private double linearServoUpperBound = 0.9;
  
  // Odometry constants
  private double trackWidth = 11.25;
  private double forwardOffset = 5.35;
  
  private void initControlHub() {
    telemetry.addData("Status", "Initializing Control Hub");
    telemetry.update();
    
    // Get servos
    this.clawServoRight = new ServoWrapper()
      .setServo(hardwareMap.servo.get("ClawServoRight"))
      .setLowerBound(clawLowerBound)
      .setUpperBound(clawUpperBound);
    this.clawServoLeft  = new ServoWrapper()
      .setServo(hardwareMap.servo.get("ClawServoLeft"))
      .setLowerBound(clawLowerBound)
      .setUpperBound(clawUpperBound);
    this.leftArm        = new ServoWrapper()
      .setServo(hardwareMap.servo.get("LeftArm"))
      .setLowerBound(armLowerBound)
      .setUpperBound(armUpperBound);
    this.frontArm       = new ServoWrapper()
      .setServo(hardwareMap.servo.get("FrontArm"))
      .setLowerBound(frontArmLowerBound)
      .setUpperBound(frontArmUpperBound);
    
    telemetry.addData("Status", "Initialized Control Hub");
    telemetry.update();
  }
  
  private void initExpansionHub() {
    telemetry.addData("Status", "Initializing Expansion Hub");
    telemetry.update();
    
    // Get motors
    this.linSlideLower = new DcMotorWrapper()
      .setDcMotor(hardwareMap.dcMotor.get("LinSlideLower"))
      .setLowerBound(linSlideLowerBound)
      .setUpperBound(linSlideUpperBound)
      .setForwardPower(linSlideExtendingPower)
      .setReversePower(linSlideReturningPower);
    this.linSlideUpper = new DcMotorWrapper()
      .setDcMotor(hardwareMap.dcMotor.get("LinSlideUpper"))
      .setLowerBound(-linSlideLowerBound)
      .setUpperBound(-linSlideUpperBound)
      .setForwardPower(linSlideExtendingPower)
      .setReversePower(linSlideReturningPower);
    
    // Get servos
    this.linearServo = new ServoWrapper()
      .setServo(hardwareMap.servo.get("LinearServo"))
      .setLowerBound(linearServoLowerBound)
      .setUpperBound(linearServoUpperBound);
    this.rightArm    = new ServoWrapper()
      .setServo(hardwareMap.servo.get("RightArm"))
      .setLowerBound(armUpperBound)
      .setUpperBound(armLowerBound);
    
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

  private void initMovement() {
    telemetry.addData("Status", "Initializing Mecanum");
    telemetry.update();

    // Create Mecanum Wrapper
    this.mecanumWrapper = new MecanumWrapper()
      .setFrontLeft(hardwareMap.dcMotor.get("FrontLeft"))
      .setFrontRight(hardwareMap.dcMotor.get("FrontRight"))
      .setBackLeft(hardwareMap.dcMotor.get("BackLeft"))
      .setBackRight(hardwareMap.dcMotor.get("BackRight"));
    
    // Create Odometry Wrapper
    this.odometryWrapper = new OdometryWrapper()
      .setLeftEncoder(hardwareMap.dcMotor.get("FrontLeft"))
      .setRightEncoder(hardwareMap.dcMotor.get("FrontRight"))
      .setBackEncoder(hardwareMap.dcMotor.get("BackLeft"))
      .setTrackWidth(this.trackWidth)
      .setForwardOffset(this.forwardOffset);
    
    telemetry.addData("Status", "Initialized Mecanum");
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

    telemetry.addData("Status", "Initialized Meta");
    telemetry.update();
  }

  private void initProcesses() {
    telemetry.addData("Status", "Initializing Processes");
    telemetry.update();

    // Create and link grab process
    Async grabProcess = new Async();
    Async frontArmDown = new Async(frontArm.gotoPosition(1.0, 1.0));
    Async clawServoLeftClose = new Async(clawServoLeft.gotoPosition(1.0, 1.0));
    Async clawServoRightClose = new Async(clawServoRight.gotoPosition(1.0, 1.0));
    Async frontArmUp = new Async(frontArm.gotoPosition(0.0, 1.0));
    Async clawServoLeftOpen = new Async(clawServoLeft.gotoPosition(0.0, 1.0));
    Async clawServoRightOpen = new Async(clawServoRight.gotoPosition(0.0, 1.0));
    Async.addLink(grabProcess, frontArmDown);
    Async.addLink(frontArmDown, clawServoLeftClose);
    Async.addLink(frontArmDown, clawServoRightClose);
    Async.addLink(clawServoLeftClose, frontArmUp);
    Async.addLink(clawServoRightClose, frontArmUp);
    Async.addLink(frontArmUp, clawServoLeftOpen);
    Async.addLink(frontArmUp, clawServoRightOpen);

    // Bind to gamepad1 x-button pressed event
    this.gamepad1Wrapper.subscribeXPressedEvent(() -> {
      grabProcess.execute();
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
    this.initMovement();
    this.initMeta();
    this.initProcesses();
    
    telemetry.addData("Status", "Initialized all");
    telemetry.update();
  }
  
  private void updateAll() {
    // Control Hub Servos
    this.clawServoRight.update();
    this.clawServoLeft.update();
    this.leftArm.update();
    
    // Expansion Hub Motors
    this.linSlideUpper.update();
    this.linSlideLower.update();
    
    // Expansion Hub Servos
    this.linearServo.update();
    this.rightArm.update();

    // Gamepads update
    this.gamepad1Wrapper.update();

    // Movement update
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
    telemetry.addData("x", this.odometryWrapper.getX());
    telemetry.addData("y", this.odometryWrapper.getY());
    telemetry.addData("z", this.odometryWrapper.getR());
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
