package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.managers.TimeManager;
import org.firstinspires.ftc.teamcode.wrappers.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.wrappers.GamepadWrapper;
import org.firstinspires.ftc.teamcode.wrappers.MecanumWrapper;
import org.firstinspires.ftc.teamcode.wrappers.OdometryWrapper;
import org.firstinspires.ftc.teamcode.wrappers.PIDWrapper;
import org.firstinspires.ftc.teamcode.wrappers.ServoWrapper;

@TeleOp
public class PowerPlayV4 extends LinearOpMode {

  // Control Hub Motors
  public DcMotorWrapper frontLeft;
  public DcMotorWrapper linSlide;

  // Control Hub Servos
  public ServoWrapper deposit;
  public ServoWrapper linearServo;
  //private ServoWrapper clawServo2;
  public ServoWrapper clawServo;
  public ServoWrapper frontArm;

  // Expansion Hub Motors
  public DcMotorWrapper frontRight;
  public DcMotorWrapper backRight;
  public DcMotorWrapper backLeft;
  public DcMotorWrapper turret;

  // Expansion Hub Servos
  public ServoWrapper rightArm;
  public ServoWrapper leftArm;

  // Expansion Hub Digital Sensors
  public TouchSensor leftTouchSensor;
  public TouchSensor rightTouchSensor;

  // Gamepad Wrappers
  public GamepadWrapper gamepad1Wrapper;
  public GamepadWrapper gamepad2Wrapper;

  // Meta
  public TimeManager timeManager;

  // Drivetrain Wrappers
  public MecanumWrapper mecanumWrapper;
  public OdometryWrapper odometryWrapper;

  // Control Parameters
  public double armLowerBound = 0.5;
  public double armUpperBound = 0.82;
  public double clawLowerBound = 1;
  public double clawUpperBound = 0.5;
  public double frontArmLowerBound = 0.04;
  public double frontArmUpperBound = 0.82;
  public int linSlideLowerBound = 0;
  public int linSlideUpperBound = 1130;
  public double linSlidePower = 0.8;
  public double depositLowerBound = 0.5;
  public double depositUpperBound = 0.9;
  public int turretLowerBound = 0;
  public int turretUpperBound = 100;
  public double linearServoLowerBound = 0.1;
  public double linearServoUpperBound = 0.9;

  public double frontLeftPower = 0;
  public double backLeftPower = 0;
  public double frontRightPower = 0;
  public double backRightPower = 0;

  // Odometry Parameters
  public double inchesToTicks = 1901.86;
  public double degreesToTicks = 100;
  public double trackWidth = 11.25;
  public double forwardOffset = 5.35;

  // Position Parameters
  public double[] armPositions = { 0.0, 0.5, 1.0 };
  public int armPosition = 0;
  public double[] clawPositions = { 0.0, 1.0 };
  public int clawPosition = 0;
  public double[] frontArmPositions = { 0.0, 1.0 };
  public int frontArmPosition = 0;
  public double[] linSlidePositions = { 0.0, 0.33, 0.67, 1.0 };
  public int linSlidePosition = 0;
  public double[] depositPositions = { 0.0, 0.5, 1.0 };
  public int depositPosition = 0;
  public double[] linearServoPositions = { 0.0, 1.0 };
  public int linearServoPosition = 0;

  //other funnies
  boolean IntakeOn = false;
  double speedScale = 1;
  boolean ClawOpen = false;

  //Async Functions
  Async LinSlideUp;
  Async LinSlideDown;
  Async DepositDown;
  Async DepositUp;
  Async DepositDump;

  Async TurretCenter;
  Async TurretToLastPos;

  // Async async = new Async(async ->{});

  Async clawOpen = new Async(async -> {
    clawServo.setPosition(clawUpperBound);
    async.finish();
  });

  Async clawClose  = new Async(async -> {
    clawServo.setPosition(clawLowerBound);
    async.finish();
  });

  Async IntakeUp = new Async(async ->{
    frontArm.setPosition(frontArmUpperBound);//test
    async.finish();
  });

  Async IntakeDown = new Async(async ->{
    frontArm.setPosition(frontArmLowerBound);//test
    async.finish();
  });

  Async IntakeIn = new Async(async ->{
    leftArm.setPosition(armLowerBound);
    rightArm.setPosition(armUpperBound);//test
    async.finish();
  });

  Async IntakeOut = new Async(async ->{
    leftArm.setPosition(armUpperBound);
    rightArm.setPosition(armLowerBound);//test
    async.finish();
  });

  Async IntakeMode = new Async(async -> {
    clawOpen.execute();
    IntakeUp.execute();
    async.finish();
  });

  Async Intake = new Async(async -> {

    //ethan im kinda screwed here can you like do this for me
    async.finish();
  });

  private void initControlHub() {
    telemetry.addData("Status", "Initializing Control Hub");
    telemetry.update();
    
    // Get motors
    this.linSlide = new DcMotorWrapper()
      .setDcMotor(hardwareMap.dcMotor.get("LinSlide"))
      .setLowerBound(this.linSlideLowerBound)
      .setUpperBound(this.linSlideUpperBound)
      .setPower(this.linSlidePower);

    this.frontLeft = new DcMotorWrapper()
            .setDcMotor(hardwareMap.dcMotor.get("FrontLeft"), false);
    this.backRight = new DcMotorWrapper()
            .setDcMotor(hardwareMap.dcMotor.get("BackRight"), false);

    // Get servos
    this.deposit     = new ServoWrapper()
      .setServo(hardwareMap.servo.get("Deposit"))
      .setLowerBound(this.depositLowerBound)
      .setUpperBound(this.depositUpperBound);
    this.linearServo = new ServoWrapper()
      .setServo(hardwareMap.servo.get("LinearServo"))
      .setLowerBound(this.linearServoLowerBound)
      .setUpperBound(this.linearServoUpperBound);
    this.clawServo  = new ServoWrapper()
      .setServo(hardwareMap.servo.get("ClawServo1"))
      .setLowerBound(1.0 - this.clawLowerBound)
      .setUpperBound(1.0 - this.clawUpperBound);
    this.frontArm    = new ServoWrapper()
      .setServo(hardwareMap.servo.get("FrontArm"));

//      .setLowerBound(this.frontArmLowerBound)
//      .setUpperBound(this.frontArmUpperBound);
    
    telemetry.addData("Status", "Initialized Control Hub");
    telemetry.update();
  }
  
  private void initExpansionHub() {
    telemetry.addData("Status", "Initializing Expansion Hub");
    telemetry.update();
    
    // Get motors
    this.turret     = new DcMotorWrapper()
      .setDcMotor(hardwareMap.dcMotor.get("Turret"), false);

    this.frontRight = new DcMotorWrapper()
            .setDcMotor(hardwareMap.dcMotor.get("FrontRight"));
    this.backLeft= new DcMotorWrapper()
            .setDcMotor(hardwareMap.dcMotor.get("BackLeft"));
    
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

    
    this.gamepad1Wrapper.subscribeDPressedEvent(() -> {
        IntakeIn.execute();
        return false;
    });
    
    this.gamepad1Wrapper.subscribeUPressedEvent(() -> {
        IntakeDown.execute();
      return false;
    });



    // Gamepad 2
//    this.gamepad2Wrapper = new GamepadWrapper()
//            .setGamepad(gamepad2);
//
//    this.gamepad2Wrapper.subscribeXPressedEvent(() -> {
//      this.linearServoPosition = (this.linearServoPosition + 1) % this.linearServoPositions.length;
//      this.linearServo.setPosition(this.linearServoPositions[this.linearServoPosition]);
//      return false;
//    });
//
//    this.gamepad2Wrapper.subscribeYPressedEvent(() -> {
//      this.depositPosition = (this.depositPosition + 1) % this.depositPositions.length;
//      this.deposit.setPosition(this.depositPositions[this.depositPosition]);
//      return false;
//    });
//
//    this.gamepad2Wrapper.subscribeAPressedEvent(() -> {
//      this.armPosition = (this.armPosition + 1) % this.armPositions.length;
//      this.leftArm.setPosition(this.armPositions[this.armPosition]);
//      this.rightArm.setPosition(this.armPositions[this.armPosition]);
//      return false;
//    });



//    this.gamepad2Wrapper.subscribeBPressedEvent(() -> {
//      this.clawPosition = (this.clawPosition + 1) % this.clawPositions.length;
//      this.clawServo.setPosition(this.clawPositions[this.clawPosition]);
//      return false;
//    });
//
//    this.gamepad2Wrapper.subscribeDPressedEvent(() -> {
//      this.frontArmPosition = (this.frontArmPosition + 1) % this.frontArmPositions.length;
//      this.frontArm.setPosition(this.frontArmPositions[this.frontArmPosition]);
//      return false;
//    });
//
//    this.gamepad2Wrapper.subscribeUPressedEvent(() -> {
//      this.linSlidePosition = (this.linSlidePosition + 1) % this.linSlidePositions.length;
//      this.linSlide.setPosition(this.linSlidePositions[this.linSlidePosition]);
//      return false;
//    });

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
    this.clawServo.setPosition(this.clawPositions[this.clawPosition]);
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

    //Insert Initalize Positions Here
    leftArm.setPosition(0.5);
    rightArm.setPosition(0.5);
    
    telemetry.addData("Status", "Initialized all");
    telemetry.update();
  }
  
  private void updateAll() {
    // Control Hub motors
    this.linSlide.update();
  
    // Control Hub servos
    this.deposit.update();
    this.linearServo.update();
    this.clawServo.update();
//    this.clawServo2.update();
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
    this.turret.getDcMotor().setPower(0.7 * (gamepad2.left_trigger - gamepad2.right_trigger));
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

  private void controls() {
//    if (gamepad1.b) {
//
//    }
    if (gamepad1.x) {
      if(IntakeOn){
        IntakeOut.execute();
        IntakeOn = true;
      }
      else{
        IntakeIn.execute();
        IntakeOn = false;
      }
    }

//    if (gamepad1.y) {
//      leftArm.setPosition(1-armUpperBound);
//      rightArm.setPosition(armUpperBound);
//    }

    if (gamepad1.dpad_right){
      speedScale += 0.05;
    }
    if (gamepad1.dpad_left){
      speedScale -= 0.05;
    }

    frontLeftPower = gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
    backLeftPower = gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
    backRightPower = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
    frontRightPower = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;

    frontLeft.setPower(frontLeftPower * speedScale);
    backLeft.setPower(backLeftPower * speedScale);
    frontRight.setPower(frontRightPower * speedScale);
    backRight.setPower(backRightPower * speedScale);

    //frontArm.setPosition(frontArm.getPosition() + (gamepad1.right_trigger - gamepad1.left_trigger) * 0.6);
    leftArm.setPosition(leftArm.getPosition() + gamepad1.right_trigger - gamepad1.left_trigger);
    rightArm.setPosition(rightArm.getPosition() - gamepad1.right_trigger + gamepad1.left_trigger);

    if (gamepad1.right_bumper) {
      if(ClawOpen){
        clawClose.execute();
        ClawOpen = false;
      }
      else{
        clawOpen.execute();
        ClawOpen = true;
      }

    }
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
      this.controls();
    }
  }
}