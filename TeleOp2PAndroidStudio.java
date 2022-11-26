package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.managers.TimeManager;
import org.firstinspires.ftc.teamcode.wrappers.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.wrappers.GamepadWrapper;
import org.firstinspires.ftc.teamcode.wrappers.MecanumWrapper;
import org.firstinspires.ftc.teamcode.wrappers.OdometryWrapper;
import org.firstinspires.ftc.teamcode.wrappers.PIDWrapper;
import org.firstinspires.ftc.teamcode.wrappers.ServoWrapper;

@TeleOp
public class TeleOp2PAndroidStudio extends LinearOpMode {
	// Control Hub Motors
	private DcMotorWrapper frontLeft;
	private DcMotorWrapper linSlide;

	// Control Hub Servos
	private ServoWrapper deposit;
	private ServoWrapper linearServo;
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
	private GamepadWrapper gamepad2Wrapper;

	// Meta
	private TimeManager timeManager;
	
	//IMU
	private BNO055IMU imu;

	// Drivetrain Wrappers
	private MecanumWrapper mecanumWrapper;
	private OdometryWrapper odometryWrapper;

	// Control Parameters
	private double armLowerBound = 0.5;
	private double armUpperBound = 0.82;
	private double clawLowerBound = 0.55;
	private double clawUpperBound = 0.85;
	private double frontArmLowerBound = 0.04;
	private double frontArmUpperBound = 0.85;
	private int linSlideLowerBound = 0;
	private int linSlideUpperBound = 1150;
	private double linSlidePower = 1;
	private double depositLowerBound = 0.53;
	private double depositUpperBound = 0.98;
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
	private double[] armPositions = { 0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1};
	private int armPosition = 5;
	private double[] clawPositions = { 0.0, 1.0 };
	private int clawPosition = 0;
	private double[] frontArmPositions = { 1, 0.0 };
	private int frontArmPosition = 0;
	private double[] linSlidePositions = { 0.0, 0.7, 0.92 };
	private int linSlidePosition = 0;
	private double[] depositPositions = { 0.0, 0.4, 1.0 };
	private int depositPosition = 0;
	private double[] linearServoPositions = { 0.0, 0.5, 1.0 };
	private int linearServoPosition = 0;
	private double[] frontArmDifference = {0 , 0.05, 0.1};
	private double[] linSlideFactor = {1, 0.75, 0.5};
	private double lastArmPositionFactor = 1;
	private boolean armOut = false;
	private int step = 0;
	private double linSlideOffset = 0;
	private boolean linSlideHigh = true;
	private double[] speedScales = {0.4, 0.6, 0.8, 1};
	private int speedScale = 3;
	private double frontArmOffset = 0; //for full extention
	
	//Function
	// private static void Score(int step) {
	// private static void Score(int step) {
	//	 if(step == 0){
	
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
		this.deposit		 = new ServoWrapper()
			.setServo(hardwareMap.servo.get("Deposit"))
			.setLowerBound(this.depositLowerBound)
			.setUpperBound(this.depositUpperBound);
		this.linearServo = new ServoWrapper()
			.setServo(hardwareMap.servo.get("LinearServo"))
			.setLowerBound(this.linearServoLowerBound)
			.setUpperBound(this.linearServoUpperBound);
		this.clawServo1	= new ServoWrapper()
			.setServo(hardwareMap.servo.get("ClawServo1"))
			.setLowerBound(1.0 - this.clawLowerBound)
			.setUpperBound(1.0 - this.clawUpperBound);
		this.frontArm		= new ServoWrapper()
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
		this.turret		 = new DcMotorWrapper()
			.setDcMotor(hardwareMap.dcMotor.get("Turret"), false);
		
		// Get servos
		this.leftArm	= new ServoWrapper()
			.setServo(hardwareMap.servo.get("LeftArm"))
			.setLowerBound(1.0 - armLowerBound)
			.setUpperBound(1.0 - armUpperBound);
		this.rightArm = new ServoWrapper()
			.setServo(hardwareMap.servo.get("RightArm"))
			.setLowerBound(armLowerBound)
			.setUpperBound(armUpperBound);
		
		// Get digital sensors
		this.leftTouchSensor	= hardwareMap.touchSensor.get("LeftTouchSensor");
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
			
		this.gamepad2Wrapper = new GamepadWrapper()
			.setGamepad(gamepad2);
		//gamePad 2:
		//dPad
		this.gamepad2Wrapper.subscribeLPressedEvent(() -> {
			this.linearServoPosition = 1;
			moveLinearServo();
			return false;
		});
		this.gamepad2Wrapper.subscribeUPressedEvent(() -> {
			this.linearServoPosition = 2;
			moveLinearServo();
			return false;
		});
		this.gamepad2Wrapper.subscribeDPressedEvent(() -> {
			this.linearServoPosition = 0;
			moveLinearServo();
			return false;
		});
		
		this.gamepad1Wrapper.subscribeUPressedEvent(() -> {
			this.linSlidePosition = (this.linSlidePosition + 1) % this.linSlidePositions.length;
			moveLinSlide();
			return false;
		});
		//Bumpers
		this.gamepad2Wrapper.subscribeRbPressedEvent(() -> {
		if(armPosition < 9) this.armPosition++;
			moveArm(armPosition);
			return false;
		});
		this.gamepad2Wrapper.subscribeLbPressedEvent(() -> {
		if(armPosition > 0) this.armPosition--;
			moveArm(armPosition);
			return false;
		});
		//letters
		this.gamepad2Wrapper.subscribeYPressedEvent(() -> {
			linSlideOffset += 0.01;
			moveLinSlide();
			return false;
		});
		this.gamepad2Wrapper.subscribeAPressedEvent(() -> {
			linSlideOffset -= 0.01;
			moveLinSlide();
			return false;
		});
		this.gamepad2Wrapper.subscribeBPressedEvent(() -> {
			linSlideOffset = 0;
			moveLinSlide();
			return false;
		});
		this.gamepad2Wrapper.subscribeRPressedEvent(() -> {
			this.speedScale = (this.speedScale + 1) % this.speedScales.length;
			return false;
		});
		
		//gamepad 1
		this.gamepad1Wrapper.subscribeLbPressedEvent(() -> {
			this.depositPosition = (this.depositPosition + 1) % this.depositPositions.length;
			moveDeposit();
			return false;
		});

		this.gamepad1Wrapper.subscribeAPressedEvent(() -> {
			if(!armOut){
			moveArm(armPosition);
			}else{
			moveArm(-1);
			}
			return false;
		});
		
		this.gamepad1Wrapper.subscribeRbPressedEvent(() -> {
			this.clawPosition = (this.clawPosition + 1) % this.clawPositions.length;
			moveClaw();
			return false;
		});
		
		this.gamepad1Wrapper.subscribeDPressedEvent(() -> {
			this.frontArmPosition = (this.frontArmPosition + 1) % this.frontArmPositions.length;
			moveFrontArm();
			return false;
		});
		
		this.gamepad1Wrapper.subscribeBPressedEvent(() -> {
			if(step == 0){
				intakeOut();
				resetLinSlide();
				step++;
			}else if(step == 1){
				intakeBack();
				step++;
			}else if(step == 2){
				linSlideUp();
				step++;
			}else{
				dump();
				step = 0;
			}
			return false;
		});

		telemetry.addData("Status", "Initialized Gamepads");
		telemetry.update();
	}

	private void initPIDs() {
		telemetry.addData("Status", "Initializing PIDs");
		telemetry.update();
		
		imu = hardwareMap.get(BNO055IMU.class, "imu");
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
		imu.initialize(parameters);

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
			.setRightEncoder(hardwareMap.dcMotor.get("BackRight"))
			.setFrontEncoder(hardwareMap.dcMotor.get("FrontEncoder"))
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
		this.frontArm.setPosition(this.frontArmPositions[this.frontArmPosition]);
		
		// Initialize Expansion Hub servo positions
		this.leftArm.setPosition(0);
		this.rightArm.setPosition(0);

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
		this.frontArm.update();
		
		// Expansion Hub motors
		this.turret.update();
		
		// Expansion Hub servos
		this.rightArm.update();
		this.leftArm.update();

		// Gamepads update
		this.gamepad1Wrapper.update();
		this.gamepad2Wrapper.update();

		// Drivetrain update
		this.mecanumWrapper.update();
		this.odometryWrapper.update();

		// Meta update
		this.timeManager.update();
	}
	
	private void interact() {
		this.mecanumWrapper.setPowerX(speedScales[speedScale] * gamepad1.left_stick_x);
		this.mecanumWrapper.setPowerY(-speedScales[speedScale] * gamepad1.left_stick_y * 1.1);
		this.mecanumWrapper.setPowerR(speedScales[speedScale] * gamepad1.right_stick_x * 0.7);
		this.turret.getDcMotor().setPower(0.7 * (gamepad2.left_stick_x));
		this.turret.getDcMotor().setPower(0.7 * (gamepad1.right_trigger - gamepad1.left_trigger));
	}

	private void displayStats() {
		telemetry.addData("left", this.odometryWrapper.getLeftEncoderPosition());
		telemetry.addData("right", this.odometryWrapper.getRightEncoderPosition());
		telemetry.addData("front", this.odometryWrapper.getFrontEncoderPosition());
		telemetry.addData("X", this.odometryWrapper.getX() / this.inchesToTicks);
		telemetry.addData("Y", this.odometryWrapper.getY() / this.inchesToTicks);
		telemetry.addData("R", this.odometryWrapper.getR() / this.degreesToTicks);
		telemetry.addData("Speed Scale ", speedScales[speedScale]);
		telemetry.addData("Arm Position ", armPositions[armPosition]);
		telemetry.addData("Linear Slide Current Offset ", linSlideOffset);
		telemetry.addData("LinSlide raw encoder position", this.linSlide.getDcMotor().getCurrentPosition());
		double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
		telemetry.addData("Angle in Degrees", angle);
		//curHeading = angles.firstAngle;
		telemetry.addLine("* Reading this doesn't seem like the best use of your time.");
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
	private void moveClaw(){
		this.clawServo1.setPosition(this.clawPositions[clawPosition]);
	} 
	private void moveLinSlide(){
		this.linSlide.setPosition(this.linSlidePositions[linSlidePosition]*linSlideFactor[linearServoPosition]+ linSlidePositions[linSlidePosition] * linSlideOffset);
	}
	private void moveArm(int position){
		if(position < 0 ||position > 9){
			this.leftArm.setPosition(0);
			this.rightArm.setPosition(0);
			armOut = false;
		}else{
			this.leftArm.setPosition(this.armPositions[position]);
			this.rightArm.setPosition(this.armPositions[position]);
			armOut = true;
		}
	}
	private void moveLinearServo(){
		this.linearServo.setPosition(this.linearServoPositions[linearServoPosition]);
	}
	private void moveDeposit(){
		this.deposit.setPosition(this.depositPositions[depositPosition]);
	}
	private void moveFrontArm(){
		if(frontArmPositions[frontArmPosition] - frontArmDifference[linearServoPosition] < 0)frontArm.setPosition(0);
		else this.frontArm.setPosition(frontArmPositions[frontArmPosition] - frontArmDifference[linearServoPosition]);
	}
	private void intakeOut(){
		frontArmPosition = 1;
		moveFrontArm();
		moveArm(armPosition);
		clawPosition = 0;
		moveClaw();
	}
	private void intakeBack(){
		clawPosition = 1; 
		moveClaw();
		//All Sleep function will be replaced by time mangers.
		sleep(250);
		frontArmPosition = 0;
		moveFrontArm();
		moveArm(-1);
	}
	private void linSlideUp(){
		clawPosition = 0;
		moveClaw();
		sleep(500);
		if(linSlideHigh) linSlidePosition = 2;
		else linSlidePosition = 1;
		moveLinSlide();
		sleep(250);
		depositPosition = 1;
		moveDeposit();
		preIntakeMode();
	}
	private void resetLinSlide(){
		if(depositPosition != 0){
			depositPosition = 0;
			moveDeposit();
			sleep(400);
		}
		linSlidePosition = 0;
		moveLinSlide();
		//Add turret center code here later!
	}
	private void preIntakeMode(){
		frontArm.setPosition(0.5*frontArmPositions[frontArmPosition]);
		moveArm(armPosition/2 - 1);
	}
	private void dump(){
		depositPosition = 2;
		moveDeposit();
	}
	private void greatReset(){
		resetLinSlide();
		intakeBack();
		step = 0;
	}
}
