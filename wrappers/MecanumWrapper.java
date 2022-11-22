package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MecanumWrapper {
  private DcMotor _frontLeft;
  private DcMotor _frontRight;
  private DcMotor _backLeft;
  private DcMotor _backRight;
  private double _powerX;
  private double _powerY;
  private double _powerR;

  public MecanumWrapper() {}

  public MecanumWrapper setFrontLeft(DcMotor frontLeft) {
    this._frontLeft = frontLeft;
    return this;
  }

  public MecanumWrapper setFrontRight(DcMotor frontRight) {
    this._frontRight = frontRight;
    return this;
  }

  public MecanumWrapper setBackLeft(DcMotor backLeft) {
    this._backLeft = backLeft;
    return this;
  }

  public MecanumWrapper setBackRight(DcMotor backRight) {
    this._backRight = backRight;
    return this;
  }

  public MecanumWrapper setPowerX(double power) {
    this._powerX = power;
    return this;
  }

  public MecanumWrapper setPowerY(double power) {
    this._powerY = power;
    return this;
  }

  public MecanumWrapper setPowerR(double power) {
    this._powerR = power;
    return this;
  }

  public MecanumWrapper addPowerX(double power) {
    this._powerX += power;
    return this;
  }

  public MecanumWrapper addPowerY(double power) {
    this._powerY += power;
    return this;
  }

  public MecanumWrapper addPowerR(double power) {
    this._powerR += power;
    return this;
  }

  public DcMotor getFrontLeft() {
    return this._frontLeft;
  }

  public DcMotor getFrontRight() {
    return this._frontRight;
  }

  public DcMotor getBackLeft() {
    return this._backLeft;
  }

  public DcMotor getBackRight() {
    return this._backRight;
  }

  public double getPowerX() {
    return this._powerX;
  }

  public double getPowerY() {
    return this._powerY;
  }

  public double getPowerR() {
    return this._powerR;
  }

  public void update() {
    double denominator = Math.max(Math.abs(this._powerX) + Math.abs(this._powerY) + Math.abs(this._powerR), 1.0);
    double frontLeftPower = (this._powerY + this._powerX + this._powerR) / denominator;
    double backLeftPower = (this._powerY - this._powerX + this._powerR) / denominator;
    double frontRightPower = (this._powerY - this._powerX - this._powerR) / denominator;
    double backRightPower = (this._powerY + this._powerX - this._powerR) / denominator;
    this._frontLeft.setPower(frontLeftPower);
    this._backLeft.setPower(backLeftPower);
    this._frontRight.setPower(frontRightPower);
    this._backRight.setPower(backRightPower);
    this._powerX = 0.0;
    this._powerY = 0.0;
    this._powerR = 0.0;
  }
}