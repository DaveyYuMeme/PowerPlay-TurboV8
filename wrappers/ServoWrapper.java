package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Async;
import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.managers.TimeManager;
import org.firstinspires.ftc.teamcode.utils.MathUtil;

public class ServoWrapper {
  private static TimeManager timeManager;

  private Servo _servo;
  private double _position = 0.0;
  private double _lowerBound = 0.0;
  private double _upperBound = 1.0;
  private boolean _isAsync = false;
  
  public ServoWrapper() {}

  public static void setTimeManager(TimeManager timeManager) {
    ServoWrapper.timeManager = timeManager;
  }
  
  public ServoWrapper setServo(Servo servo) {
    this._servo = servo;
    return this;
  }
  
  public ServoWrapper setLowerBound(double lowerBound) {
    this._lowerBound = lowerBound;
    return this;
  }
  
  public ServoWrapper setUpperBound(double upperBound) {
    this._upperBound = upperBound;
    return this;
  }

  public ServoWrapper setRange(double lowerBound, double upperBound) {
    this._lowerBound = lowerBound;
    this._upperBound = upperBound;
    this._servo.scaleRange(this._lowerBound, this._upperBound);
    return this;
  }

  public ServoWrapper setPosition(double weight) {
    double targetPosition = MathUtil.applyWeight(this._lowerBound, this._upperBound, weight);
    this._servo.setPosition(targetPosition);
    return this;
  }

  public Async.AsyncBody gotoPosition(double weight, double time) {
    return async -> {
      this.setPosition(weight);
      this._isAsync = true;
      ServoWrapper.timeManager.subscribeTimeEvent(time, () -> {
        this._isAsync = false;
        async.finish();
      });
    };
  }

  public Servo getServo() {
    return this._servo;
  }

  public double getLowerBound() {
    return this._lowerBound;
  }

  public double getUpperBound() {
    return this._upperBound;
  }

  public boolean isAsync() {
    return this._isAsync;
  }

  public double getPosition() {
    return this._position;
  }

  public void update() {}
}