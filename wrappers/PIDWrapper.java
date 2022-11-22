package org.firstinspires.ftc.teamcode;

public class PIDWrapper {
  private static TimeManager timeManager;
  private static double EPSILON = 0.4;

  private ErrorFunction errorFunction;
  private ResponseFunction responseFunction;
  private double _kp = 1.0;
  private double _ki = 1.0;
  private double _kd = 1.0;
  private double _kv = 1.0;
  private double _ka = 1.0;
  private double _gain = 0.0;
  private double _integral = 0.0;
  private double _derivative;
  private double _error = 0.0;
  private double _errorDelta = 0.0;
  private double _errorDeltaLowpass = 0.0;
  private double _integralMax = 1e+9;

  public PIDWrapper() {}

  public interface ErrorFunction {
    public double execute();
  }

  public interface ResponseFunction {
    public void execute(double factor);
  }

  public static void setTimeManager(TimeManager timeManager) {
    PIDWrapper.timeManager = timeManager;
  }

  public PIDWrapper setErrorFunction(ErrorFunction errorFunction) {
    this.errorFunction = errorFunction;
    return this;
  }

  public PIDWrapper setResponseFunction(ResponseFunction responseFunction) {
    this.responseFunction = responseFunction;
    return this;
  }

  public PIDWrapper setKp(double kp) {
    this._kp = kp;
    return this;
  }

  public PIDWrapper setKi(double ki) {
    this._ki = ki;
    return this;
  }

  public PIDWrapper setKd(double kd) {
    this._kd = kd;
    return this;
  }

  public PIDWrapper setKv(double kv) {
    this._kv = kv;
    return this;
  }

  public PIDWrapper setKa(double ka) {
    this._ka = ka;
    return this;
  }

  public PIDWrapper setGain(double gain) {
    this._gain = gain;
    return this;
  }

  public PIDWrapper setIntegralMax(double integralMax) {
    this._integralMax = integralMax;
    return this;
  }

  public ErrorFunction getErrorFunction() {
    return this.errorFunction;
  }

  public ResponseFunction getResponseFunction() {
    return this.responseFunction;
  }

  public double getKp() {
    return this._kp;
  }

  public double getKi() {
    return this._ki;
  }

  public double getKd() {
    return this._kd;
  }

  public double getKv() {
    return this._kv;
  }

  public double getKa() {
    return this._ka;
  }

  public double getGain() {
    return this._gain;
  }

  public double getIntegralMax() {
    return this._integralMax;
  }

  public double getError() {
    return this._error;
  }

  public double getIntegral() {
    return this._integral;
  }

  public double getDerivative() {
    return this._derivative;
  }

  public double getErrorDelta() {
    return this._errorDelta;
  }

  public double getErrorDeltaLowpass() {
    return this._errorDeltaLowpass;
  }

  public void updateError() {
    double nowError = this.errorFunction.execute();
    double timeDelta = PIDWrapper.timeManager.getTimeDelta();

    // Put error delta through low-pass filter to smooth disturbances
    this._errorDelta = nowError - this._error;
    this._errorDeltaLowpass = MathUtil.applyWeight(this._errorDelta, this._errorDeltaLowpass, this._gain);

    // Prevent integral wind-up
    double localIntegral = nowError * timeDelta;
    if (localIntegral < PIDWrapper.EPSILON) this._integral = 0.0;
    this._integral = MathUtil.clamp(this._integral + localIntegral, -this._integralMax, this._integralMax);

    this._derivative = this._errorDeltaLowpass / timeDelta;
    this._error = nowError;
  }

  public void updateResponse() {
    double factor = 0.0;
    factor += this._kp * this._error;
    factor += this._ki * this._integral;
    factor += this._kd * this._derivative;
    this.responseFunction.execute(factor);
  }

  public void update() {
    this.updateError();
    this.updateResponse();
  }
}