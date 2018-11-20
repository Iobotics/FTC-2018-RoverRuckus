package org.firstinspires.ftc.teamShane;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.sql.Time;

public class PIDControl {
    private double pCoeff;
    private double iCoeff;
    private double dCoeff;

    private double feedForward;
    private double errorSource;
    private double target;
    private double error = 0;

    private double integral;
    private double previousTimeI = 0;
    private double currentTimeI;

    private double previousTimeD = 0;
    private double currentTimeD;
    private double previousError = getError();

    private ElapsedTime timer = new ElapsedTime();

    public PIDControl(double target, double errorSource, double pCoeff, double iCoeff, double dCoeff, double feedForward){
        this.errorSource = errorSource;
        this.pCoeff = pCoeff;
        this.iCoeff = iCoeff;
        this.dCoeff = dCoeff;
        this.feedForward = feedForward;
        this.target = target;
    }
    public PIDControl(double target, double errorSource, double pCoeff, double iCoeff, double dCoeff){
        this.errorSource = errorSource;
        this.pCoeff = pCoeff;
        this.iCoeff = iCoeff;
        this.dCoeff = dCoeff;
        this.feedForward = 0;
        this.target = target;
    }
    public PIDControl(double target, double errorSource, double pCoeff, double iCoeff){
        this.errorSource = errorSource;
        this.pCoeff = pCoeff;
        this.iCoeff = iCoeff;
        this.dCoeff = 0;
        this.feedForward = 0;
        this.target = target;
    }
    public PIDControl(double target, double errorSource, double pCoeff){
        this.errorSource = errorSource;
        this.pCoeff = pCoeff;
        this.iCoeff = 0;
        this.dCoeff = 0;
        this.feedForward = 0;
        this.target = target;
    }

    public double getError(){
        return target - errorSource;
    }
    private double getPValue(double error, double pCoeff){
        return error * pCoeff;
    }

    private double getIntegral(double error){
        currentTimeI = timer.seconds();
        integral += (currentTimeI - previousTimeI)* error;
        previousTimeI = currentTimeI;
        return integral;
    }

    private double getIValue(double error, double iCoeff){
        return getIntegral(error) * iCoeff;
    }

    private double getDerivative(double error){
        currentTimeD = timer.seconds();
        double derivative =  (previousError - error) / (previousTimeD - currentTimeD);
        previousTimeD = currentTimeD;
        previousError = error;
        return derivative;
    }

    private double getDValue(double error, double pCoeff){
        return getDerivative(error) * pCoeff;
    }

    public double getValue(){
        error = getError();
        return feedForward + getPValue(error, pCoeff) + getIValue(error, iCoeff) + getDValue(error, pCoeff);
    }
}
