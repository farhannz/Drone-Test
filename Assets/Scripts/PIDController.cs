using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Proportion - Integrator - Derivative Controller
/// </summary>
public class PIDController
{
    //float error = 0f;
    float errorPrev = 0f;
    float errorSum = 0f;

    // float firstPeak = 0f;
    // float secondPeak = 0f;
    public float antiWindUpCutOff = 40f;
    public float Kp = 0f;
    public float Ki = 0f;
    public float Kd = 0f;

    public float GetOutputPID(float error)
    {
        float output = CalculateOutputPID(error);

        return output;
    }
    public float GetOutputPID(float Kp, float Ki, float Kd, float error)
    {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        float output = CalculateOutputPID(error);

        return output;
    }
    public float GetOutputPID(Vector3 gains, float error)
    {
        this.Kp = gains.x;
        this.Ki = gains.y;
        this.Kd = gains.z;
        float output = CalculateOutputPID(error);

        return output;
    }

    public float GetOutputPID(Vector4 gains, float error)
    {
        this.Kp = gains.x;
        this.Ki = gains.y;
        this.Kd = gains.z;
        this.antiWindUpCutOff = gains.w;
        float output = CalculateOutputPID(error);

        return output;
    }
    private float CalculateOutputPID(float error)
    {
        float output = 0f;



        // P - Proportional
        output += Kp * error;

        // I - Integral

        errorSum += Time.fixedDeltaTime * error;

        errorSum = Mathf.Clamp(errorSum, -antiWindUpCutOff, antiWindUpCutOff);

        output += Ki * errorSum;

        // D - Derivative

        float derivativeError = (error - errorPrev) / Time.fixedDeltaTime;
        output += derivativeError * Kd;

        errorPrev = error;

        return output;
    }


    public float AntiWindUpCutOff
    {
        get{return antiWindUpCutOff;}
        set{antiWindUpCutOff = value;}
    }


}
