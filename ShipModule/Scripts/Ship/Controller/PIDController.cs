using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using System;
using System.Numerics;
using System.Linq;

/// <summary>
/// A (P)roportional, (I)ntegral, (D)erivative Controller
/// </summary>
/// <remarks>
/// The controller should be able to control any process with a
/// measureable value, a known ideal value and an input to the
/// process that will affect the measured value.
/// </remarks>
/// <see cref="https://en.wikipedia.org/wiki/PID_controller"/>
public sealed class PIDController
{
    private float processVariable = 0;
    private float prev_SetPoint = 0;
    private List<float> Oscillation = new List<float>();
    public bool oscillation = false;

    public PIDController(float OutputMax, float OutputMin)
    {
        this.OutputMax = OutputMax;
        this.OutputMin = OutputMin;

        this.GainDerivative = 1;
        this.GainIntegral = 1;
        this.GainProportional = 0;

    }

    public void setGains(float GainProportional, float GainIntegral, float GainDerivative)
    {

        this.GainDerivative = GainDerivative ;
        this.GainIntegral = GainIntegral;
        this.GainProportional = GainProportional;

    }

    /// <summary>
    /// The controller output
    /// </summary>
    /// <param name="timeSinceLastUpdate">timespan of the elapsed time
    /// since the previous time that ControlVariable was called</param>
    /// <returns>Value of the variable that needs to be controlled</returns>
    public float ControlVariable(float dt, float _error)
    {
        float error = (SetPoint - ProcessVariable);
        
        if (_error != 0f )
        {
            if (Mathf.Abs(error) < _error)
            {
                prev_SetPoint = SetPoint;
                return 0;
            }
        }
        
        // integral term calculation
        IntegralTerm += (GainIntegral * error * dt);
        IntegralTerm = Clamp(IntegralTerm);

        // derivative term calculation
        float dInput = processVariable - ProcessVariableLast;
        float derivativeTerm = GainDerivative * (dInput / dt);
        
        // proportional term calcullation
        float proportionalTerm = GainProportional * error;
     
        float output = proportionalTerm + IntegralTerm - derivativeTerm;

        output = Clamp(output);

        //oscillation = filterOscillation(output);

        prev_SetPoint = SetPoint;

        return output;
    }

    private bool filterOscillation(float input)
    {
        float output = input;
        int sampleSize = 100;


        if (Oscillation.Count < sampleSize +1)
        {
            Oscillation.Add(input);
            //Debug.Log("Count = " + Oscillation.Count + " add = " + input);

            if (Oscillation.Count == sampleSize)
            {
                float max = Oscillation.Max();
                //Debug.Log("Count = " + Oscillation.Count + " Max = 0" + max);

                if (max < 0.1)
                {
                    return true;
                }
            }
           
        }
        else if (Oscillation.Count > sampleSize - 1)
        {
            Oscillation.Clear();
        }

        return false;

        /*
        if (Oscillation.Count < sampleSize + 1)
        {
            Oscillation.Add(input);

            Debug.Log("FFT Input = " + input);
            Debug.Log("Count = " + Oscillation.Count);

            if (Oscillation.Count > sampleSize) {

                double[] realPart = new double[Oscillation.Count];
                double[] imaginaryPart = new double[Oscillation.Count];

                int j = 0;
                // Get real and complex components
                foreach (int i in Oscillation)
                {
                    Complex complex = new Complex(i,0);

                    realPart[j] = complex.Real;
                    imaginaryPart[j] = complex.Imaginary;

                    Debug.Log("FFT Input = " + i + " Real = " + complex.Real + " Imaginary = " + complex.Imaginary);

                    j ++;
                }

                // perform FFT
                FourierTransform fft = new FourierTransform();
                fft.run(realPart, imaginaryPart);

                // Results
                List<float> outputReal = new List<float>();
                List<float> outputImaginary = new List<float>();

                outputReal = fft.outputReal;
                outputImaginary = fft.outputImaginary;

                Debug.Log("FFT Out Real = ");
                foreach(int i in outputReal)
                {
                    Debug.Log(i + " , ");
                }

                Debug.Log("FFT Out Imaginary = ");
                foreach (int i in outputImaginary)
                {
                    Debug.Log(i + " , ");
                }

            }

        } 

        */


        //return output;
    }



    public float _error { get; private set; } = 0;


    /// <summary>
    /// The derivative term is proportional to the rate of
    /// change of the error
    /// </summary>
    public float GainDerivative { get; set; } = 0;

    /// <summary>
    /// The integral term is proportional to both the magnitude
    /// of the error and the duration of the error
    /// </summary>
    public float GainIntegral { get; set; } = 0;

    /// <summary>
    /// The proportional term produces an output value that
    /// is proportional to the current error value
    /// </summary>
    /// <remarks>
    /// Tuning theory and industrial practice indicate that the
    /// proportional term should contribute the bulk of the output change.
    /// </remarks>
    public float GainProportional { get; set; } = 0;

    /// <summary>
    /// The max output value the control device can accept.
    /// </summary>
    public float OutputMax { get;  set; } = 0;

    /// <summary>
    /// The minimum ouput value the control device can accept.
    /// </summary>
    public float OutputMin { get;  set; } = 0;

    /// <summary>
    /// Adjustment made by considering the accumulated error over time
    /// </summary>
    /// <remarks>
    /// An alternative formulation of the integral action, is the
    /// proportional-summation-difference used in discrete-time systems
    /// </remarks>
    public float IntegralTerm { get; private set; } = 0;


    /// <summary>
    /// The current value
    /// </summary>
    public float ProcessVariable
    {
        get { return processVariable; }
        set
        {
            ProcessVariableLast = processVariable;
            processVariable = value;
        }
    }

    /// <summary>
    /// The last reported value (used to calculate the rate of change)
    /// </summary>
    public float ProcessVariableLast { get; private set; } = 0;

    /// <summary>
    /// The desired value
    /// </summary>
    public float SetPoint { get; set; } = 0;

    /// <summary>
    /// Limit a variable to the set OutputMax and OutputMin properties
    /// </summary>
    /// <returns>
    /// A value that is between the OutputMax and OutputMin properties
    /// </returns>
    /// <remarks>
    /// Inspiration from http://stackoverflow.com/questions/3176602/how-to-force-a-number-to-be-in-a-range-in-c
    /// </remarks>
    private float Clamp(float variableToClamp)
    {
        if (variableToClamp <= OutputMin) { return OutputMin; }
        if (variableToClamp >= OutputMax) { return OutputMax; }
        return variableToClamp;
    }
}