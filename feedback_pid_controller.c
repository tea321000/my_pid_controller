//*********************************************************************************
// Arduino PID Library Version 1.0.1 Modified Version for C -
// Platform Independent
// 
// Revision: 1.1
// 
// Description: The PID Controller module originally meant for Arduino made
// platform independent. Some small bugs present in the original Arduino source
// have been rectified as well.
// 
// For a detailed explanation of the theory behind this library, go to:
// http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
// 
// Revisions can be found here:
// https://github.com/tcleg
// 
// Modified by: Trent Cleghorn , <trentoncleghorn@gmail.com>
// 
// Copyright (C) Brett Beauregard , <br3ttb@gmail.com>
// 
//                                 GPLv3 License
// 
// This program is free software: you can redistribute it and/or modify it under 
// the terms of the GNU General Public License as published by the Free Software 
// Foundation, either version 3 of the License, or (at your option) any later 
// version.
// 
// This program is distributed in the hope that it will be useful, but WITHOUT ANY 
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
// PARTICULAR PURPOSE.  See the GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License along with 
// this program.  If not, see <http://www.gnu.org/licenses/>.
//*********************************************************************************

//*********************************************************************************
// Headers
//*********************************************************************************
#include "feedback_pid_controller.h"

//*********************************************************************************
// Macros and Globals
//*********************************************************************************
#define CONSTRAIN(x,lower,upper)    ((x)<(lower)?(lower):((x)>(upper)?(upper):(x)))

//*********************************************************************************
// Functions
//*********************************************************************************

void PIDInit(PIDControl *pid, float FdK, float PreK, float PreKv, 
             float sampleTimeSeconds, float minOutput, float maxOutput, 
             PIDMode mode, PIDDirection controllerDirection)     	
{
    pid->controllerDirection = controllerDirection;
    pid->mode = mode;
    //pid->iTerm = 0.0f;
    pid->input = 0.0f;
    //pid->lastInput = 0.0f;
    pid->pre_error=0.0f;
    pid->pre_two_error=0.0f;
    pid->output = 0.0f;
    pid->setpoint = 0.0f;
    
    if(sampleTimeSeconds > 0.0f)
    {
        pid->sampleTime = sampleTimeSeconds;
    }
    else
    {
        // If the passed parameter was incorrect, set to 1 second
        pid->sampleTime = 1.0f;
    }
    
    PIDOutputLimitsSet(pid, minOutput, maxOutput);
    PIDTuningsSet(pid, FdK, PreK, PreKv);
    
}
        
bool
PIDCompute(PIDControl *pid) 
{
    //float error, dInput;
    float error;

    if(pid->mode == MANUAL)
    {
        return false;
    }
    
    // The classic PID error term:注意负反馈PID的偏差值与一般PID的刚好相反
    error =  (pid->input) - (pid->setpoint);
    // Compute the integral term separately ahead of time
    //pid->iTerm += (pid->alteredKi) * error;
    
    // Constrain the integrator to make sure it does not exceed output bounds
    //pid->iTerm = CONSTRAIN( (pid->iTerm), (pid->outMin), (pid->outMax) );
    
    // Take the "derivative on measurement" instead of "derivative on error"
    //dInput = (pid->input) - (pid->lastInput);


    // Run all the terms together to get the overall output
    //pid->output += (pid->alteredKp)*(error-pid->pre_error)+pid->alteredKi*error+pid->alteredKd*(error-2*pid->pre_error+pid->pre_two_error);
    //pid->output = (pid->alteredKp) * error + (pid->iTerm) - (pid->alteredKd) * dInput;


    //线性预测负反馈:Delta Output= -FdK*(ET+vT*PreK)；
			pid->output-=(pid->FdK)*(error+(error-pid->pre_error)*(pid->PreK));
		
    //非线性预测负反馈一般形式：Delta Output= -FdK*[ET+PreK*（vT+PreKv*aT）]
    //pid->output-=(pid->FdK)*(error+(pid->PreK)*(error-pid->pre_error+(pid->PreKv)*(error-2*pid->pre_error+pid->pre_two_error)));
    


    // Bound the output
		
    pid->output = CONSTRAIN( (pid->output), (pid->outMin), (pid->outMax));
    pid->pre_two_error = pid->pre_error;
    pid->pre_error = error;
    // Make the current input the former input
    //pid->lastInput = pid->input;
    
    return true;
}
     
void 
PIDModeSet(PIDControl *pid, PIDMode mode)                                                                                                                                       
{
    // If the mode changed from MANUAL to AUTOMATIC
    if(pid->mode != mode && mode == AUTOMATIC)
    {
        // Initialize a few PID parameters to new values
        //pid->iTerm = pid->output;
        //pid->lastInput = pid->input;
        
        // Constrain the integrator to make sure it does not exceed output bounds
        //pid->iTerm = CONSTRAIN( (pid->iTerm), (pid->outMin), (pid->outMax) );
    }
    
    pid->mode = mode;
}

void 
PIDOutputLimitsSet(PIDControl *pid, float min, float max) 							  							  
{
    // Check if the parameters are valid
    if(min >= max)
    {
        return;
    }
    
    // Save the parameters
    pid->outMin = min;
    pid->outMax = max;
    
    // If in automatic, apply the new constraints
    if(pid->mode == AUTOMATIC)
    {
        pid->output = CONSTRAIN(pid->output, min, max);
    }
}

void 
PIDTuningsSet(PIDControl *pid, float FdK, float PreK, float PreKv)         	                                         
{
    // Check if the parameters are valid
    if(FdK < 0.0f || PreK < 0.0f || PreKv < 0.0f)
    {
        return;
    }
    
    // Save the parameters for displaying purposes
    // pid->dispKp = kp;
    // pid->dispKi = ki;
    // pid->dispKd = kd;
    
    // Alter the parameters for PID
    // pid->alteredKp = kp;
    // pid->alteredKi = ki * pid->sampleTime;
    // pid->alteredKd = kd / pid->sampleTime;

    pid->FdK=FdK;
    pid->PreK=PreK;
    pid->PreKv=PreKv;
    
    // Apply reverse direction to the altered values if necessary
    if(pid->controllerDirection == REVERSE)
    {
        // pid->alteredKp = -(pid->alteredKp);
        // pid->alteredKi = -(pid->alteredKi);
        // pid->alteredKd = -(pid->alteredKd);
        pid->FdK=-(pid->FdK);
        pid->PreK=-(pid->PreK);
        pid->PreKv=-(pid->PreKv);
    }
}

void 
PIDTuningFdkSet(PIDControl *pid, float FdK)
{
    PIDTuningsSet(pid, FdK, pid->PreK, pid->PreKv);
}

void 
PIDTuningPreKSet(PIDControl *pid, float PreK)
{
    PIDTuningsSet(pid, pid->FdK, PreK, pid->PreKv);
}

void 
PIDTuningPreKvSet(PIDControl *pid, float PreKv)
{
    PIDTuningsSet(pid, pid->FdK, pid->PreK, PreKv);
}

void 
PIDControllerDirectionSet(PIDControl *pid, PIDDirection controllerDirection)	  									  									  									  
{
    // If in automatic mode and the controller's sense of direction is reversed
    if(pid->mode == AUTOMATIC && controllerDirection == REVERSE)
    {
        // Reverse sense of direction of PID gain constants
        pid->FdK=-(pid->FdK);
        pid->PreK=-(pid->PreK);
        pid->PreKv=-(pid->PreKv);
    }
    
    pid->controllerDirection = controllerDirection;
}

// void 
// PIDSampleTimeSet(PIDControl *pid, float sampleTimeSeconds)                                                       									  									  									   
// {
//     float ratio;

//     if(sampleTimeSeconds > 0.0f)
//     {
//         // Find the ratio of change and apply to the altered values
//         ratio = sampleTimeSeconds / pid->sampleTime;
//         pid->alteredKi *= ratio;
//         pid->alteredKd /= ratio;
        
//         // Save the new sampling time
//         pid->sampleTime = sampleTimeSeconds;
//     }
// }


