#include "Subsystems/Pivot.h"
#include "Constants/Constants.h"

Pivot::Pivot() 
{

}

// This method will be called once per scheduler run
void Pivot::Periodic() 
{

}


double Pivot::GetTemperature()
{
  return m_pivotMotor.GetDeviceTemp().GetValue().value();
}

double Pivot::GetPower() 
{
  return m_pivotMotor.Get();
}

void Pivot::SetPower(double power)
{
  m_pivotMotor.Set(power);
}

void Pivot::SetTargetPosition(int position)
{
    pivotHome , pivotOpen = false;

    if (position == ALGAE_POS_HOME)
  {
    //move elevator home
    targetPosition = PIVOT_POSITION_HOME;
    pivotHome = true;
  }
  else if (position == ALGAE_POS_OPEN)
  {
    targetPosition = PIVOT_POSITION_OPEN;
    pivotOpen = true;
  }

  m_pivotMotor.SetControl(m_mmPivot.WithPosition(targetPosition));

}