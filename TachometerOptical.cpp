
// ######################################################################
// Include libraries:

#include "TachometerOptical.h"

using namespace TachometerOptical_Namespace;

// #######################################################################
// Define macros:

#define _2PI          6.2831853       // 2*pi

// ########################################################################
// Initialize static variables:

TachometerOptical* TachometerOptical::_instances[3] = {nullptr};

TimerControl* TachometerOptical::_TIMER = nullptr;

uint16_t TachometerOptical::_MAX = 0;

uint16_t TachometerOptical::_MIN = 0;

float TachometerOptical::_FILTER_FRQ = 0;

float TachometerOptical::_UPDATE_FRQ = 0;

float TachometerOptical::_alpha = 0;

volatile uint32_t TachometerOptical::_T = 0;

float TachometerOptical::ValuesStructure::sharedRPM = 0;	

std::string TachometerOptical::errorMessage = "";

// ##########################################################################
// General function definitions:

 void TachometerOptical_Namespace::_calcInput_CH1(void)
{
  unsigned long tNow = TachometerOptical::_TIMER->micros();
  TachometerOptical::_instances[0]->_period = (uint32_t)(tNow - TachometerOptical::_instances[0]->_startPeriod);
  TachometerOptical::_instances[0]->_startPeriod = tNow;
}

 void TachometerOptical_Namespace::_calcInput_CH2(void)
{
  unsigned long tNow = TachometerOptical::_TIMER->micros();
  TachometerOptical::_instances[1]->_period = (uint32_t)(tNow - TachometerOptical::_instances[1]->_startPeriod);
  TachometerOptical::_instances[1]->_startPeriod = tNow;
}

 void TachometerOptical_Namespace::_calcInput_CH3(void)
{
  unsigned long tNow = TachometerOptical::_TIMER->micros();
  TachometerOptical::_instances[2]->_period = (uint32_t)(tNow - TachometerOptical::_instances[2]->_startPeriod);
  TachometerOptical::_instances[2]->_startPeriod = tNow;
}

// ##########################################################################
// TachometerOptical class:

TachometerOptical::TachometerOptical()
{
		// Set default value at construction function:

    parameters.GPIO_PORT = nullptr;
    parameters.GPIO_PIN = GPIO_PIN_0;
    parameters.CHANNEL_NUM = 0;	

    EXTI_Callback = nullptr;

    value.rawRPM = 0;
    value.RPM = 0;

    _period = 0;
    _startPeriod = 0;

    _attachedFlag = false;
}

TachometerOptical::~TachometerOptical() 
{
  _instances[parameters.CHANNEL_NUM - 1] = nullptr;
  _attachedFlag = false;
}
	
void TachometerOptical::update(void)
{
	unsigned long t = TachometerOptical::_TIMER->micros();
  unsigned long dt = t - _T;

  if(TachometerOptical::_UPDATE_FRQ > 0)
  {
    if(dt < (1000000.0/TachometerOptical::_UPDATE_FRQ))
    {
      return ;
    }
  }

  if(TachometerOptical::_FILTER_FRQ > 0)
  {
    TachometerOptical::_alpha = 1.0 / (1.0 + _2PI * TachometerOptical::_FILTER_FRQ * dt / 1000000.0);
  }
  else
  {
    TachometerOptical::_alpha = 0;
  }

  for(int i = 1; i <= 3; i++)
  {
    if(_instances[i-1]->_attachedFlag == true)
    {
      float temp = (double)60.0/(double)(_instances[i-1]->_period)*1000000.0;

      if( (t - TachometerOptical::_instances[i-1]->_startPeriod) >  1000000.0)
      {
        temp = 0;
      }

      if(temp > TachometerOptical::_MIN)
      {
        if( (float)(temp - _instances[i-1]->value.rawRPM) / (float)dt > 10000.0)
        {
          _instances[i-1]->value.rawRPM = temp;
          continue;
        }
      }

      _instances[i-1]->value.rawRPM = temp;

      if(temp < TachometerOptical::_MIN)
      {
        temp = 0;
      }
      else if( (temp > TachometerOptical::_MAX) && (TachometerOptical::_MAX > 0) )
      {
        continue;
      }

      if(TachometerOptical::_FILTER_FRQ > 0)
      {
        _instances[i-1]->value.RPM = _alpha * _instances[i-1]->value.RPM + (1.0 - _alpha) * temp;
      }
      else
      {
        _instances[i-1]->value.RPM = temp;
      }

      ValuesStructure::sharedRPM = _instances[i-1]->value.RPM;
    }
  }	
	
		_T = t;
	
}	

bool TachometerOptical::setRange(uint16_t min, uint16_t max)
{
  if(max < min)
  {
    errorMessage = "Error TachometerOptical: The max RPM parameter value can not be less than The min RPM parameter value.";
    return false;
  }

  TachometerOptical::_MIN = min;
  TachometerOptical::_MAX = max;

  return true;
}

bool TachometerOptical::setUpdateFrequency(float value)
{
  if(value < 0)
  {
    errorMessage = "Error TachometerOptical:: The update frequency parameter value can not be less than 0.";
    return false;
  }

  TachometerOptical::_UPDATE_FRQ = value;
  return true;
}

bool TachometerOptical::setFilterFrequency(float value)
{
  if(value < 0)
  {
    errorMessage = "Error TachometerOptical: The filter frequency parameter value can not be less than 0.";
    return false;
  }

  TachometerOptical::_FILTER_FRQ = value;
  return true;
}

bool TachometerOptical::setTimerControl(TimerControl* timer)
{
  if(timer == nullptr)
  {
    errorMessage = "Error TachometerOptical: The TimerControl object pointer can not be nullptr.";
    return false;
  }

  if(timer->getInitState() == false)
  {
    errorMessage = "Error TachometerOptical: The TimerControl object must be initialized successfully before the TachometerOptical object.";
    return false;
  }

  TachometerOptical::_TIMER = timer;
  return true;
}

bool TachometerOptical::init(void)
{
  if(!_checkParameters())
  {
    return false;
  }

  _period = 0;
  _startPeriod = 0;

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if(parameters.GPIO_PORT != nullptr)
  {   
      RCC_GPIO_CLK_ENABLE(parameters.GPIO_PORT);
      GPIO_InitStruct.Pin = parameters.GPIO_PIN;
      GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(parameters.GPIO_PORT, &GPIO_InitStruct);
  }
  
  if(parameters.GPIO_PIN == GPIO_PIN_0)
  {
    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  }
  else if(parameters.GPIO_PIN == GPIO_PIN_1)
  {
    HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  }
  else if(parameters.GPIO_PIN == GPIO_PIN_2)
  {
    HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  }
  else if(parameters.GPIO_PIN == GPIO_PIN_3)
  {
    HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  }
  else if(parameters.GPIO_PIN == GPIO_PIN_4)
  {
    HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  }
  else if( (parameters.GPIO_PIN == GPIO_PIN_5) || (parameters.GPIO_PIN == GPIO_PIN_6) || (parameters.GPIO_PIN == GPIO_PIN_7) || (parameters.GPIO_PIN == GPIO_PIN_8) || (parameters.GPIO_PIN == GPIO_PIN_9) )
  {
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  }
  else if( (parameters.GPIO_PIN == GPIO_PIN_10) || (parameters.GPIO_PIN == GPIO_PIN_11) || (parameters.GPIO_PIN == GPIO_PIN_12) || (parameters.GPIO_PIN == GPIO_PIN_13) || (parameters.GPIO_PIN == GPIO_PIN_14) || (parameters.GPIO_PIN == GPIO_PIN_15) )
  {
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  }
  else
  {
    errorMessage = "Error TachometerOptical: The GPIO_PIN parameter is not correct.";
    return false;
  }

  EXTI_Callback = nullptr;
  
  switch(parameters.CHANNEL_NUM)
  {
    case 1:
      EXTI_Callback = _calcInput_CH1;
    break;
    case 2:
      EXTI_Callback = _calcInput_CH2;
    break;
    case 3:
      EXTI_Callback = _calcInput_CH3;
    break;	
  }

  // attachInterrupt(digitalPinToInterrupt(parameters.PIN_NUM), _funPointer, RISING);
  
  // Store object address in instances array.
  _instances[parameters.CHANNEL_NUM - 1] = this;

  _attachedFlag = true;

  return true;
}

bool TachometerOptical::_checkParameters(void)
{
  if( (parameters.CHANNEL_NUM > 3) || (parameters.CHANNEL_NUM == 0) )
  {
    errorMessage = "Error TachometerOptical: channel number is not correct.";
    return false;
  }

  if (_instances[parameters.CHANNEL_NUM - 1] != nullptr) 
  {
    errorMessage = "Error TachometerOptical: The channel number is used for another object. please select another channel.";
    return false;;

    /*
    // Detach any existing object for this channel
    _instances[channel_number - 1]->detach();
    delete _instances[channel_number - 1];
    */
  }

  bool state = (TachometerOptical::_FILTER_FRQ >= 0) && (TachometerOptical::_UPDATE_FRQ >= 0) &&
               (parameters.GPIO_PORT != nullptr) && (parameters.CHANNEL_NUM >= 1) && (parameters.CHANNEL_NUM <= 3) &&
               (TachometerOptical::_MAX >= TachometerOptical::_MIN) && (TachometerOptical::_TIMER != nullptr) ;

  if(state == false)
  {
    errorMessage = "Error TachometerOptical: One or some parameters is not correct.";
    return false;
  }

  if(TachometerOptical::_TIMER->getInitState() == false)
  {
    errorMessage = "Error TachometerOptical: The TimerControl object must be initialized successfully before the TachometerOptical object.";
    return false;
  }

  return true;
}

void TachometerOptical::RCC_GPIO_CLK_ENABLE(GPIO_TypeDef *GPIO_PORT)
{
    if(GPIO_PORT == nullptr)
    {
        return;
    }

    #ifdef GPIOA
    if(GPIO_PORT == GPIOA)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
    #endif
    #ifdef GPIOB
    else if (GPIO_PORT == GPIOB)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }
    #endif
    #ifdef GPIOC
    else if (GPIO_PORT == GPIOC)
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
    #endif
    #ifdef GPIOD
    else if (GPIO_PORT == GPIOD)
    {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    }
    #endif
    #ifdef GPIOE
    else if (GPIO_PORT == GPIOE)
    {
        __HAL_RCC_GPIOE_CLK_ENABLE();
    }
    #endif
    #ifdef GPIOF
    else if (GPIO_PORT == GPIOF)
    {
        __HAL_RCC_GPIOF_CLK_ENABLE();
    }
    #endif
    #ifdef GPIOG
    else if (GPIO_PORT == GPIOG)
    {
        __HAL_RCC_GPIOG_CLK_ENABLE();
    }
    #endif
    #ifdef GPIOH
    else if (GPIO_PORT == GPIOH)
    {
        __HAL_RCC_GPIOH_CLK_ENABLE();
    }
    #endif
    #ifdef GPIOI
    else if (GPIO_PORT == GPIOI)
    {
        __HAL_RCC_GPIOI_CLK_ENABLE();
    }
    #endif
}