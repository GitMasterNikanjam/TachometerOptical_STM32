#pragma once

// ##############################################################################################
// MCU Select:

#include "mcu_select.h"

/*
    If there is not exist mcu_select.h at beside of this header file, Create it and put this bellow following content. 
    Then select your desired MCU that want work with.
*/
// ----------------------------------------------------------------
// mcu_select.h file:

// Define the target MCU family here
// Uncomment the desired MCU family definition below:

// #define STM32F1
// #define STM32F4
// #define STM32H7

// ----------------------------------------------------------------
// ##################################################################
// Library information:
/*
TachometerOptical - a small optical tachometer library for STM32.
For more information read README.md file.
*/
// ###################################################################
// Include libraaries:

#if defined(STM32F1)
#include "stm32f1xx_hal.h"      // HAL library for STM32F1 series
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"      // HAL library for STM32F4 series
#elif defined(STM32H7)
#include "stm32h7xx_hal.h"      // HAL library for STM32H7 series
#else
#error "Unsupported MCU family. Please define a valid target (e.g., STM32F1, STM32F4, STM32H7)."
#endif

#include <string>               // Include the standard string library for error handling and messages
#include "TimerControl.h"

// ####################################################################
// Define Global macros:


// ###################################################################################
//  General function declarations:

namespace TachometerOptical_Namespace
{
  void _calcInput_CH1(void);       /// @brief Interrupt handler function for TachometerOptical channel 1.
  void _calcInput_CH2(void);       /// @brief Interrupt handler function for TachometerOptical channel 2.
  void _calcInput_CH3(void);       /// @brief Interrupt handler function for TachometerOptical channel 3.
}

// ##################################################################################3
// TachometerOptical class

/**
  @class TachometerOptical
  @brief The class for tachometer applications. eg: measure motor RPM.  
*/
class TachometerOptical
{
  public:

    /// @brief Last error accured for object.
    static std::string errorMessage;

    /**
      @struct ParametersStructure
      @brief Parameters structure.
    */ 
    struct ParametersStructure
    {
      /**
       * @brief Sensor GPIO port connection.
       */
      GPIO_TypeDef *GPIO_PORT;

      /**
       * @brief Sensor GPIO pin connection. It can be GPIO_PIN_0, GPIO_PIN_1, ... 
       */
      uint16_t GPIO_PIN;

      /**
       * @brief Channel number. A maximum of three different channels can be used for all RPM objects.
       * @note - This value can only be 1, 2, or 3.
       *  */ 
      uint8_t CHANNEL_NUM;											

    }parameters;

    /**
      @struct ValuesStructure
      @brief Values structure.
    */ 
    struct ValuesStructure
    {
      /// @brief Raw input RPM signal measurement values. [RPM].
      float rawRPM;
      
      /// @brief RPM value after low-pass filter and MIN/MAX saturation. [RPM].
      float RPM;

      /// @brief RPM value updated by all TachometerOptical objects.  
      static float sharedRPM;		
    }value;

    /// @brief Define function pointer type
    typedef void (*FunctionPtr)();

    /// @brief FunctionPtr object for signals interrupts handler.
    FunctionPtr EXTI_Callback;

    /**
    * @brief Default constructor. Init default value of variables and parameters.
    */
    TachometerOptical();

    /// @brief Destructor
    ~TachometerOptical();

    /**
     * @brief Initialize object. Check parameters validation.
     * @return true if succeeded.
     */ 
    bool init(void);

    /**
     * @brief Update and calculate filtered RPM value.
     */
    static void update(void);

    /**
     * @brief Set RPM acceptable value range.
     * @return true if successful.
     */
    static bool setRange(uint16_t min, uint16_t max);

    /**
     * @brief Set The RPM update frequency. [Hz]
     * @note - This value ensures that RPM filtered values are updated at a certain frequency.
     * @note - A value of 0 means it is disabled.
     * @note - This parameter is static and applies globally to all TachometerOptical objects.
     */
    static bool setUpdateFrequency(float value);

    /**
     * @brief Set the RPM Low pass filter frequency(Cutoff filter frequency). [Hz].
     * @note - A value of 0 means it is disabled.
     * @note - This parameter is static and applies globally to all TachometerOptical objects.
     */
    static bool setFilterFrequency(float value);

    /**
     * @brief Set the TimerControl object pointer.
     * @note - Set this timer carefully because the object calculate RPM by this timer.
     * 
     * @note - The TimerControl object must be configured and initialized prior to and outside of the object.
     * 
     * @note - All TachometerOptical objects share the use of this timer. It is can set by each object individually.
     */
    static bool setTimerControl(TimerControl* timer);

  private:

    /**
     * @brief TimerControl pointer. 
     * @note - Set this timer carefully because the object calculate RPM by this timer.
     * 
     * @note - The TimerControl object must be configured and initialized prior to and outside of the object.
     * 
     * @note - All TachometerOptical objects share the use of this timer. It is can set by each object individually.
     */
    static TimerControl* _TIMER;

    /**
     * @brief Minimum RPM value accepted in the update method. If the RPM is below this minimum, it returns a zero value.
     * @note - This parameter is static and applies globally to all TachometerOptical objects.
    */
    static uint16_t _MIN;

    /**
     * @brief Maximum RPM value accepted in the update method. If the RPM is above this maximum, it returns the last updated value.
     * @note - A value of 0 means it is disabled.
     * @note - This parameter is static and applies globally to all TachometerOptical objects.
    */
    static uint16_t _MAX;

    /**
     * @brief Low pass filter frequency(Cutoff filter frequency). [Hz].  
     * @note - A value of 0 means it is disabled.
     * @note - This parameter is static and applies globally to all TachometerOptical objects.
    */ 
    static float _FILTER_FRQ;

    /**
     * @brief Update frequency. This value ensures that RPM filtered values are updated at a certain frequency. 
     * @note - A value of 0 means it is disabled.
     * @note - This parameter is static and applies globally to all TachometerOptical objects.
    */ 
    static float _UPDATE_FRQ;

    /**
     * @brief Static array to store instances per channel.  
     * Array to hold one object per channel (1-3).    
     * Cell 0 is for channel 1. Cell 1 is for channel 2. Cell 2 is for channel 3.
    */
    static TachometerOptical* _instances[3];

    /// @brief Flag to store the state of channels that are attached (true) or not attached (false).
    bool _attachedFlag;

    /// @brief Start timer value. [us]
    volatile uint32_t _startPeriod;		

    /**
     * @brief Period time value. [us]
     */
    volatile uint32_t _period;				
    
    /**
      @brief Gain used in low-pass filter calculations.  
      @note _alpha = 1.0 / (1.0 + _2PI * FILTER_FRQ / UPDATE_FRQ)
    */ 
    static float _alpha;
    
    /// @brief Time at the update() method. [us].
    static volatile uint32_t _T;

    /** 
    * @brief Check parameters validation.
    * @return true if succeeded.
    */
    bool _checkParameters(void);

    /**
     * @brief Enable RCC GPIO PORT for certain port.
     */
    void RCC_GPIO_CLK_ENABLE(GPIO_TypeDef *GPIO_PORT);

    // ---------------------------------------------------------------
    // Friends functions:

    /// @brief Interrupt handler function for TachometerOptical channel 1.
    friend void TachometerOptical_Namespace::_calcInput_CH1(void);

    /// @brief Interrupt handler function for TachometerOptical channel 2.
    friend void TachometerOptical_Namespace::_calcInput_CH2(void);

    /// @brief Interrupt handler function for TachometerOptical channel 3.
    friend void TachometerOptical_Namespace::_calcInput_CH3(void);
    
};



