#ifndef TACHOMETER_OPTICAL_H
#define TACHOMETER_OPTICAL_H

// ##################################################################
// Library information:
/*
TachometerOptical - a small optical tachometer library for Arduino.
For more information read README.md file.
*/
// ###################################################################
// Include libraaries:

#include <Arduino.h>

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
// RPM class

/**
  @class TachometerOptical
  @brief The class for tachometer applications. eg: measure motor RPM.  
*/
class TachometerOptical
{
  public:

    /// @brief Last error accured for object.
    String errorMessage;

    /**
      @struct ParametersStructure
      @brief Parameters structure.
    */ 
    struct ParametersStructure
    {
      /**
       * @brief Minimum RPM value accepted in the update method. Below that, it returns a zero value.
       * Default value: 0. This means it is disabled.
      */
      static uint16_t MIN;

      /**
       * @brief Maximum RPM value accepted in the update method. Above that, it returns the last updated value.
       * Default value: 0. This means it is disabled.
      */
      static uint16_t MAX;

      /**
       * @brief Low pass filter frequency(Cutoff filter frequency). [Hz].  
       * Default value: 0. This means it is disabled.
      */ 
      static float FILTER_FRQ;

      /**
        @brief Update frequency. This value ensures that RPM filtered values are updated at a certain frequency. 
        Default value: 0. This means it is disabled.
      */ 
      static float UPDATE_FRQ;

      /**
        @brief Digital pin number of the Arduino used for input PWM signal.
        A value of -1 means no pin is assigned.
      */ 
      int8_t PIN_NUM;	

      /// @brief Channel number. A maximum of 3 different channels can be used for all RPM objects.
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

  private:

    /**
     * @brief Static array to store instances per channel.  
     * Array to hold one object per channel (1-3).    
     * Cell 0 is for channel 1. Cell 1 is for channel 2. Cell 2 is for channel 3.
    */
    static TachometerOptical* _instances[3];

    /// @brief Define function pointer type
    typedef void (*FunctionPtr)();

    /// @brief FunctionPtr object for signals interrupts handler.
    FunctionPtr _funPointer;

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

    // ---------------------------------------------------------------
    // Friends functions:

    /// @brief Interrupt handler function for TachometerOptical channel 1.
    friend void TachometerOptical_Namespace::_calcInput_CH1(void);

    /// @brief Interrupt handler function for TachometerOptical channel 2.
    friend void TachometerOptical_Namespace::_calcInput_CH2(void);

    /// @brief Interrupt handler function for TachometerOptical channel 3.
    friend void TachometerOptical_Namespace::_calcInput_CH3(void);
    
};


#endif