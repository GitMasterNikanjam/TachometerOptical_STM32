# TachometerOptical Library for Arduino 

- This library can used for optical tachometers applications. eg: Motor RPM measurement.  
- The main class is **TachometerOptical**. 
- Max 3 number optical TachometerOptical object can created from RPTachometerOpticalM class at the same time.   
- Each TachometerOptical object has its own digital pin number for signal interrupts.  
- It should be just use digital pins that can used in hardware external interrupts mode. otherwise it can not work correct.  
- The pins for get input interrupts is pull up.   

## Public Member Functions

```c++

/**
     * @brief Initialize object. Check parameters validation.
     * @return true if succeeded.
     */ 
    bool init(void);

    /**
     * @brief Update and calculate filtered RPM value.
     */
    static void update(void);
    
```

## Public Member Variables

```c++

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

```