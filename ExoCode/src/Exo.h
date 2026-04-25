/**
 * @file Exo.h
 *
 * @brief 声明 Exo 类，Exo类是外骨骼控制逻辑的主要实现，系统所有其他组件都将归属于该类中     Declares exo class that all the other components will live in. 
 * 
 * @author P. Stegall 
 * @date Jan. 2022
*/


#ifndef Exo_h
#define Exo_h

//Arduino compiles everything in the src folder even if not included so it causes an error for the nano if this is not included
// Arduino 会编译 src 文件夹下的所有文件，即便这些文件没有被其他文件 #include 包含。
// 因此，如果不加上这段条件编译，代码在 Arduino Nano 上编译时就会报错。
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

#include "Arduino.h"

#include "Side.h"
#include <stdint.h>
#include "ParseIni.h"
#include "Board.h"
#include "Utilities.h"
#include "SyncLed.h"
#include "StatusLed.h"
#include "StatusDefs.h"
#include "Config.h"

class Exo
{
    public:
		Exo(ExoData* exo_data); //Constructor: uses initializer list for the Side objects.
		
        /**
         * @brief Reads motor data from each motor used on that side and stores the values
         * 
         * @return true if the code ran, ie tiiming was satisfied
         * @return false 
         */
        bool run();  
		
        ExoData *data;      /**< Pointer to ExoData that is getting updated by the coms mcu so they share format.*/
        Side left_side;     /**< Left side object that contains all the joints and sensors for that side */
        Side right_side;    /**< Right side object that contains all the joints and sensors for that side */
        
        #ifdef USE_SPEED_CHECK
            utils::SpeedCheck speed_check; /**< Used to check the speed of the loop without needing prints */
        #endif
        
        SyncLed sync_led;       /**< Used to syncronize data with a motion capture system */
        StatusLed status_led;   /**< Used to display the system status */
			
	private:
		
};
#endif

#endif