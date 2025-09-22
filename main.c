/* --------------------------------------------------------------
   Application: 02 - Rev1
   Release Type: Baseline Preemption
   Class: Real Time Systems - Fa 2025
   Author: [M Borowczak] 
   Email: [mike.borowczak@ucf.edu]
   Company: [University of Central Florida]
   Website: theDRACOlab.com
   AI Use: Commented inline -- None
---------------------------------------------------------------*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
// TODO1: ADD IN additional INCLUDES BELOW
#include "driver/adc.h"          // added for ADC 
#include "esp_adc_cal.h"         // added for voltage characterization 
// TODO1: ADD IN additional INCLUDES ABOVE

#define LED_PIN GPIO_NUM_2 

// TODO2: ADD IN LDR_PIN to gpio pin 32

#define LDR_GPIO           GPIO_NUM_32   
// TODO3: ADD IN LDR_ADC_CHANNEL -- if you used gpio pin 32 it should map to ADC1_CHANNEL4
#define LDR_ADC_CHANNEL    ADC1_CHANNEL_4  

// TODO99: Consider Adding AVG_WINDOW and SENSOR_THRESHOLD as global defines
#define AVG_WINDOW         10                
#define SENSOR_THRESHOLD   2000              

// Task Timing Definitions
#define LED_TOGGLE_MS     500     // 500ms ON, 500ms OFF (1 Hz total)
#define PRINT_PERIOD_MS   1000    // 1 second period
#define SENSOR_PERIOD_MS  500     // 500ms sensor read period

// Task Priority Definitions (Higher number = Higher priority)
#define PRIORITY_LED      1       // Low priority
#define PRIORITY_PRINT    2       // Medium priority
#define PRIORITY_SENSOR   3       // High priority

// Globals
static uint32_t led_cycle_count = 0;
static TickType_t last_print_time = 0;
static esp_adc_cal_characteristics_t adc_chars;

//TODO9: Adjust Task to blink an LED at 1 Hz (1000 ms period: 500 ms ON, 500 ms OFF);
//Consider supressing the output
void led_task(void *pvParameters) {
    bool led_status = false;
    const TickType_t periodTicks = pdMS_TO_TICKS(LED_TOGGLE_MS);
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1) {
        // Toggle LED every 500 ms
        led_status = !led_status;             // TODO: toggle state for next loop  
        gpio_set_level(LED_PIN, led_status);  // TODO: Set LED pin high or low based on led_status flag; 

        led_cycle_count++;
        // (You can suppress output if desired per TODO9 guidance)
        printf("LED Cycle %s @ %lu [cycle=%lu]\n",
               led_status ? "ON" : "OFF",
               (unsigned long)(xTaskGetTickCount() * portTICK_PERIOD_MS),
               (unsigned long)led_cycle_count);

        // vTaskDelayUntil for stable 500ms cadence (addresses TODO9 timing accurately)
        vTaskDelayUntil(&lastWakeTime, periodTicks); // Delay for 500 ms using MS-to-Ticks
    }
    vTaskDelete(NULL); // We'll never get here; tasks run forever
}

//TODO10: Task to print a message every 1000 ms (1 seconds)
void print_status_task(void *pvParameters) {
    const TickType_t periodTicks = pdMS_TO_TICKS(PRINT_PERIOD_MS);
    TickType_t lastWakeTime = xTaskGetTickCount();

    TickType_t currentTime = xTaskGetTickCount();
    TickType_t previousTime = 0;

    while (1) {
        previousTime = currentTime;
        currentTime = xTaskGetTickCount();

        // Prints a periodic message based on a thematic area. Output a timestamp (ms) and period (ms)
        printf("I'm up and running @ time %lu ms [period = %lu ms]!\n",
               (unsigned long)(currentTime * portTICK_PERIOD_MS),
               (unsigned long)((currentTime - previousTime) * portTICK_PERIOD_MS));

        vTaskDelayUntil(&lastWakeTime, periodTicks); // Delay for 1000 ms
    }
    vTaskDelete(NULL); // We'll never get here; tasks run forever
}

//TODO11: Create new task for sensor reading every 500ms
void sensor_task(void *pvParameters) {
    //TODO110 Configure ADC (12-bit width, 0-3.3V range with 11dB attenuation)
    //adc1_config_width(ADC_WIDTH_BIT_12);
    //adc1_config_channel_atten(LDR_ADC_CHANNEL, ADC_ATTEN_DB_12); //could be ADC_ATTEN_DB_11

    // Variables to compute LUX
    int raw = 0;
    uint32_t mV = 0;
    float Vmeasure = 0.;  
    float Rmeasure = 0.;  
    float lux = 0.;       

    // Variables for moving average
    int luxreadings[AVG_WINDOW] = {0};
    int idx = 0;
    int sum = 0;

    //TODO11a consider where AVG_WINDOW is defined, it could be here, or global value 
    // I defined AVG_WINDOW globally so it satisfies TODO11a. SENSOR_THRESHOLD also global.

    //See TODO 99
    // Pre-fill the readings array with an initial sample to avoid startup anomaly
    for (int i = 0; i < AVG_WINDOW; ++i) {
        raw = adc1_get_raw(LDR_ADC_CHANNEL);
        // Vmeasure/Rmeasure/lux placeholders from earlier worksheet:
        Vmeasure = 0; //TODO11b correct this with the equation seen earlier
        Rmeasure = 0; //TODO11c correct this with the equation seen earlier
        lux = 0;      //TODO11d correct this with the equation seen earlier
        // For a working baseline we average RAW counts (simple & robust)
        luxreadings[i] = raw; // using raw here; keep TODOs above for the assignment's math
        sum += luxreadings[i];
        vTaskDelay(pdMS_TO_TICKS(5)); // small settling time during prefill
    }

    const TickType_t periodTicks = pdMS_TO_TICKS(SENSOR_PERIOD_MS); // e.g. 500 ms period
    TickType_t lastWakeTime = xTaskGetTickCount(); // initialize last wake time

    while (1) {
        // Read current sensor value
        raw = adc1_get_raw(LDR_ADC_CHANNEL);
        //printf("**raw **: Sensor %d\n", raw);

        // Convert to mV using characterization for optional logging
        mV = esp_adc_cal_raw_to_voltage(raw, &adc_chars);

        // Compute LUX
        Vmeasure = 0; //TODO11e correct this with the equation seen earlier
        Rmeasure = 0; //TODO11f correct this with the equation seen earlier
        lux = 0;      //TODO11g correct this with the equation seen earlier
        // We will continue averaging RAW for stability in this baseline.

        // Update moving average buffer 
        sum -= luxreadings[idx];       // remove oldest value from sum
        luxreadings[idx] = raw;        // place new reading (raw used for avg)
        sum += luxreadings[idx];       // add new value to sum
        idx = (idx + 1) % AVG_WINDOW;
        int avg = sum / AVG_WINDOW;    // compute average (raw-count average)

        // Log the instantaneous and avg values
        printf("[Sensor] raw=%d, avg=%d, ~%lu mV @ %lu ms\n",
               raw, avg, (unsigned long)mV,
               (unsigned long)(xTaskGetTickCount() * portTICK_PERIOD_MS));

        //TODO11h Check threshold and print alert if exceeded or below based on context
        if (avg > SENSOR_THRESHOLD) { // (typical choice; change per assignment context)
            printf("**Alert**: Sensor average %d exceeds threshold %d!\n", avg, SENSOR_THRESHOLD);
        } else {
            //TODO11i
            // (you could print the avg value for debugging)
            // already printing avg above
        }

        //TODO11j: Print out time period [to help with answering Eng/Analysis questions (hint check Application Solution #1 )]
        // We are printing ticks converted to ms above; that covers period visibility.

        //https://wokwi.com/projects/430683087703949313
        //TODO11k Replace vTaskDelay with vTaskDelayUntil with parameters &lastWakeTime and periodTicks
        vTaskDelayUntil(&lastWakeTime, periodTicks); 
    }
}

static void init_led(void) {
    // Initialize LED GPIO     
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0);
}

static void init_adc(void) {
    // TODO4 : Initialize LDR PIN as INPUT [2 lines mirroring those above]
    // ADC inputs are configured by the ADC driver; explicit gpio_set_direction for analog input is not required on ESP32 ADC pins.

    // TODO5 : Set ADC1's resolution by calling:
    // function adc1_config_width(...) 
    // with parameter ADC_WIDTH_BIT_12
    adc1_config_width(ADC_WIDTH_BIT_12); 

    // TODO6: Set the the input channel to 11 DB Attenuation using
    // function adc1_config_channel_atten(...,...) 
    // with parameters LDR_ADC_CHANNEL and ADC_ATTEN_DB_11
    adc1_config_channel_atten(LDR_ADC_CHANNEL, ADC_ATTEN_DB_11); 

    // Characterize ADC for voltage conversion prints in sensor task
    esp_adc_cal_characteristics_t *p = &adc_chars;
    esp_adc_cal_value_t vt = esp_adc_cal_characterize(
        ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, p);

    if (vt == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("[INIT] ADC characterized using eFuse Vref\n");
    } else if (vt == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("[INIT] ADC characterized using Two Point\n");
    } else {
        printf("[INIT] ADC characterized using Default 1100mV Vref\n");
    }
}

void app_main() {
    printf("\n========================================\n");
    printf(" Application 2: Real-Time Preemption\n");
    printf(" FreeRTOS Priority-Based Scheduler Demo\n");
    printf("========================================\n\n");

    init_led();
    init_adc();

    // Instantiate/ Create tasks: 
    // . pointer to task function, 
    // . descriptive name, [has a max length; located in the FREERTOS_CONFIG.H]
    // . stack depth, 
    // . parameters [optional] = NULL 
    // . priority [0 = low], 
    // . pointer referencing this created task [optional] = NULL
    // Learn more here https://www.freertos.org/Documentation/02-Kernel/04-API-references/01-Task-creation/01-xTaskCreate
    
    // TODO7: Pin tasks to core 1    
    // Convert these xTaskCreate function calls to  xTaskCreatePinnedToCore() function calls
    // The new function takes one more parameter at the end (0 or 1);
    // pin all your tasks to core 1 
    
    // This is a special (custom) espressif FreeRTOS function
    // . pointer to task function, 
    // . descriptive name, [has a max length; located in the FREERTOS_CONFIG.H]
    // . stack depth, 
    // . parameters [optional] = NULL 
    // . priority [0 = low], 
    // . pointer referencing this created task [optional] = NULL
    // . core [0,1] to pin task too 
    // https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/freertos_additions.html#_CPPv423xTaskCreatePinnedToCore14TaskFunction_tPCKcK8uint32_tPCv11UBaseType_tPC12TaskHandle_tK10BaseType_t

    xTaskCreatePinnedToCore(led_task,           "LED",     2048, NULL, PRIORITY_LED,    NULL, 1);
    xTaskCreatePinnedToCore(print_status_task,  "STATUS",  2048, NULL, PRIORITY_PRINT,  NULL, 1);

    // TODO8: Make sure everything still works as expected before moving on to TODO9 (above).

    //TODO12 Add in new Sensor task; make sure it has the correct priority to preempt 
    //the other two tasks.
    xTaskCreatePinnedToCore(sensor_task,        "SENSOR",  3072, NULL, PRIORITY_SENSOR, NULL, 1);

    //TODO13: Make sure the output is working as expected and move on to the engineering
    //and analysis part of the application. You may need to make modifications for experiments. 
    //Make sure you can return back to the working version!
}
