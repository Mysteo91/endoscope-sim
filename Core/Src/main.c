/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "vl6180_api.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define max_scale 1
#define ALLOW_DISABLE_WAF_FROM_BLUE_BUTTON 1
#define theVL6180Dev   0x52    // what we use as "API device
#define def_i2c_time_out 100
#if VL6180_HAVE_DMAX_RANGING
#define DMaxDispTime     0 /* Set to 1000 to display Dmax during 1 sec when no target is detected */
#else
#define DMaxDispTime     0
#endif
#define OutORangeDispfTime  800

#define debug(msg, ...)  (void)0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t asd = 0;
volatile int VL6180_IsrFired=0;

/**
*/

int nErr=0;
void OnErrLog(void){
    /* just log */
    nErr++;
}

enum runmode_t{
    RunRangePoll=0,
    RunAlsPoll,
    InitErr,
    ScaleSwap,
    WaitForReset,
    AlrmStart,
    AlrmRun,
    FromSwitch,
};

char buffer[10];

struct state_t {
    int OutofRAnge:1;
    int AutoScale:1;
    int FilterEn:1;
    uint8_t mode;
    int8_t ScaleSwapCnt;
    uint8_t InitScale;

    uint8_t CurAlrm;
    uint8_t AlrmFired; /* just used to keep display at least min time */
}State;

uint32_t TimeStarted;       /* various display and mode delay starting time */
VL6180_RangeData_t Range;  /* Range measurmeent  */

int alpha =(int)(0.85*(1<<16));    /* range distance running average cofs */
uint16_t range;             /* range average distance */

#define AutoThreshHigh  80  /*auto scale high thresh => AutoThreshHigh *  max_raneg => scale ++  */
#define AutoThreshLow   33  /*auto scale low thresh  => AutoThreshHigh *  max_raneg => scale ++  */
#define ErrRangeDispTime    0 /* Set to 800 ms to display error code when no target os detected */
#if ErrRangeDispTime == 0
/*   supress Warning[Pe186]: pointless comparison of unsigned integer with zero */
#   ifdef __ARMCC_VERSION /* ARM/KEIL */
#   pragma diag_suppress 186
#   endif  /* _ARMCC_VERSION */
#   ifdef __ICCARM__ /* IAR */
#       pragma diag_suppress=Pe186
#   endif  /* _ARMCC_VERSION */
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void debug_stuff(void) {
    int reg_cmd = 0;
    static uint32_t reg_data;
    static uint16_t reg_index;

    if (reg_cmd) {
        switch (reg_cmd) {
            case -1:
                VL6180_WrByte(theVL6180Dev, reg_index, reg_data);
                debug("Wr B 0x%X = %d", reg_index, (int)reg_data);
                break;
            case -2:
                VL6180_WrWord(theVL6180Dev, reg_index, reg_data);
                debug("Wr W 0x%X = %d", reg_index,(int) reg_data);
                break;

            case -3:
                VL6180_WrDWord(theVL6180Dev, reg_index, reg_data);
                debug("WrDW 0x%X = %d", reg_index, (int)reg_data);
                break;

            case 1:
                reg_data=0;
                VL6180_RdByte(theVL6180Dev, reg_index, (uint8_t*)&reg_data);
                debug("RD B 0x%X = %d", reg_index, (int)reg_data);
                break;
            case 2:
                reg_data=0;
                VL6180_RdWord(theVL6180Dev, reg_index, (uint16_t*)&reg_data);
                debug("RD W 0x%X = %d", reg_index, (int)reg_data);
                break;

            case 3:
                VL6180_RdDWord(theVL6180Dev, reg_index, &reg_data);
                debug("RD DW 0x%X = %d", reg_index, (int)reg_data);
                break;
            default:
                debug("Invalid command %d", reg_cmd);
                /* nothing to do*/
                ;
        }
    }
}



#if !FREE_RTOS
extern __IO uint32_t uwTick;

#define g_TickCnt uwTick
char buff[10]="---.--";
float v=0000.1;


void WaitMilliSec(int ms){
    uint32_t start, now;
    int dif;
    start=g_TickCnt;
    //debug("waiting %d @%d\n",ms, g_TickCnt);
    do{
        now=g_TickCnt;
        dif= now -start;
    }
    while(dif<ms);
    //debug("waited  %d @%d\n",dif, g_TickCnt);
}

#else
//for FreeRtos ue os wait
void WaitMilliSec(int ms)   vTaskDelay( ms/ portTICK_PERIOD_MS)

#endif

const char *DISP_NextString;

void SetDisplayString(const char *msg) {
    DISP_NextString=msg;
}
void AbortErr( const char * msg ){
    SetDisplayString( msg);
    State.mode=  WaitForReset;
}

void DoScalingSwap(int scaling){
    if( State.AutoScale){
        if( State.FilterEn )
            SetDisplayString("Sf A");
        else
            SetDisplayString("Sc A");
    }
    else{
        if( State.FilterEn )
            sprintf(buffer, "Sf %d", (int)scaling);
        else
            sprintf(buffer, "Sc %d", (int)scaling);
        SetDisplayString(buffer);

    }
    State.mode = ScaleSwap;
    TimeStarted=g_TickCnt;
}

void RangeState(void) {
    int status;
    uint16_t hlimit;
    uint8_t scaling;

    scaling = VL6180_UpscaleGetScaling(theVL6180Dev);
    status = VL6180_RangePollMeasurement(theVL6180Dev, &Range); /* these invoke dipslay for  polling */
    if (status) {
        AbortErr("Er r");
        return;
    }

    hlimit = VL6180_GetUpperLimit(theVL6180Dev);
    if (Range.range_mm >= (hlimit * AutoThreshHigh) / 100 && scaling < 3 && State.AutoScale) {
        VL6180_UpscaleSetScaling(theVL6180Dev, scaling + 1);
    }
    if (Range.range_mm < (hlimit * AutoThreshLow) / 100 && scaling > 1 && State.AutoScale) {
        VL6180_UpscaleSetScaling(theVL6180Dev, scaling - 1);
    }

    if (Range.errorStatus) {
        /* no valid ranging*/
        if (State.OutofRAnge) {
#if VL6180_HAVE_DMAX_RANGING
            if (g_TickCnt - TimeStarted >= ErrRangeDispTime &&  g_TickCnt - TimeStarted <  ErrRangeDispTime + DMaxDispTime ){
                    sprintf(buffer, "d%3d", (int)Range.DMax);
                    SetDisplayString(buffer);
            }
            else

#endif
            if(g_TickCnt - TimeStarted < ErrRangeDispTime  )
            {

                sprintf(buffer, "rE%2d", (int) Range.errorStatus);
                SetDisplayString(buffer);
            }
            else{
                State.OutofRAnge=0; /* back to out of range display */
                TimeStarted=g_TickCnt;
            }
        }
        else {
            int FilterEn;
#if VL6180_WRAP_AROUND_FILTER_SUPPORT
           FilterEn = VL6180_FilterGetState(theVL6180Dev);
            if (FilterEn && VL6180_RangeIsFilteredMeasurement(&Range) ){
                SetDisplayString("F---");
            }
            else
                SetDisplayString("r---");
#else
            SetDisplayString("r---");
#endif
            if( g_TickCnt - TimeStarted > OutORangeDispfTime ) {
                State.OutofRAnge = 1;
                TimeStarted = g_TickCnt;
            }
        }
    }
    else {
        State.OutofRAnge = 0;
        TimeStarted = g_TickCnt;
        range = (range * alpha + Range.range_mm * ((1 << 16) - alpha)) >> 16;
        sprintf(buffer, "r%3d", (int) range);
        if (State.AutoScale) {
            if (scaling == 1) {
                buffer[0] = '_';
            }
            else
            if (scaling == 2)
                buffer[0] = '=';
            else
                buffer[0] = '~';
        }

        SetDisplayString(buffer);
    }


    if (asd == 1) {
        asd = 0;
        TimeStarted = g_TickCnt;
        State.ScaleSwapCnt++;
        if (State.ScaleSwapCnt % (max_scale + 1) == max_scale) {
            State.AutoScale = 1;
            scaling = max_scale;
        }
        else {
#if ALLOW_DISABLE_WAF_FROM_BLUE_BUTTON
            /* togle filtering every time we roll over all scaling(pass by autoscale) */
            if (State.AutoScale)
                State.FilterEn = !State.FilterEn;
#endif
            State.AutoScale = 0;
            scaling = State.InitScale + (State.ScaleSwapCnt % max_scale);
            if (scaling > max_scale)
                scaling = scaling - (max_scale);
        }
        status = VL6180_UpscaleGetScaling(theVL6180Dev);
        if (status == 0 )
        {
            __NOP();
        }
        status = VL6180_UpscaleSetScaling(theVL6180Dev, scaling);

        if (status<0) {
            AbortErr("ErUp");
            State.mode = InitErr;
        }
        else {
            /* do not check status may fail when filter support not active */
            VL6180_FilterSetState(theVL6180Dev, State.FilterEn);
            DoScalingSwap(scaling);
        }
    }
}
#define AlrmDispTime        800


void AlarmShowMode(const char *msg)
{
    SetDisplayString( msg);
    TimeStarted=g_TickCnt;
    do {
        __NOP();
    } while (g_TickCnt - TimeStarted < AlrmDispTime);
}

void AlarmLowThreshUseCase(void){
    AlarmShowMode("A-Lo");

    /* make sure from now on all register in group are not fetched by device */
    VL6180_SetGroupParamHold(theVL6180Dev, 1);

    /* get interrupt whenever we go below 200mm */
    VL6180_RangeSetThresholds(theVL6180Dev, 200, 0, 0 );
    /* set range interrupt reporting low threshold*/
    VL6180_RangeConfigInterrupt(theVL6180Dev, CONFIG_GPIO_INTERRUPT_LEVEL_LOW);

    /* leave device peak up all new register in group */
    VL6180_SetGroupParamHold(theVL6180Dev, 0);

    /* clear any interrupt that should ensure a new edge get generated even if we missed it */
    VL6180_RangeClearInterrupt(theVL6180Dev);
}



void AlarmHighThreshUseCase(void){
    AlarmShowMode("A-hi");
    /* make sure from now on all register in group are not fetched by device */
    VL6180_SetGroupParamHold(theVL6180Dev, 1);

    /* get interrupt whenever  higher than 200mm (low threshold don't care) */
    VL6180_RangeSetThresholds(theVL6180Dev, 0, 200, 0 );

    /* set range interrupt reporting high threshold*/
    VL6180_RangeConfigInterrupt(theVL6180Dev, CONFIG_GPIO_INTERRUPT_LEVEL_HIGH);

    /* leave device peak up all new register in group */
    VL6180_SetGroupParamHold(theVL6180Dev, 0);

    /* clear any interrupt that should ensure a new edge get generated even if we missed it */
    VL6180_RangeClearInterrupt(theVL6180Dev);

}

void AlarmWindowThreshUseCase(void){

    AlarmShowMode("A-0o");

    /* make sure from now on all register in group are not fetched by device */
    VL6180_SetGroupParamHold(theVL6180Dev, 1);

    /* get interrupt whenever  out of  100mm  250mm  range */
    VL6180_RangeSetThresholds(theVL6180Dev, 100, 200, 0 );

    /* set range interrupt reporting out of window  */
    VL6180_RangeConfigInterrupt(theVL6180Dev, CONFIG_GPIO_INTERRUPT_OUT_OF_WINDOW);

    /* leave device peak up all new register in group */
    VL6180_SetGroupParamHold(theVL6180Dev, 0);

    /* clear any interrupt that should ensure a new edge get generated even if we missed it */
    VL6180_RangeClearInterrupt(theVL6180Dev);

}

void AlarmUpdateUseCase(void)
{
    State.CurAlrm =(State.CurAlrm%3);

    switch ( State.CurAlrm) {
        case 0: /* low thresh */
            AlarmLowThreshUseCase();
            break;
        case 1: /* high thresh */
            AlarmHighThreshUseCase();;
            break;
        case 2: /* out of window */
            AlarmWindowThreshUseCase();
    }
    VL6180_RangeClearInterrupt(theVL6180Dev); /* clear any active interrupt it will ensure we get a new active edge is raised */
}


void AlarmStop(void){
    VL6180_RangeSetSystemMode(theVL6180Dev, MODE_CONTINUOUS|MODE_START_STOP);
    /* Wait some time for last potential measure to stop ?
     * TODO can we poll check something to avoid that delay? */
    WaitMilliSec(100);
    /* Clear any left pending interrupt
     * these is not mandatory or a left uncleared status can mess-up next intr mode change and status check  without a prior intr clear */
    VL6180_ClearAllInterrupt(theVL6180Dev);

    /* Anover way to stop is to switch and trigger a single shot measure (in a safe way)
     * set interrupt report mode new sample ready
     * clear interrupt
     * kick of a measure
     * poll for measure ready
     * all that can take up to arround 2x max convergence time typically set to 50ms  */

    /* TODO we can also disable the output pin to save some current */

    /* disable  interrupt handling at CPU level */

}




void AlarmInit(void){
    State.mode = AlrmRun;
    TimeStarted=g_TickCnt;
    uint16_t InterMeasPeriod=10; /* 10 ms is the minimal */
    /* We assume device is stopped  */

    VL6180_Prepare(theVL6180Dev);
    /* Increase convergence time to the max (this is because proximity config of API is used) */
    VL6180_RangeSetMaxConvergenceTime(theVL6180Dev, 63);
    /* set max upscale so we can work up to some  50cm */
    VL6180_UpscaleSetScaling(theVL6180Dev, 3);

    /* set inter measurement period (that is in fact inter measure time)
     * note that when low refresh rate  is need time like 100ms is best to keep power low  */
    VL6180_RangeSetInterMeasPeriod(theVL6180Dev, InterMeasPeriod);
    /* if fast reaction is required then set a time of 0 (will set minimal possible) */
    /* VL6180_RangeSetInterMeasPeriod(theVL6180Dev, 0); */

    /* setup gpio1 pin to range interrupt output with high polarity (rising edge) */
    VL6180_SetupGPIO1(theVL6180Dev, GPIOx_SELECT_GPIO_INTERRUPT_OUTPUT, INTR_POL_HIGH);
    /* set threshold for current used case and update the display */
    AlarmUpdateUseCase();
    /* enable interrupt at CPU level */

    /*Clear any pending device interrupt even if already active it should force a new edge so we can pick up*/
    VL6180_ClearAllInterrupt(theVL6180Dev);

    /* start continuous mode */
    VL6180_RangeSetSystemMode(theVL6180Dev, MODE_START_STOP|MODE_CONTINUOUS);
    /* from now vl6180x is running on it's own and will interrupt us when condition is met
     * the interrupt set a flag peek-up in AlarmState run loop*/
}

#define AlarmKeepDispTime   250  /*  alarm message retain time after it fires */
volatile int IntrFired=0;


void AlarmState(void){
    IntrStatus_t IntStatus;
    int status;

    if (IntrFired != 0) {
        /* Interrupt did fired Get interrupt  causes */
        status = VL6180_RangeGetInterruptStatus(theVL6180Dev, &IntStatus.val);
        if (status) {
            AbortErr("Al 1");
            goto done;
        }
        switch( IntStatus.status.Range ) {
            case RES_INT_STAT_GPIO_LOW_LEVEL_THRESHOLD :
                SetDisplayString("L---");
                break;
            case RES_INT_STAT_GPIO_HIGH_LEVEL_THRESHOLD :
                SetDisplayString("H---");
                break;
            case RES_INT_STAT_GPIO_OUT_OF_WINDOW :
                SetDisplayString("O---");
                break;
            case RES_INT_STAT_GPIO_NEW_SAMPLE_READY:
                SetDisplayString("n---");
                break;
        }
        VL6180_RangeClearInterrupt(theVL6180Dev); /* clear it */
        IntrFired = 0;
        TimeStarted=g_TickCnt;
        State.AlrmFired = 1;
    }
    else{
        int flush=0;
        //sanity check we are not in a state where i/o is active without an edge
        if( g_TickCnt-TimeStarted> 5000 ){
            if( flush )
                VL6180_RangeClearInterrupt(theVL6180Dev); /* clear it */
            TimeStarted=g_TickCnt;
        }
    }
    if( State.AlrmFired ){
        /* After an interrupt fire keep the display message for some minimal time
         * over wise it could not be visible at all */
        if( g_TickCnt-TimeStarted > AlarmKeepDispTime )
            State.AlrmFired = 0;
    }
    else{
        /* show what alarm mode we are one */
        switch( State.CurAlrm ){
            case 0 :
                SetDisplayString("L"); /* low */
                break;
            case 1 :
                SetDisplayString("H"); /* high */
                break;
            case 2:
                SetDisplayString("O"); /* window */
                break;
        }
    }
    /* keep On refreshing display at every idle loop */
    done:
    ;
}

#define ScaleDispTime       800

void GoToAlaramState(void) {
    AlarmInit();
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    int status;
    int new_switch_state;
    int switch_state = -1;
    State.mode = 1;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
    VL6180_WaitDeviceBooted(theVL6180Dev);
    VL6180_InitData(theVL6180Dev);
    State.InitScale=VL6180_UpscaleGetScaling(theVL6180Dev);
    State.FilterEn=VL6180_FilterGetState(theVL6180Dev);



    /* Enable Dmax calculation only if value is displayed (to save computation power) */
    VL6180_DMaxSetState(theVL6180Dev, DMaxDispTime>0);

    switch_state=-1 ; /* force what read from switch to set new working mode */
    State.mode = RunRangePoll;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint8_t status ;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      status = VL6180_Prepare(theVL6180Dev);
      VL6180_RangeSetMaxConvergenceTime(theVL6180Dev, 20);
      VL6180_RangeSetInterMeasPeriod(theVL6180Dev, 20);
      State.InitScale=VL6180_UpscaleGetScaling(theVL6180Dev);
      if (status) {
          __NOP();
      }
      switch (State.mode) {
          case RunRangePoll:
              RangeState();
              break;

          case InitErr:
              TimeStarted = g_TickCnt;
              State.mode = WaitForReset;
              break;

          case AlrmStart:
              GoToAlaramState();
              break;

          case AlrmRun:
              AlarmState();
              break;

          case FromSwitch:
              /* force reading swicth as re-init selected mode  */
              break;

          case ScaleSwap:

              if (g_TickCnt - TimeStarted >= ScaleDispTime) {
                  State.mode = RunRangePoll;
                  TimeStarted=g_TickCnt; /* reset as used for --- to er display */
              }
              else
                  __NOP();
              break;

          default:
#if !FREE_RTOS
__NOP();
#endif
              if (g_TickCnt - TimeStarted >= 5000) {
                  NVIC_SystemReset();
              }
      }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int VL6180_I2CRead(VL6180Dev_t addr, uint8_t  *buff, uint8_t len){
    int status;
    status = HAL_I2C_Master_Receive(&hi2c1,  addr, buff, len , def_i2c_time_out);
    if( status ){
        MX_I2C1_Init();
    }

    return status;
}

int VL6180_I2CWrite(VL6180Dev_t addr, uint8_t  *buff, uint8_t len){
    int status;
    status = HAL_I2C_Master_Transmit(&hi2c1,  addr, buff, len , def_i2c_time_out);
    if( status ){
        MX_I2C1_Init();
    }
    return status;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
