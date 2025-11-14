/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * mini_car_race
  * v4.2.4
	* 速度60脉冲/1ms
	* 状态判断
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dodo_BMI270.h" //陀螺仪驱动
#include "multiplexer.h"//多路复用器驱动，用于读取光电管读数
#include "stdio.h"
#include "math.h"
#include "stdbool.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define QUEUE_SIZE 20
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
typedef struct {
    uint16_t data[QUEUE_SIZE];
    uint8_t front;  // 队头指针
    uint8_t tail;   // 队尾指针
} Queue;
void initQueue(Queue *q) {// 初始化队列
    q->front = 0;
    q->tail = 0;
}
bool isEmpty(Queue *q) {// 检查队列是否为空
    return q->front == q->tail;
}
bool isFull(Queue *q) {// 检查队列是否已满
    return (q->tail + 1) % QUEUE_SIZE == q->front;
}
bool enqueue(Queue *q, uint16_t value) {// 入队
    if (isFull(q)) {
        printf("队列已满，无法入队！\n");
			  return false;
    }
    q->data[q->tail] = value;
    q->tail = (q->tail + 1) % QUEUE_SIZE;
		return true;
}
bool dequeue(Queue *q) {// 出队
    if (isEmpty(q)) {
        printf("队列为空，无法出队！\n");
        return false;
    }
    q->front = (q->front + 1) % QUEUE_SIZE;
    return true;
}
bool getFront(Queue *q, uint16_t *value) {// 获取队头元素
    if (isEmpty(q)) {
        //printf("队列为空！\n");
        return false;
    }
    *value = q->data[q->front];
    return true;
}

// 获取队列长度
int getSize(Queue *q) {
    return (q->tail - q->front + QUEUE_SIZE) % QUEUE_SIZE;
}
//---------------------一元卡尔曼滤波------------------
typedef struct {
    float x;      // 状态变量（估计的速度/脉冲数）
    float p;      // 估计协方差
    float q;      // 过程噪声协方差（系统模型的不确定性）
    float r;      // 测量噪声协方差（传感器噪声）
    float k;      // 卡尔曼增益
} KalmanFilter1;

// 初始化函数
void Kalman_Init(KalmanFilter1 *kf, float q, float r, float initial_value) {
    kf->q = q;
    kf->r = r;
    kf->x = initial_value;
    kf->p = 1;
    kf->k = 0;
}
KalmanFilter1 encoderSpeedL,encoderSpeedR,encoderROT;//卡尔曼定义

// 卡尔曼滤波更新函数
float Kalman_Update(KalmanFilter1 *kf, float measurement) {
    kf->p = kf->p + kf->q;  // 预测
    kf->k = kf->p / (kf->p + kf->r);  // 更新卡尔曼增益
    kf->x = kf->x + kf->k * (measurement - kf->x);  // 更新估计值
    kf->p = (1 - kf->k) * kf->p;  // 更新误差协方差
    return kf->x;
}
//--------------------二元卡尔曼滤波------------------

//--------------------PID-----------------------------
#define integralLimit 20000
struct PIDController {
int16_t targetVal; // 目标
float currentError; // 当前误差
float preError; // 先前误差
float derivative; // 微分
float integral; // 积分
float output; // 输出
float Kp, Ki, Kd; // 
};

void ComputePID(struct PIDController *pid, int16_t measuredVal) {
pid->preError = pid->currentError;   //误差更新
pid->currentError = pid->targetVal - measuredVal;  
pid->derivative = pid->currentError - pid->preError;  //微分更新
pid->integral += pid->currentError;  //积分更新

if (pid->integral > integralLimit) {    //积分越界
pid->integral = integralLimit;
} else if (pid->integral < -integralLimit) {
pid->integral = -integralLimit;
}

pid->output = pid->Kp * pid->currentError + pid->Ki * pid->integral + pid->Kd * pid->derivative; //pid输出

}
float gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z;//陀螺仪数据
struct PIDController L={.Kp=50,.Ki=0.6,.Kd=0.1,.targetVal=0,.currentError=0,.preError=0,.derivative=0,.integral=0};
struct PIDController R={.Kp=50,.Ki=0.6,.Kd=0.1,.targetVal=0,.currentError=0,.preError=0,.derivative=0,.integral=0};   //PID调参
struct PIDController ROT={.Kp=1.43,.Ki=0,.Kd=3.8,.targetVal=0,.currentError=0,.preError=0,.derivative=0,.integral=0};  //转向
struct PIDController ANG={.Kp=0.2,.Ki=0,.Kd=0.02,.targetVal=0,.currentError=0,.preError=0,.derivative=0,.integral=0};  //角速度

//-------------------偏差计算-------------------------
int16_t MUX_Weight[12]={230,-170,-25,-13,-6,-4,4,6,13,25,170,230};
int16_t UARTCounter=0,OUTCounter=0,ANGCounter=0;
uint16_t current_time=0;
float speed_factor = 1.0;
bool STOPFlag=false,TURNFlag = false;
Queue qenterSAW;
#define defultSpeed 60     //[speed]默认速度
#define maxSpeed 160       //[speed]最大速度
#define maxDEV 750         //[stop]最大偏出赛道的时间
#define maxTIME 12000      //[stop]此时间后停车
#define warnANG 150        //[stop]角速度预警，大于此速度开始计时
#define maxANG 1000        //[stop]空转限，角速度连续此时间大于预警值，将停止
#define sharpROT 800       //[sharp]“急弯”态的默认MUXVal输出
#define minsharpFactor 0.04//[sharp]“急弯”态的最小输出乘数
#define dersharpFactor 0.00//[sharp]“急弯”态每ms的输出减少的比重 [用置零的方式暂时停用]
#define enterSAWcount 50   //[saw]可以判定“锯齿”态的1s内连续大幅偏移数 [合理值为2或3，用极大数的方式暂时停用]
#define enterSAWtime 400   //[saw]“锯齿”跟踪的时间长度
#define timeRECOVERING 200 //[recovering]“恢复”态时长，用于直角弯检测消抖
#define timeUART 20        //[uart]每次UART发送间隔的中断数
typedef enum {
    STRAIGHT,    //0
    GENTLE_CURVE,//1 
    SHARP_TURN,  //2
    EDGE,        //3
    RECOVERING,  //4
    SAWTOOTH,    //5  //暂时停用
	  OUTLINE_DEFAULT,//6
	  OUTLINE_SHARP//7
} STATE;

STATE current_state = STRAIGHT;
STATE last_state = STRAIGHT;int16_t LEDCounter = 0;
float sharp_factor=1.0;
uint16_t recCounter = 0;
float computeMUXVal(uint16_t *mux_value) {
	static uint8_t sawENTERlastside=0; //0无1左2右
	static int16_t SHARPlastside=0;
  static float last_reliable_error = 0;
	
	// 读取传感器并统计
	float centroid=0;
  LEDCounter = 0;
  int8_t LCounter = 0, RCounter = 0;
  int8_t Lmost = 12, Rmost = -1;
  for(int i = 0; i <= 11; i++) {
    if(MUX_GET_CHANNEL(*mux_value, i)) {
      if(i < 6) LCounter++; else RCounter++;
      if(i < Lmost) Lmost = i;
      if(i > Rmost) Rmost = i;
      LEDCounter++;
		  centroid += i;
    }
  }
	if(LEDCounter > 0) centroid/=LEDCounter;    
  
	//  SAWTOOTH(锯齿、左右震荡)进入退出
	if((Lmost == 0 || Rmost == 11)){
	  if(LCounter > RCounter && (sawENTERlastside == 2 || sawENTERlastside == 0)){
		  if(!enqueue(&qenterSAW,current_time))STOPFlag = true;
			sawENTERlastside = 1;
		}
		if(LCounter < RCounter && (sawENTERlastside == 1 || sawENTERlastside == 0)){
		  if(!enqueue(&qenterSAW,current_time))STOPFlag = true;
			sawENTERlastside = 2;
		}
	}
	if(current_state == SAWTOOTH && getSize(&qenterSAW) < enterSAWcount)current_state = STRAIGHT;
	if(getSize(&qenterSAW) >= enterSAWcount)current_state = SAWTOOTH;
	if(getSize(&qenterSAW) == 0)sawENTERlastside = 0;
	
	uint16_t qfront=0;
	if(getFront(&qenterSAW,&qfront)){ //删除关注时间以外的极点
	  if(current_time - qfront > enterSAWtime)dequeue(&qenterSAW);
	}
	
	
		//出线判断
	if(LEDCounter == 0) {
    OUTCounter++;
    if(OUTCounter > maxDEV) STOPFlag = true;
		if(current_state != SAWTOOTH && current_state != OUTLINE_SHARP && current_state != OUTLINE_DEFAULT){
			if(last_state == SHARP_TURN || last_state == RECOVERING) { //SHARP出界和SHARP消抖出界
			  current_state = OUTLINE_SHARP;
				sharp_factor = 1.0;
			}
		  else current_state = OUTLINE_DEFAULT;
		  
		}
  } else {
    OUTCounter = 0; //非出界状态时，出界计数器计0
  }
	
	  // 状态判断
  if(LEDCounter != 0 && current_state != SAWTOOTH){ //有灯亮、非锯齿时进入判定
    if(abs(LCounter-RCounter) <= 3 && LEDCounter <= 4 && Lmost != 0 && Rmost != 11) {
      current_state = STRAIGHT;
    } else if(((LCounter >= 4 && Lmost == 0 && Rmost != 11) ||  
               (RCounter >= 4 && Rmost == 11 && Lmost != 0) )
               && LEDCounter >=6		) {
      current_state = SHARP_TURN;
			if(last_state != SHARP_TURN) SHARPlastside = 0;//新的急转，转向计数置零
			SHARPlastside += LCounter > RCounter ? -1:1;//转向计数（消抖处理，防止出界最后时刻的情况不可靠）
    } else if(LEDCounter >= 3) {
      current_state = GENTLE_CURVE;
    } else if(LEDCounter >= 1) { 
      current_state = EDGE;
    } else {
      current_state = STRAIGHT; 
    }
  }
	if(LEDCounter == 12){
	  current_state = STRAIGHT;//道路交叉
		SHARPlastside = 0;
	}
		
  //状态恢复检测：从急弯转出至RECOVERING
  if((last_state == SHARP_TURN  && current_state != SHARP_TURN && current_state != OUTLINE_SHARP && current_state != OUTLINE_DEFAULT) 
		|| (last_state == OUTLINE_SHARP && current_state != OUTLINE_SHARP && current_state != OUTLINE_DEFAULT)) {
    current_state = RECOVERING;
    recCounter = 0;
  }
  // RECOVERING状态计时
  if(last_state == RECOVERING) {
    recCounter++;
		if(current_state != SHARP_TURN && current_state != OUTLINE_DEFAULT && current_state != OUTLINE_SHARP)current_state = RECOVERING; //恢复状态除新的急转和出界外维持恢复
		if(recCounter >= timeRECOVERING) { // 恢复timeRECOVERING秒后回到正常状态
      current_state = STRAIGHT;
			SHARPlastside = 0; //恢复结束，转向计数置零
    }
	}
	
	last_state = current_state;
	
  //输出计算	
  float result = 0;
	
  float base_error = (centroid - 5.5) * 50; // 基础偏差 [-275, 275]
  switch(current_state) {
    case STRAIGHT:
      result = base_error; // 正常响应
			speed_factor = 1.0;
      break;

    case GENTLE_CURVE:
      result = base_error * 2.3f; // 适度增强
	    speed_factor = 1.0;
      break;

    case SHARP_TURN:
      result = SHARPlastside < 0 ? -sharpROT : sharpROT;
			speed_factor = -0.07;
      break;
    case OUTLINE_SHARP:
			result = SHARPlastside < 0 ? -sharpROT * sharp_factor: sharpROT * sharp_factor;
		if(sharp_factor > minsharpFactor)sharp_factor = sharp_factor - dersharpFactor; //急转的转向半径随时间增大，每ms减少dersharoFactor，最小值minsharpFactor.当前der置零停用
		  speed_factor = -0.07; 
		  break;
		case OUTLINE_DEFAULT:
			result = last_reliable_error;
		  if(result > 500)result = 500;
		  else if(result <-500)result = -500;
		  speed_factor = 1.0;
		  break;
    case EDGE:
			/*
      for(int i = 0; i <= 11; i++) {
        result += MUX_GET_CHANNEL(*mux_value, i) * MUX_Weight[i];//传统方法
      }*/
			result = base_error * 2.0f;
		  speed_factor = 1.0;
      break;        

    case RECOVERING:
      result = base_error * 0.7f;
		  speed_factor = 1.0;
      break;

    case SAWTOOTH:
			
			if(LEDCounter != 0)
				/*
        for(int i = 0; i <= 11; i++) {
          result += MUX_GET_CHANNEL(*mux_value, i) * MUX_Weight[i];传统方法
        }*/
			result = base_error * 2.3f;
				else result = last_reliable_error * 1.4;
      speed_factor = 1.0;
      break;
	}
        
  // 保存可靠误差值
  if(LEDCounter > 0) {
    last_reliable_error = result;
  }
	
	//UART输出
	static float result_output=0;
	if(UARTCounter%timeUART==0){                                                                    ////////////
	  printf(" %d %d %.2f\r\n", current_state,SHARPlastside,result_output/timeUART);
		result_output = 0;
	}else{                                                                                          ////////////
		 result_output+=result;
	}
  return result;
}

//-----------------------中断回调---------------------
int16_t L_measureVal,R_measureVal,ANG_measureVal,Dir_measureVal,pwm=0;
int16_t last_pwm_L=0,last_pwm_R=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim2)
  {
		current_time++;
	  if(current_time>maxTIME)STOPFlag=true;
		//I.转向环PID
		//加权偏差663
		uint16_t mux_value;
    MUX_get_value(&mux_value);
		Dir_measureVal=Kalman_Update(&encoderROT,computeMUXVal(&mux_value));
		
			
		ComputePID(&ROT,Dir_measureVal);  //转向PID  正往左
		ANG.targetVal = ROT.output;//角速度调整
		
		//II.角速度环PID
		dodo_BMI270_get_data();  
		gyro_z = BMI270_gyro_transition(BMI270_gyro_z);
		
		if(fabs(gyro_z)>warnANG)ANGCounter++;else ANGCounter=0;  //自旋
		if(ANGCounter>maxANG)STOPFlag=true;
		
		//ANG_measureVal = Kalman_Update(&encoderSpeedANG,gyro_z);
		ComputePID(&ANG,gyro_z);  //角速度PID
		if (ANG.output > 2000.0f) ANG.output = 2000.0f;//角速度限幅
    if (ANG.output < -2000.0f) ANG.output = -2000.0f;
		L.targetVal = defultSpeed * speed_factor - ANG.output;  //轮速度调整
		R.targetVal = defultSpeed * speed_factor + ANG.output;
		if(L.targetVal>maxSpeed)L.targetVal=maxSpeed;else if(L.targetVal<-maxSpeed)L.targetVal=-maxSpeed;//轮速度限幅
		if(R.targetVal>maxSpeed)R.targetVal=maxSpeed;else if(R.targetVal<-maxSpeed)R.targetVal=-maxSpeed;
		
		//III.速度环PID
    L_measureVal = -(int16_t)__HAL_TIM_GET_COUNTER(&htim4);//获取左右轮速度
		R_measureVal = -(int16_t)__HAL_TIM_GET_COUNTER(&htim3);
	  __HAL_TIM_SET_COUNTER(&htim3, 0);
		__HAL_TIM_SET_COUNTER(&htim4, 0);
		L_measureVal= (int16_t)(Kalman_Update(&encoderSpeedL,L_measureVal));//卡尔曼滤波
		R_measureVal= (int16_t)(Kalman_Update(&encoderSpeedR,R_measureVal));
		
		if(UARTCounter%timeUART==0){                       ////////////
      //printf(" %s,0\r\n", current_);                 //串口输出//
			UARTCounter=1;                                  //        //
		}UARTCounter++;                                   ////////////
		
    ComputePID(&R, R_measureVal);  //速度PID
		ComputePID(&L, L_measureVal);
		
		pwm=L.output;
		if(STOPFlag)pwm=0;
		if(pwm-last_pwm_L>100)pwm=last_pwm_L+100;//变化率限幅
		if(pwm-last_pwm_L<-100)pwm=last_pwm_L-100;
		last_pwm_L=pwm;
    if (pwm > 3600) pwm = 3600; else if (pwm < -3600) pwm = -3600;  //输出限幅
	  if (pwm >= 0) {
      TIM1->CCR1 = pwm, HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    } else {
      TIM1->CCR1 = -pwm, HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    }
		
		pwm = R.output;
		if(STOPFlag)pwm=0;
		if(pwm-last_pwm_R>100)pwm=last_pwm_R+100;//变化率限幅
		if(pwm-last_pwm_R<-100)pwm=last_pwm_R-100;
		last_pwm_R=pwm;
    if (pwm > 3600) pwm = 3600; else if (pwm < -3600) pwm = -3600;  //输出限幅
	  if (pwm >= 0) {
      TIM1->CCR2 = pwm, HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    } else {
      TIM1->CCR2 = -pwm, HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    }
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

	HAL_Delay(3500);
	
	dodo_BMI270_init();//初始化陀螺仪
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim2);
	initQueue(&qenterSAW);
	
	Kalman_Init(&encoderSpeedL, 0.03f, 2.0f, 0.0f);//卡尔曼初始化
	Kalman_Init(&encoderSpeedR, 0.03f, 2.0f, 0.0f);
	Kalman_Init(&encoderROT, 0.5f, 100.0f, 0.0f);

	//L.targetVal=150;
	//R.targetVal=150;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /*
		L.targetVal=160;
		R.targetVal=160;
		HAL_Delay(4500);
		L.targetVal=80;
		R.targetVal=80;
		HAL_Delay(4500);
		L.targetVal=-160;
		R.targetVal=-160;
		HAL_Delay(4500);
		L.targetVal=80;
		R.targetVal=80;
		HAL_Delay(4500);*/
		//TIM1->CCR2=1000;HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		/*
    //以下为陀螺仪使用示例
    dodo_BMI270_get_data();//调用此函数会更新陀螺仪数据
    gyro_x=BMI270_gyro_transition(BMI270_gyro_x);//将原始陀螺仪数据转换为物理值，单位为度每秒
    gyro_y=BMI270_gyro_transition(BMI270_gyro_y);
    gyro_z=BMI270_gyro_transition(BMI270_gyro_z);

    accel_x=BMI270_acc_transition(BMI270_accel_x);//将原始加速度计数据转换为物理值，单位为g，一般不需要使用此数据
    accel_y=BMI270_acc_transition(BMI270_accel_y);
    accel_z=BMI270_acc_transition(BMI270_accel_z);
    printf("G: %f %f %f | A: %f %f %f\r\n", gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z);//输出陀螺仪读数，测试是否成功启动，正常使用时不需要这行代码

    //以下为读取光电管的示例（从左到右编号0~11）
    uint16_t mux_value;
    MUX_get_value(&mux_value);
    for(int i=0;i<=11;i++){
      printf("%d,",MUX_GET_CHANNEL(mux_value,i));//获取第i个光电管的数值并输出
    }
    printf("\n");*/
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
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
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 7199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|R_DIR_Pin|MUX_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L_DIR_Pin|MUX_1_Pin|MUX_2_Pin|MUX_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : L_DIR_Pin */
  GPIO_InitStruct.Pin = L_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(L_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : R_DIR_Pin */
  GPIO_InitStruct.Pin = R_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(R_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MUX_READ_Pin */
  GPIO_InitStruct.Pin = MUX_READ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MUX_READ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MUX_0_Pin */
  GPIO_InitStruct.Pin = MUX_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(MUX_0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MUX_1_Pin MUX_2_Pin MUX_3_Pin */
  GPIO_InitStruct.Pin = MUX_1_Pin|MUX_2_Pin|MUX_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
