/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "MPU6050.h"
#include "math.h"
#include "API.h"

#include "queue.h"
#include "queue_int.h"
#include "stack.h"
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
double distance_straight = 0;
double distance_left = 0;
double distance_right = 0;

char Buffer[25] = {0};

MPU6050_t MPU6050;

double roll,pitch,yaw, froll, fpitch, fyaw = 0;
uint32_t elapsedTime, currentTime, previousTime;
double dt = 0.001;

uint32_t counterTim2 = 0;
int16_t countTim2 = 0;

uint32_t counterTim3 = 0;
int16_t countTim3 = 0;

uint8_t flag = 0;

const  int rows=16;
const  int cols=16;
const int dx[] = {1, -1, 0, 0};
const int dy[] = {0, 0, -1, 1};

wall_maze maze;
Queue myQueue;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

void angle(int goc){
	if(goc > 0){
		while(1){
			previousTime = HAL_GetTick();
			MPU6050_Read_Gyro(&hi2c1, &MPU6050);
			roll += (MPU6050.Gx - froll) *dt;
			pitch += (MPU6050.Gy - fpitch) *dt;
			yaw += (MPU6050.Gz - fyaw) *dt;
			currentTime = HAL_GetTick();
			elapsedTime = currentTime - previousTime;
			dt = ((double) elapsedTime) / 1000;
			if(yaw >= goc){
				roll = 0;
				pitch = 0;
				yaw = 0;
				return;
			}
		}
	}
	else{
		while(1){
			previousTime = HAL_GetTick();
			MPU6050_Read_Gyro(&hi2c1, &MPU6050);
			roll += (MPU6050.Gx - froll) *dt;
			pitch += (MPU6050.Gy - fpitch) *dt;
			yaw += (MPU6050.Gz - fyaw) *dt;
			currentTime = HAL_GetTick();
			elapsedTime = currentTime - previousTime;
			dt = ((double) elapsedTime) / 1000;
			if(yaw <= goc){
				roll = 0;
				pitch = 0;
				yaw = 0;
				return;
			}
		}
	}
}
double distanceLeft(){
	uint16_t ADC_VAL;
	double vol = 0;
	uint16_t total = 0;
  	for (uint8_t i = 0; i < 5; i++){
  	  	HAL_ADC_Start(&hadc3);
  	  	HAL_ADC_PollForConversion(&hadc3, 1000);
  	  	ADC_VAL = HAL_ADC_GetValue(&hadc3);
  	  	HAL_ADC_Stop(&hadc3);
  	    total = total + ADC_VAL;
  	}

  	vol = (total/5*3.3)/4095;
  	return 13 * pow(vol, -1) - 0.7;
}
double distanceStraight(){
	uint16_t ADC_VAL;
	double vol = 0;
	uint16_t total = 0;
  	for (uint8_t i = 0; i < 5; i++){
  	  	HAL_ADC_Start(&hadc2);
  	  	HAL_ADC_PollForConversion(&hadc2, 1000);
  	  	ADC_VAL = HAL_ADC_GetValue(&hadc2);
  	  	HAL_ADC_Stop(&hadc2);
  	    total = total + ADC_VAL;
  	}

  	vol = (total/5*3.3)/4095;
  	return 13 * pow(vol, -1)- 0.7;
}
double distanceRight(){

	uint16_t ADC_VAL;
	double vol = 0;
	uint16_t total = 0;
  	for (uint8_t i = 0; i < 5; i++){
  		HAL_ADC_Start(&hadc1);
  	  	HAL_ADC_PollForConversion(&hadc1, 1000);
  	  	ADC_VAL = HAL_ADC_GetValue(&hadc1);
  	    total = total + ADC_VAL;
  	    HAL_ADC_Stop(&hadc1);
  	}
  	vol = (total/5*3.3)/4095;
  	return 13 * pow(vol, -1) - 0.7;
}
void filter_gyro(){
	printf("Start probe filter\r\n");
	HAL_Delay(500);
	for(uint8_t x = 0; x < 10; x++){
		printf("*");
	    HAL_Delay(100);
	}
	printf("*\r\n");
	for(int x = 0; x < 1000; x++){
		MPU6050_Read_Gyro(&hi2c1, &MPU6050);
		froll += MPU6050.Gx;
		fpitch += MPU6050.Gy;
		fyaw += MPU6050.Gz;
	}
	froll = froll / 1000;
	fpitch = fpitch /1000;
	fyaw = fyaw/1000;
	printf("froll: %.2f fpitch: %.2f fyaw: %.2f\r\n",froll, fpitch, fyaw);
	printf("Prove filter done!\r\n");

}

bool isValid(int x, int y) {
    return (x >= 0 && x < rows && y >= 0 && y < cols);
}
void init_arr(int arr[ROW][COL], int row, int col) {
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < col; j++) {
            arr[i][j] = -1;
        }
    }
}
void check_and_fill(int arr[ROW][COL],int row,int col,int value)
{
    if(row<0 ||col<0||row>=16||col>=16||arr[row][col]!=-1)return;
    value+=1;
    coord point={row,col,value};
    pushQueue(&myQueue,point);
    arr[row][col]=value;
}

void init_flood(int arr[ROW][COL],int row,int col)
{
    int count_=0;
    coord point={row,col,count_};
    pushQueue(&myQueue,point);
    arr[row][col]=0;
    coord point2={row+1,col,count_};
    pushQueue(&myQueue,point2);
    arr[row+1][col]=0;
    coord point3={row,col+1,count_};
    pushQueue(&myQueue,point3);
    arr[row][col+1]=0;
    coord point4={row+1,col+1,count_};
    pushQueue(&myQueue,point4);
    arr[row+1][col+1]=0;
    while (!isEmptyQueue(&myQueue)) {
        coord frontCoord = peekQueue(&myQueue);
        popQueue(&myQueue);
        check_and_fill(arr,frontCoord.row+1,frontCoord.col,frontCoord.value);
        check_and_fill(arr,frontCoord.row-1,frontCoord.col,frontCoord.value);
        check_and_fill(arr,frontCoord.row,frontCoord.col+1,frontCoord.value);
        check_and_fill(arr,frontCoord.row,frontCoord.col-1,frontCoord.value);
    }
}

void init_maze()
{
    for(int i =0;i<16;i++)
    {
        for(int j=0;j<16;j++)
        {
            maze.cells[i][j].visited=0;
            maze.cells[i][j].angle_update=90;
            maze.cells[i][j].dead=0;
            for(int k = 0 ;k<4;k++)maze.cells[i][j].walls[k]=0;
        }
    }
}

void intToStr(int num, char *str) {
    sprintf(str, "%d", num);
}
cell_info cell_direction_adjust(cell_info cell)
{
    cell_info cell_new;
    cell_new=cell;

    for(int i=0;i<4;i++)
    {
        int ind = i;

        switch(cell.angle_update)
        {
            case 90:
                break;
            case 270:
                if(i%2==0)ind+=1;
                else ind-=1;
                break;
            case 0:
                if(i==0 || i ==1)ind+=2;
                else if(i==2)ind=1;
                else ind=0;
                break;
            case 180:
                if(i==2 || i ==3)ind-=2;
                else if(i==0)ind=3;
                else ind=2;
                break;
        }
        cell_new.walls[i]=cell.walls[ind];
    }
    return cell_new;
}

cell_info update_walls(int angle_now,int row,int col)
{
    cell_info new_cell;
    new_cell.angle_update=angle_now;
    double straight = distanceStraight();
    bool boolfront = 0;
    bool boolleft = 0;
    bool boolright = 0;

	if(HAL_GPIO_ReadPin(Sensor_Left_GPIO_Port, Sensor_Left_Pin) == 0){
		boolleft = 1;

	}
	if(HAL_GPIO_ReadPin(Sensor_Right_GPIO_Port, Sensor_Right_Pin) == 0){
		boolright = 1;
	}

    if(straight < 10){
    	boolfront = 1;
    }


    new_cell.walls[UP]=boolfront;
    new_cell.walls[DOWN]=0;
    new_cell.walls[LEFT]=boolleft;
    new_cell.walls[RIGHT]=boolright;
    new_cell.dead=0;
    new_cell.visited=1;
    maze.cells[row][col]=cell_direction_adjust(new_cell);
    if(new_cell.walls[UP]==1&&new_cell.walls[LEFT]==1&&new_cell.walls[RIGHT]==1&&row!=0&&col!=0)
    {
        maze.cells[row][col].dead=1;
    }
    for(int i=0;i<4;i++)
    {
        int newRow=row+dy[i];
        int newCol=col+dx[i];
        if(isValid(newRow,newCol))
        {
            if(i==UP)maze.cells[newRow][newCol].walls[DOWN]=maze.cells[row][col].walls[UP];
            else if(i==LEFT)maze.cells[newRow][newCol].walls[RIGHT]=maze.cells[row][col].walls[LEFT];
            else if(i==RIGHT)maze.cells[newRow][newCol].walls[LEFT]=maze.cells[row][col].walls[RIGHT];
        }
    }
    return new_cell;
}
void go_to_cell(int *angle_now,int dir)
{
    switch(dir)
            {
                case -1:
                    break;
                case UP:
                    API_moveForward();
                    break;
                case DOWN:
                    *angle_now-=180;
                    API_turnRight();
                    API_turnRight();
                    API_moveForward();
                    break;
                case LEFT:
                    *angle_now+=90;
                    API_turnLeft();
                    API_moveForward();
                    break;
                case RIGHT:
                    *angle_now-=90;
                    API_turnRight();
                    API_moveForward();
                    break;
                default:
                    break;
            }
            *angle_now = *angle_now % 360;
            // �?ảm bảo góc không bị âm
            if (*angle_now < 0) {
                *angle_now += 360;
            }
}
bool check_wall_angle(cell_info cell,int *dir)
{
    switch(cell.angle_update)
    {
        case 90:
            break;
        case 270:
            if(*dir%2==0)*dir+=1;
            else *dir-=1;
            break;
        case 0:
            if(*dir==0 || *dir ==1)*dir+=2;
            else if(*dir==2)*dir=1;
            else *dir=0;
            break;
        case 180:
             if(*dir==2 || *dir ==3)*dir-=2;
            else if(*dir==0)*dir=3;
            else *dir=2;
            break;
    }
    return cell.walls[*dir];
}
coord get_min_neighbour(cell_info cell_wall,coord cur, int (*arr)[ROW][COL],bool change_)
{
    int min_neightbor=255;
    coord next_step;
    next_step.value=-1;
    int ind;
    for (int dir = 0; dir < 4; ++dir) {
        int newRow = cur.row + dy[dir]; // 0 0 -1 1
        int newCol = cur.col + dx[dir]; //1 -1 0 0
        ind=dir;
        bool check_=cell_wall.walls[dir];
        if(change_)check_=check_wall_angle(cell_wall,&ind);

        if(isValid(newRow,newCol) && !check_)
        {
            if((*arr)[newRow][newCol]<=min_neightbor)
            {
                min_neightbor=(*arr)[newRow][newCol];
                next_step.row=newRow;
                next_step.col=newCol;
                next_step.value=ind;
            }
        }
    }
    return next_step;
}
void flood(Stack *stack_flood,int (*arr)[ROW][COL])
{
    // log_out("flood");
    coord cur_stack;
    coord next_step;

    while(!isEmptyStack(stack_flood))
    {

        cur_stack=peekStack(stack_flood);
        popStack(stack_flood);
        int min_neightbor=255;
        bool check_;

        next_step=get_min_neighbour(maze.cells[cur_stack.row][cur_stack.col],cur_stack,arr,0);

        min_neightbor=(*arr)[next_step.row][next_step.col];
        if((*arr)[cur_stack.row][cur_stack.col]-1 != min_neightbor )
        {
            for(int i =0 ;i<4;i++)
            {
                coord cur_add;
                cur_add.row= cur_stack.row + dy[i]; // 0 0 -1 1
                cur_add.col= cur_stack.col + dx[i]; //1 -1 0 0
                check_=maze.cells[cur_stack.row][cur_stack.col].walls[i];
                if(isValid(cur_add.row,cur_add.col) &&(*arr)[cur_add.row][cur_add.col]!=0&&!check_)
                {
                    pushStack(stack_flood,cur_add);
                }
            }
            if((*arr)[cur_stack.row][cur_stack.col]!=0)(*arr)[cur_stack.row][cur_stack.col]=min_neightbor+1;
        }
        int stack_size=sizeStack(stack_flood);
        if(stack_size>=35){
            for(int i=0;i<stack_size;i++)
            {
                popStack(stack_flood);
            }
            return;
        }
    }
}
coord floodfill(coord start,coord dest,int (*arr)[ROW][COL],int *angle_now)
{
    Queue path_queue;
    initializeQueue(&path_queue);

    pushQueue(&path_queue,start);
    coord cur=start;
    cell_info new_cell;

    Stack stack_flood;
    initializeStack(&stack_flood);

    pushStack(&stack_flood,start);

    int path_distance_value_find=0;
    // int save_row,save_col;
    coord next_step;

    while(1)
    {
        if(!isEmptyQueue(&path_queue)) // dua ra quyet dinh va go
        {
            cur = peekQueue(&path_queue);

            new_cell=update_walls(*angle_now,cur.row,cur.col);

            if((*arr)[cur.row][cur.col]==(*arr)[dest.row][dest.col]){
                break;
            }
            flood(&stack_flood,arr);
            popQueue(&path_queue);
            next_step=get_min_neighbour(new_cell,cur,arr,1);
            pushQueue(&path_queue,next_step);
            pushStack(&stack_flood,next_step);
            go_to_cell(angle_now,next_step.value);
            path_distance_value_find++;
        }
        else{
            break;
        }
    }

    while(!isEmptyQueue(&path_queue)) popQueue(&path_queue);

    coord p_return={next_step.row,next_step.col,0};
    return p_return;
}
void init_flood_start(int (*arr)[ROW][COL],int row_,int col_,int back_)
{
    int count_=0;
    for(int i=0;i<16;i++)
    {
        for(int j = 0 ;j<16;j++)
        {
            (*arr)[i][j]=-1;
            if(back_==2&&maze.cells[i][j].visited==false){
                (*arr)[i][j]=255;
                maze.cells[i][j].dead=true;
            }
        }
    }
    if(back_!=1)
    {
        coord point2={row_+1,col_,count_};
        pushQueue(&myQueue,point2);
        (*arr)[row_+1][col_]=0;
        coord point3={row_,col_+1,count_};
        pushQueue(&myQueue,point3);
        (*arr)[row_][col_+1]=0;
        coord point4={row_+1,col_+1,count_};
        pushQueue(&myQueue,point4);
        (*arr)[row_+1][col_+1]=0;
    }
    coord point={row_,col_,count_};
    pushQueue(&myQueue,point);
    (*arr)[row_][col_]=0;
    while(!isEmptyQueue(&myQueue))
    {
        coord frontCoord = peekQueue(&myQueue);
        popQueue(&myQueue);
          for (int i = 0; i < 4; ++i) {
                int newRow = frontCoord.row + dy[i]; // 0 0 -1 1
                int newCol = frontCoord.col + dx[i]; //1 -1 0 0
                bool check_=maze.cells[frontCoord.row][frontCoord.col].walls[i];
                if(!check_)check_and_fill(*arr,newRow,newCol,frontCoord.value);
          }
          if(sizeQueue(&myQueue)>120){
            break;
          }
    }
}

void shorted_path_go(int (*arr)[ROW][COL],int angle_now,coord start,coord dest)
{
    QueueInt next_dir_path;
    initializeQueueInt(&next_dir_path);

    int save_row,save_col;
    coord cur=start;
    for(int i=0;i<(*arr)[start.row][start.col];i++)
    {
            int next_dir=-1;
            int newRow;
            int newCol;
            for (int dir = 0; dir < 4; ++dir) {
                newRow = cur.row + dy[dir]; // 0 0 -1 1
                newCol = cur.col + dx[dir]; //1 -1 0 0
                bool check_=maze.cells[cur.row][cur.col].walls[dir];
                if(isValid(newRow,newCol) && !check_)
                {
                    if((*arr)[newRow][newCol]<(*arr)[cur.row][cur.col])
                    {
                        next_dir=dir;
                        save_row=newRow;
                        save_col=newCol;
                    }
                }
            }
            if(next_dir!=-1)
            {
                cur.row=save_row;
                cur.col=save_col;
                pushQueueInt(&next_dir_path,next_dir);
                char value[20];
                intToStr((*arr)[save_row][save_col], value);

            }
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        counterTim2 = __HAL_TIM_GET_COUNTER(htim);
        countTim2 = (int16_t)counterTim2;
    }
    else if (htim->Instance == TIM3)
    {
        counterTim3 = __HAL_TIM_GET_COUNTER(htim);
        countTim3 = (int16_t)counterTim3;
    }
}

uint16_t ADC_VALUE[3];
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

//  while (1)
//  {
//		HAL_ADC_Start(&hadc1);
//	  	HAL_ADC_PollForConversion(&hadc1, 1000);
//	  	ADC_VALUE[0] = HAL_ADC_GetValue(&hadc1);
//	    HAL_ADC_Stop(&hadc1);
//
//		HAL_ADC_Start(&hadc2);
//	  	HAL_ADC_PollForConversion(&hadc2, 1000);
//	  	ADC_VALUE[1] = HAL_ADC_GetValue(&hadc2);
//	    HAL_ADC_Stop(&hadc2);
//
//		HAL_ADC_Start(&hadc3);
//	  	HAL_ADC_PollForConversion(&hadc3, 1000);
//	  	ADC_VALUE[2] = HAL_ADC_GetValue(&hadc3);
//	    HAL_ADC_Stop(&hadc3);
//  }

//    while(1){
//  	  distance_straight = distanceStraight();
//  	  distance_left = distanceLeft();
//  	  distance_right = distanceRight();
//    }

//  while(1){
//	  distance_straight = distanceStraight();
//	  if(HAL_GPIO_ReadPin(Sensor_Left_GPIO_Port, Sensor_Left_Pin) == 0){
//			HAL_GPIO_TogglePin(LED5_GPIO_Port, LED5_Pin);
//
//	  }
//	  if(HAL_GPIO_ReadPin(Sensor_Right_GPIO_Port, Sensor_Right_Pin) == 0){
//		  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
//	  }
//
//  }
//  while(1){
//	  	if(HAL_GPIO_ReadPin(Sensor_Left_GPIO_Port, Sensor_Left_Pin) == 0){
//			timer_Left(10);
//			HAL_GPIO_TogglePin(LED5_GPIO_Port, LED5_Pin);
//			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);
//			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
//		}
//		else if(HAL_GPIO_ReadPin(Sensor_Right_GPIO_Port, Sensor_Right_Pin) == 0){
//			timer_Right(10);
//			HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
//			HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, RESET);
//			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);
//		}
//  }


//  API_moveForward();
//	forwardGPIO();
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 10000); //left motor
//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 10500); //right motor
//	HAL_Delay(1000);
//	stop();
//	  while(1);


//  API_moveForward();
//  while(1);

  while (MPU6050_Init(&hi2c1) == 1);
  filter_gyro();
//  while(1){
//		previousTime = HAL_GetTick();
//		MPU6050_Read_Gyro(&hi2c1, &MPU6050);
//		roll += (MPU6050.Gx - froll) *dt;
//		pitch += (MPU6050.Gy - fpitch) *dt;
//		yaw += (MPU6050.Gz - fyaw) *dt;
//		currentTime = HAL_GetTick();
//		elapsedTime = currentTime - previousTime;
//		dt = ((double) elapsedTime) / 1000;
//  }

  initializeQueue(&myQueue);
  int arr[ROW][COL];

  init_arr(arr,ROW,COL);
  init_flood(arr,7,7);
  init_maze();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  coord start={0,0,arr[0][0]};
  coord dest={7,7,arr[7][7]};
  int angle_now=90;
  coord new_coord;
  new_coord = floodfill(start,dest,&arr,&angle_now);

  init_flood_start(&arr,0,0,1);
  new_coord=floodfill(new_coord,start,&arr,&angle_now);
  init_flood_start(&arr,7,7,2);
  shorted_path_go(&arr,angle_now,new_coord,dest);
  floodfill(start,dest,&arr,&angle_now);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim1.Init.Prescaler = 18-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000-1;
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
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 18-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED2_Pin|LED1_Pin|STBY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, AI2_Pin|AI1_Pin|BI1_Pin|BI2_Pin
                          |LED5_Pin|LED6_Pin|LED3_Pin|LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED2_Pin LED1_Pin STBY_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED1_Pin|STBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : AI2_Pin AI1_Pin BI1_Pin BI2_Pin
                           LED5_Pin LED6_Pin LED3_Pin LED4_Pin */
  GPIO_InitStruct.Pin = AI2_Pin|AI1_Pin|BI1_Pin|BI2_Pin
                          |LED5_Pin|LED6_Pin|LED3_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SW1_Pin SW2_Pin */
  GPIO_InitStruct.Pin = SW1_Pin|SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Sensor_Right_Pin Sensor_Left_Pin */
  GPIO_InitStruct.Pin = Sensor_Right_Pin|Sensor_Left_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
