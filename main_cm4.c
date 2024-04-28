#include <project.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sensors.h>
#include <inttypes.h>
#include <tty_utils.h>
#include <clcd.h>
#include <func_list.h>
#include <limits.h>

/***************************************
*     Global variables
***************************************/
static cy_stc_scb_i2c_master_xfer_config_t register_setting;
static uint8 rbuff[2]; // Read buffer
static uint8 wbuff[2]; // Write buffer

// altimeter variables
uint8_t xm;
uint8_t xc;
uint8_t xl;
int off;
uint8_t x;
uint8_t tm;
uint8_t tl;
uint8_t STA;
int8 n;
float pressure;
float altitude;
float temp;
float temperatureAlt;

int joe = 0;

// humidity sensor variables
uint16_t humidityRaw;

// temp sensor variables
uint8_t tempMSB;
uint8_t tempLSB;
uint16_t temperature;

// magnet sensor variables
int8_t XL;
int8_t XH;
int8_t YL;
int8_t YH;
int8_t ZL;
int8_t ZH; 
//LIS3DH
float x_accl = 0;
float y_accl = 0;
float z_accl = 0;
int xa;
int ya;
int za;
float runtime = 0;
float resttime =0;
int totalTime = 0;

int16_t xMagnetometerData;
int16_t yMagnetometerData;
int16_t zMagnetometerData;
int count = 0;
double heading;
double averageMag = 0;

// IR temp. sensor
uint32_t rawObjTemp;
float objTempC;
float objTempF;

// other
int flag = 0;
int buttonStatus[5] = {0,0,0,0,0};


/*******************************************************************************
* Button Interrupt Handlers
*******************************************************************************/
void button1Handler(void){ //P9.0
    NVIC_ClearPendingIRQ(Button1_Int_cfg.intrSrc);
    flag = 1;
    printf("INT1\r\n");
}
void button2Handler(void){ //P9.1
    NVIC_ClearPendingIRQ(Button2_Int_cfg.intrSrc);
    flag = 2;
    printf("INT2\r\n"); 
}
void button3Handler(void){ //P9.2
    NVIC_ClearPendingIRQ(Button3_Int_cfg.intrSrc);
    flag = 3;
    printf("INT3\r\n"); 
}
void button4Handler(void){ //P9.3
    NVIC_ClearPendingIRQ(Button4_Int_cfg.intrSrc);
    flag = 4;
    printf("INT4\r\n"); 
}
void button5Handler(void){ //P9.4
    NVIC_ClearPendingIRQ(Button5_Int_cfg.intrSrc);
    flag = 5;
    printf("INT5\r\n"); 
}

/*******************************************************************************
* Function Name: WaitForOperation
*******************************************************************************/
static void WaitForOperation(){ // function that chec to make sure either a rice or read function has complted
    while(0 != (I2C_MasterGetStatus() & CY_SCB_I2C_MASTER_BUSY)){}
    {
        CyDelayUs(1); //keep waiting 
    }
}

/*******************************************************************************
* Function Name: WriteRegister()
*******************************************************************************/
static void WriteRegister(uint8 reg_addr, uint8 data){
    wbuff[0] = reg_addr; 
    wbuff[1] = data;
    
    register_setting.buffer = wbuff;
    register_setting.bufferSize = 2;
    register_setting.xferPending = false;
    
    I2C_MasterWrite(&register_setting);
    WaitForOperation();
}

/*******************************************************************************
* Function Name: ReadRegister()
*******************************************************************************/
static uint8 ReadRegister(uint8 reg_addr){
    wbuff[0] = reg_addr;
    
    register_setting.buffer = wbuff;
    register_setting.bufferSize = 1;
    register_setting.xferPending = true;
    
    I2C_MasterWrite(&register_setting);
    WaitForOperation();
    
    register_setting.buffer = rbuff;
    register_setting.xferPending = false;
    
    I2C_MasterRead(&register_setting);
    WaitForOperation();
    
    return rbuff[0];
}

/*******************************************************************************
* Turn ON / OFF Sensors
*******************************************************************************/
void sensorOFF(int sensorAddress){
    switch(sensorAddress){
        case ADT7410_ADDR:
            register_setting.slaveAddress = ADT7410_ADDR;
            WriteRegister(ADT7410_CONFIG,0b11100000);
            break;
        case MLX90614_DEFAULT_ADDRESS:
            break;
        case LIS3MDL_ADDR1:
            register_setting.slaveAddress = LIS3MDL_ADDR1;
            WriteRegister(LIS3MDL_CTRL_REG3,0b00000011);
            break;
        case HTU31D_ADDR:
            // Uses Load Switch
            Cy_GPIO_Write(LoadSwitch1_0_PORT,LoadSwitch1_0_NUM,1);
            break;
        case ALTIMETER_ADDR:
            // Uses Load Switch
            Cy_GPIO_Write(LoadSwitch2_0_PORT,LoadSwitch2_0_NUM,1);
            break;
    }
}

void sensorON(int sensorAddress){
    switch(sensorAddress){
        case ADT7410_ADDR:
            register_setting.slaveAddress = ADT7410_ADDR;
            WriteRegister(ADT7410_CONFIG,0b10000000);
            break;
        case MLX90614_DEFAULT_ADDRESS:
            break;
        case LIS3MDL_ADDR1:
            register_setting.slaveAddress = LIS3MDL_ADDR1;
            WriteRegister(LIS3MDL_CTRL_REG3,0b00000011);
            break;
        case HTU31D_ADDR:
            // Uses Load Switch
            Cy_GPIO_Write(LoadSwitch1_0_PORT,LoadSwitch1_0_NUM,0);
            break;
        case ALTIMETER_ADDR:
            // Uses Load Switch
            Cy_GPIO_Write(LoadSwitch2_0_PORT,LoadSwitch2_0_NUM,0);
            break;
    }
}

/*******************************************************************************
* Function Name: changeButtonStatus()
*******************************************************************************/
void changeButtonStatus(char arr[]){
    int sensorNum = -1;
    if(!strcmp(arr,"Temperature")){
        sensorNum = 0;
    }
    else if(!strcmp(arr,"IRTemp")){
        sensorNum = 1;
    }
    else if(!strcmp(arr,"Magnet")){
        sensorNum = 2;
    }
    else if(!strcmp(arr,"Humidity")){
        sensorNum = 3;
    }
    else if(!strcmp(arr,"Altitude")){
        sensorNum = 4;
    }
    
    for(int i = 0; i < 5; i++){
        if(i == sensorNum){
            buttonStatus[i] = 1;
        }
        else{
            buttonStatus[i] = 0;
        }
    }
}

/*******************************************************************************
* LCD Functions
*******************************************************************************/
void do_pos(int line, int col){
    // Moves the position of the cursor.
    int addr;
    if (line < 1) { 
        line = 1;
    }
    if (line > CLCD_NUM_ROWS) {
        line = CLCD_NUM_ROWS;
    }
    if (col < 1) {
        col = 1;
    } 
    if (col > CLCD_NUM_COLS) {
        col = CLCD_NUM_COLS ;
    }
    addr = 0x40 * (line-1) + (col-1);
    CLCD_SetDDRAMAddr(addr) ;
}

static void format_sensor_data(float data, char* buffer, int buffer_size){    
    // Convert sensor data number to string
    snprintf(buffer, buffer_size, "%.2f", data); // string with 2 decimal places
}

 int foo = 0;
float deez = 0.0;
void write_sensor(float data){ 
    // Function to write sensor data to LCD
    char sensor_data_str[16];
    format_sensor_data(data, sensor_data_str, sizeof(sensor_data_str));
    //CLCD_PutString(sensor_data_str);
    
    switch(flag){
       
        case 1:
            // Ambient Temperature
          
            
               CLCD_PutString("Ambient Temp.");
                do_pos(2,1);
                CLCD_PutString(sensor_data_str);
                CLCD_PutString("\337F"); 
            
          
            
            
            break;
        case 2:
            // IR Object Temperature
            // Accelerometer
          
            if(joe == 0){
                CLCD_PutString("Speed.");
                CLCD_PutString(sensor_data_str);
                //CLCD_PutString("Object Temp.");
                do_pos(2,1);
                //CLCD_PutString(sensor_data_str);
                CLCD_PutString(" m/s");
                joe +=1;
                deez = data;
            }
            else if(joe == 1){
               CLCD_PutString("Avg Speed.");
                CLCD_PutString(sensor_data_str);
                //CLCD_PutString("Object Temp.");
                do_pos(2,1);
                //CLCD_PutString(sensor_data_str);
                CLCD_PutString(" m/s");
                joe +=1; 
            }
            else if(joe == 2){
                CLCD_PutString("Dog Activity.");
                if(deez >= (1.8)){
                    do_pos(2,1);
                    CLCD_PutString("Active");  
                }
                else{
                    do_pos(2,1);
                    CLCD_PutString("Rest"); 
                }
                joe =0; 
            }
            break;
        case 3:
            // Magnetometer
            
            CLCD_PutString(sensor_data_str);
            CLCD_PutString("\337");
            break;
        case 4:
            // Humidity
            
            CLCD_PutString("Humidity");
            do_pos(2,1);
            CLCD_PutString(sensor_data_str);
            CLCD_PutString("%");
            break;
        case 5:
            // Altitude
            
            CLCD_PutString("Altitude");
            do_pos(2,1);
            CLCD_PutString(sensor_data_str);
            CLCD_PutString(" meters");
            break;
    }
}

/*******************************************************************************
* Function Name: CheckSensorIdentity()
*******************************************************************************/
int CheckSensorIdentity(uint16_t WHO_AM_I_REG_ADDR, uint16_t DEVICE_ID){
    uint8 whoAmIValue = ReadRegister(WHO_AM_I_REG_ADDR);

    if (whoAmIValue == DEVICE_ID){
        printf("\r\nSensor detected. Who Am I value: 0x%X\r\n", whoAmIValue);
        return 1;
    }
    else{
        printf("\r\nSensor not detected: 0x%X\r\n", whoAmIValue);
        return 0;
    }
}


/*******************************************************************************
* Temperature Initialize and Active Functions
*******************************************************************************/
void tempInitialize(void){
    // Turn OFF other sensors. Turn ON Ambient Temp sensor.
    sensorON(ADT7410_ADDR);
    //sensorOFF(LIS3MDL_ADDR1);
    sensorOFF(HTU31D_ADDR);
    sensorOFF(ALTIMETER_ADDR);
    
    register_setting.slaveAddress = ADT7410_ADDR;
    
    //0b10000000 (Continuous Reading and 16 bit resolution).
    WriteRegister(ADT7410_CONFIG, 0x80); 
    WriteRegister(ADT7410_TEMPMSB,0x00);
    WriteRegister(ADT7410_TEMPLSB,0x00);
    changeButtonStatus("Temperature");
}
void tempActive(void){
    tempMSB = ReadRegister(ADT7410_TEMPMSB);
    tempLSB = ReadRegister(ADT7410_TEMPLSB);
    temperature = (tempMSB << 8)  | tempLSB;
        
    printf("MSB = %d\r\n",tempMSB);
    printf("LSB = %d\r\n",tempLSB);
        
    float tempCelsius = 0;
    if (temperature > 0x8000){
        tempCelsius = ((float)temperature - 65536)/128;
    }
    else {
        tempCelsius = ((float)temperature)/128;
    } 
   
    printf("Temperature: %.2f degrees C\r\n", tempCelsius);
    printf("Temperature: %.2f degrees F\r\n\n", (tempCelsius * (9.0/5.0)) + 32.0);
    CLCD_Clear();
    write_sensor((float)((tempCelsius * (9.0/5.0)) + 32.0)); 
}

/*******************************************************************************
* IR Temperature Initialize and Active Functions
*******************************************************************************/
void IRtempIntialize(void){
    // Turn OFFother sensors. Turn ON IR Temp sensor.
    sensorOFF(ADT7410_ADDR);
    //sensorOFF(LIS3MDL_ADDR1);
    sensorOFF(HTU31D_ADDR);
    sensorOFF(ALTIMETER_ADDR);
    
    changeButtonStatus("IRTemp");
    register_setting.slaveAddress = MLX90614_DEFAULT_ADDRESS;  
}

void IRtempActive(void){
    wbuff[0] = MLX90614_REGISTER_TOBJ1;

    register_setting.buffer = wbuff;
    register_setting.bufferSize = 1;
    register_setting.xferPending = true;

    I2C_MasterWrite(&register_setting);
    WaitForOperation();

    register_setting.buffer = rbuff;
    register_setting.bufferSize = 2; // Increase the read buffer size to get 2 bytes.
    register_setting.xferPending = false;

    I2C_MasterRead(&register_setting);
    WaitForOperation();
    rawObjTemp = (double) ((rbuff[1] << 8) | rbuff[0]);
    objTempC = (rawObjTemp * 0.02) - 273.15;
    objTempF = objTempC * (9.0/5.0) + 32.0;
    printf("Object Temp: %0.2lfC\r\n",objTempC);
    printf("Object Temp: %0.2lfF\r\n\n",objTempF); 
    CLCD_Clear();
    write_sensor((float)objTempF); 
}

/*******************************************************************************
* Function Name: CalculateHeading
********************************************************************************
*
* Calculates the heading (direction) in degrees for the compass
*
*******************************************************************************/
double CalculateHeading(int16_t x, int16_t y){
    double heading = atan2(y,x) * (180/M_PI);    //cal heading in degree for compass
    if(heading < 0){
        heading += 360;
    }
    return heading; 
}

/*******************************************************************************
* Magnetometer Initialize and Active Functions
*******************************************************************************/
void magnetInitialize(void){
    // Turn off other sensors.
    sensorOFF(ADT7410_ADDR);
    //sensorON(LIS3MDL_ADDR1);
    sensorOFF(HTU31D_ADDR);
    sensorOFF(ALTIMETER_ADDR);
    
    changeButtonStatus("Magnet");
    
    register_setting.slaveAddress = LIS3MDL_ADDR1;
    
    WriteRegister(LIS3MDL_CTRL_REG1, 0xFC); //set ODR to 80Hz and ultra perfomance mode for x and y.
    WriteRegister(LIS3MDL_CTRL_REG2, 0b01000000); //configure the sensor to be more sensitive with +-12 gauss  
    CyDelay(100);
    WriteRegister(LIS3MDL_CTRL_REG3, 0b00000000); //change from power down to continuous operating mode
    WriteRegister(LIS3MDL_CTRL_REG4, 0b00001100); //set zaxis to ultra high performace mode
    WriteRegister(LIS3MDL_CTRL_REG5, 0b01000000); //set BDU to 1
    
}
void magnetActive(void){ 
    // Sometimes the PSoC does not recognize the sensor for the current slave address.
    // The if statement will switch the address every time the sensor is not recognized.
    if(!CheckSensorIdentity(WHO_AM_I_REG_ADDR_LIS3MDL,LIS3MDL_DEVICE_ID)){
        count++;
        if(count % 2 == 1){
            register_setting.slaveAddress = LIS3MDL_ADDR2;
        }
        else{
            register_setting.slaveAddress = LIS3MDL_ADDR1;
        }
    }
        
    XL = ReadRegister(OUT_X_L);
    XH = ReadRegister(OUT_X_H);
    YL = ReadRegister(OUT_Y_L);
    YH = ReadRegister(OUT_Y_H);
    ZL = ReadRegister(OUT_Z_L);
    ZH = ReadRegister(OUT_Z_H);
    
    // shift the high data to left and or it to combine low and high data
    xMagnetometerData = ((XH << 8) | XL);  
    yMagnetometerData = ((YH << 8) | YL);
    zMagnetometerData = ((ZH << 8) | ZL);
    
    printf("MagData X: %" PRId16 "r\r\n", xMagnetometerData);
    printf("MagData Y: %" PRId16 "r\r\n", yMagnetometerData);
    //printf("MagData Z: %" PRId16 "\r\n", zMagnetometerData); // Unused 
    
    heading = CalculateHeading(xMagnetometerData, yMagnetometerData);
    printf("Heading: %.2f degrees\r\n", heading);
    CLCD_Clear();
    write_sensor((float)heading);
    do_pos(2,1);
    
    // Determine Direction
    if((heading >= 359 && heading <= 360) || (heading >= 0 && heading <= 1)){
        CLCD_PutString("North");
    }
    else if(heading > 1 && heading <= 88){
        CLCD_PutString("North East\n");
    }
    else if(heading >= 89 && heading <= 91){
        CLCD_PutString("East");
    }
    else if(heading > 91 && heading < 179){
        CLCD_PutString("South East");
    }
    else if(heading >= 179 && heading <= 181){
        CLCD_PutString("South");
    }
    else if(heading > 181 && heading < 269){
        CLCD_PutString("South West");
    }
    else if(heading >= 269 && heading <= 271){
        CLCD_PutString("West");
    }
    else if(heading > 271 && heading < 359){
        CLCD_PutString("North West");
    }
}

/*******************************************************************************
* Humidity Initialize and Active Functions
*******************************************************************************/
void humidityInitialize(void){
    sensorOFF(ADT7410_ADDR);
    //sensorOFF(LIS3MDL_ADDR1);
    sensorON(HTU31D_ADDR);
    sensorOFF(ALTIMETER_ADDR);
    
    changeButtonStatus("Humidity");
    register_setting.slaveAddress = HTU31D_ADDR;
}
void humidityActive(void){
    // Execute “Conversion” command with the desired resolution to perform measurement and load it in sensor memory
    WriteRegister(HTU31D_CONVERSION,0x01);
    
    // Wait for the conversion time (5ms)
    CyDelay(5);
    
    humidityRaw = (ReadRegister(HTU31D_READHUM) << 8);
    printf("Raw Humidity: %d\r\n",humidityRaw);
    
    // Convert to actual humidity value (check the datasheet for details)
    float actualHumidity = 100.0 * (humidityRaw/(pow(2,16)-1));
    
    // Print the humidity
    printf("Actual Humidity: %.2f\r\n\n", actualHumidity);    
    CLCD_Clear();
    write_sensor((float)actualHumidity); 
}

/*******************************************************************************
* LIS3DH Initialize and Active Functions
*******************************************************************************/
void LIS3DHInitialize(void){
    sensorOFF(ADT7410_ADDR);
    //sensorOFF(LIS3MDL_ADDR1);
    sensorOFF(HTU31D_ADDR);
    sensorOFF(ALTIMETER_ADDR);
    register_setting.slaveAddress = LIS3DH_ADDRESS;
    setvbuf(stdin, NULL, _IONBF,0);
    WriteRegister(0x20,0x57); //going up to 100 Hz
    WriteRegister(0x22,0xC0); //Sets up interupt tried(C0) no change
    WriteRegister(0x25,0b01101000);//Enable CTRL_REG6 I2-IA2 INT2
    WriteRegister(0x30,0xE0); // //NO CHANGE
    WriteRegister(0x38,0x2A); //Sets up config <-sets up double tap //NO CHANGE (TRY 2A)

    
    WriteRegister(0x34,0b00100000);//INT2_CFG Enable 6 Direction dection function
    WriteRegister(0x36,0b00111110);// INT_THS
    WriteRegister(0x37,0b00001111);//INT-DURAtion 0x36
    WriteRegister(0x3E,0b00010111);//ACT_THS testiing
    WriteRegister(0x3F,0b00000000);//ACT_DUR Testing
       
    WriteRegister(0x33,0x40); //Duration //hint(40)
    WriteRegister(0x3A,0x7); // tried 8,threshold //hint  0000011, High number, Highsensitivity
    WriteRegister(0x3B,0x15); // tried 8,Limit
    WriteRegister(0x3C,0xA); // latency  (changes width in time of output interupt)
    WriteRegister(0x3D,0xF); // tried 8,window
}

//int x_arr[30];
//int x_ind = 0;
int y_ind = 0;
int y_avg = 0;
int active = 0;
int rest = 0;


void LIS3DHActive(void){
            xa = (ReadRegister(0x28) | (ReadRegister(0x29)<<8)); // Combining Higer 8bit and lower 8 bit
            ya = (ReadRegister(0x2A) | (ReadRegister(0x2B)<<8)); 
            za = (ReadRegister(0x2C) | (ReadRegister(0x2D)<<8)); 

            x_accl = ((2.0 * xa) / 32767) * 9.8;
            y_accl = ((2.0 * ya) / 32767) * 9.8;
            z_accl = ((2.0 * za) / 32767) * 9.8;
            
            if(y_accl > 4){
                runtime = runtime + 1;
                active = 1;
                rest = 0;
                if (y_ind == 3) {
                    y_avg = y_avg / 3;
                    y_ind = 0;
                } else {
                    y_avg += y_accl;
                    y_ind++;
                }
            }
            else {
                active = 0;
                rest = 1;
                resttime = resttime + 1;
            }
            
            printf("x: %.2f, y: %.2f, z: %.2f  \r\n", x_accl,y_accl, z_accl);
            printf("Run time: %.2f\r\n", runtime);
            printf("Rest time: %.2f\r\n", resttime);
            totalTime = totalTime + 1;
            printf("Total time: %d\r\n", totalTime);
            CLCD_Clear();
            
            
            int selector = 0; 
            if(joe == 0) { 
                write_sensor((float)y_accl);
                joe +=1;
            }
            else if(joe == 1){
                write_sensor((float)y_avg);
                joe +=1;
            }
            else if(joe == 2){
                write_sensor((float)active);
                joe = 0;
            }
                
            else
            rest = 0;
            active = 0;
}

/*******************************************************************************
* Altimeter Initialize and Active Functions
*******************************************************************************/
void altimeterInitialize(void){
    //sensorOFF(ADT7410_ADDR);
    sensorOFF(LIS3MDL_ADDR1);
    sensorOFF(HTU31D_ADDR);
    sensorON(ALTIMETER_ADDR);
    
    changeButtonStatus("Altitude");
    register_setting.slaveAddress = ALTIMETER_ADDR;
    WriteRegister(MPL3115A2_CTRL_REG1, 0xB8);   // Standby Mode
    WriteRegister(MPL3115A2_PT_DATA_CFG, 0x07); // Sensor Data Reg; allows new data to overwrite old data
    WriteRegister(MPL3115A2_CTRL_REG1, 0xB9);   // Active Mode & Altimeter Mode
    WriteRegister(MPL3115A2_OFF_H, 0b01111111); // Offset -10
    WriteRegister(0x0F, 00);
}
void altimeterActive(void){
    CheckSensorIdentity(WHO_AM_I_REG_ADDR_ALTIMETER,ALTIMETER_DEVICE_ID);

    xm = ReadRegister(MPL3115A2_OUT_P_MSB);
    xc = ReadRegister(MPL3115A2_OUT_P_CSB);
    xl = ReadRegister(MPL3115A2_OUT_P_LSB);
    off = ReadRegister(MPL3115A2_OFF_H);
    tm = ReadRegister(MPL3115A2_OUT_T_MSB);
    tl = ReadRegister(MPL3115A2_OUT_T_LSB);

    // Calculating Temp
    printf("\nJoe:%d Mungus:%d\r\n",tm,tl); 
    temp = ((tm << 8) | tl) / 256.0;    // Temp in Celcius
    printf("\nTemperature: %.2f degrees Celsius\r\n", temp);
    temperatureAlt = (temp * 9.0 / 5.0) + 32.0; // Temp in Fahranheit
    printf("\nTemperature: %.2f degrees Fahrenheit\r\n", temperatureAlt);
            
    // Convert raw data to altitude (check MPL3115A2 datasheet for details)
    altitude = (float)((xc << 8) | xm) / 1600.0;
    printf("Altitude: %.2f meters\r\n", altitude);
    CLCD_Clear();
    write_sensor((float)altitude);
}

int main(void){
    __enable_irq(); /* Enable global interrupts. */
    
    I2C_Start();
    UART_Start();
    ADC_Start();
    
    setvbuf(stdin, NULL, _IONBF, 0);
    
    // Button Setups
    Cy_SysInt_Init(&Button1_Int_cfg, button1Handler);
    NVIC_EnableIRQ(Button1_Int_cfg.intrSrc);
    
    Cy_SysInt_Init(&Button2_Int_cfg, button2Handler);
    NVIC_EnableIRQ(Button2_Int_cfg.intrSrc);
    
    Cy_SysInt_Init(&Button3_Int_cfg, button3Handler);
    NVIC_EnableIRQ(Button3_Int_cfg.intrSrc);
    
    Cy_SysInt_Init(&Button4_Int_cfg, button4Handler);
    NVIC_EnableIRQ(Button4_Int_cfg.intrSrc);
    
    Cy_SysInt_Init(&Button5_Int_cfg, button5Handler);
    NVIC_EnableIRQ(Button5_Int_cfg.intrSrc);
    
    double value;
    double volts;
    
    tty_init(USE_CM4);
    CLCD_Init();
    
    do_pos(1,3);
    CLCD_PutString("WagWatch!");
    CyDelay(1500);
    CLCD_Clear();
    do_pos(1,3);
    CLCD_PutString("Presented by");
    do_pos(2,3);
    CLCD_PutString("Ender!");
    CyDelay(1500);
    do_pos(2,3);
    CLCD_PutString("Thomas!");
    CyDelay(1500);
    do_pos(2,3);
    CLCD_PutString("Nathan!");
    CyDelay(1500);
    CLCD_Clear();
    CLCD_PutString("Awaiting Action");
    CyDelay(500);
    
    while(1){

       switch (flag){
            case 1:
                if(!buttonStatus[0]){
                    tempInitialize();
                }
                tempActive();
                break;
            case 2:
                if(!buttonStatus[1]){
                    LIS3DHInitialize();
                }
                LIS3DHActive();
                break;

            case 3:
                if(!buttonStatus[2]){
                    magnetInitialize();
                }
                magnetActive();
                break;
                
            case 4:
               if(!buttonStatus[3]){
                    humidityInitialize();
                }
                humidityActive();
                break;
                
                
               
                
            case 5:
                if(!buttonStatus[4]){
                    altimeterInitialize();
                }
                altimeterActive();
                break;
        }
        
        // Measure the battery percentage
        Cy_SAR_StartConvert(SAR, CY_SAR_START_CONVERT_SINGLE_SHOT);
        value = Cy_SAR_GetResult16(SAR, 0);
        volts = Cy_SAR_CountsTo_mVolts(SAR,0,value) * 2.0;
        
        /*
        // Print Battery 
        Percentage to LCD (UNUSED)
        CLCD_Clear();
        write_sensor((float)volts);
        CLCD_PutString("mV");
        do_pos(2,1);
        write_sensor((float)(volts/4000)*100);
        CLCD_PutString("%");
        */
        
        /*
        // Display  (UNUSED)
        if (volts > 4000 ) {
            Cy_GPIO_Write(RedLED_PORT, RedLED_NUM, 0);
        }
        else if ((volts <= 4000) && (volts > 3850)) {
            Cy_GPIO_Inv(RGB_Red1_PORT, RedLED_NUM);
        }
        else if (volts <= 500) {
            Cy_GPIO_Inv(RedLED_PORT, RedLED_NUM);
        }
        */
        //printf("Battery Voltage = %.2lf mV\r\n", volts);
        
        
        for(int i = 0; i < 500; i++)
        {
            Cy_GPIO_Write(Buzzer_PORT, Buzzer_NUM, 1);
            CyDelay(1);
            Cy_GPIO_Write(Buzzer_PORT, Buzzer_NUM, 0);
            CyDelay(1);
        }
        
        //CyDelay(1000);
    }
}
