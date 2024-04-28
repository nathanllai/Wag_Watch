/*******************************************************************************
* TEMPERATURE
*******************************************************************************/
#define ADT7410_ADDR 0x48

#define ADT7410_TEMPMSB 0x00	// Temp. value MSB
#define ADT7410_TEMPLSB 0x01	// Temp. value LSB
#define ADT7410_STATUS 0x2		// Status register
#define ADT7410_CONFIG 0x3		// Configuration register
#define ADT7410_ID 0xB			// Manufacturer identification
#define ADT7410_SWRST 0x2F		// Temperature hysteresis
/*******************************************************************************
* LIS3DH
*******************************************************************************/
#define LIS3DH_ADDRESS                0x19
enum
{                                               // BINARY   DEFAULT   TYPE
  LIS3DH_REGISTER_STATUS_REG_AUX      = 0x07,   // 00000111           r
  LIS3DH_REGISTER_OUT_ADC1_L          = 0x08,   // 00001000 output    r
  LIS3DH_REGISTER_OUT_ADC1_H          = 0x09,   // 00001001 output    r
  LIS3DH_REGISTER_OUT_ADC2_L          = 0x0A,   // 00001010 output    r
  LIS3DH_REGISTER_OUT_ADC2_H          = 0x0B,   // 00001011 output    r
  LIS3DH_REGISTER_OUT_ADC3_L          = 0x0C,   // 00001100 output    r
  LIS3DH_REGISTER_OUT_ADC3_H          = 0x0D,   // 00001101 output    r
  LIS3DH_REGISTER_INT_COUNTER_REG     = 0x0E,   // 00001110           r
  LIS3DH_REGISTER_WHO_AM_I            = 0x0F,   // 00001111 00110011  r
  LIS3DH_REGISTER_TEMP_CFG_REG        = 0x1F,   // 00011111           rw
  LIS3DH_REGISTER_CTRL_REG1           = 0x20,   // 00100000 00000111  rw
  LIS3DH_REGISTER_CTRL_REG2           = 0x21,   // 00100001 00000000  rw
  LIS3DH_REGISTER_CTRL_REG3           = 0x22,   // 00100010 00000000  rw
  LIS3DH_REGISTER_CTRL_REG4           = 0x23,   // 00100011 00000000  rw
  LIS3DH_REGISTER_CTRL_REG5           = 0x24,   // 00100100 00000000  rw
  LIS3DH_REGISTER_CTRL_REG6           = 0x25,   // 00100101 00000000  rw
  LIS3DH_REGISTER_REFERENCE           = 0x26,   // 00100110 00000000  rw
  LIS3DH_REGISTER_STATUS_REG2         = 0x27,   // 00100111 00000000  r
  LIS3DH_REGISTER_OUT_X_L             = 0x28,   // 00101000 output    r
  LIS3DH_REGISTER_OUT_X_H             = 0x29,   // 00101001 output    r
  LIS3DH_REGISTER_OUT_Y_L             = 0x2A,   // 00101010 output    r
  LIS3DH_REGISTER_OUT_Y_H             = 0x2B,   // 00101011 output    r
  LIS3DH_REGISTER_OUT_Z_L             = 0x2C,   // 00101100 output    r
  LIS3DH_REGISTER_OUT_Z_H             = 0x2D,   // 00101101 output    r
  LIS3DH_REGISTER_FIFO_CTRL_REG       = 0x2E,   // 00101110 00000000  rw
  LIS3DH_REGISTER_FIFO_SRC_REG        = 0x2F,   // 00101111           r
  LIS3DH_REGISTER_INT1_CFG            = 0x30,   // 00110000 00000000  rw
  LIS3DH_REGISTER_INT1_SOURCE         = 0x31,   // 00110001 00000000  r
  LIS3DH_REGISTER_INT1_THS            = 0x32,   // 00110010 00000000  rw
  LIS3DH_REGISTER_INT1_DURATION       = 0x33,   // 00110011 00000000  rw
  LIS3DH_REGISTER_CLICK_CFG           = 0x38,   // 00111000 00000000  rw
  LIS3DH_REGISTER_CLICK_SRC           = 0x39,   // 00111001 00000000  r
  LIS3DH_REGISTER_CLICK_THS           = 0x3A,   // 00111010 00000000  rw
  LIS3DH_REGISTER_TIME_LIMIT          = 0x3B,   // 00111011 00000000  rw
  LIS3DH_REGISTER_TIME_LATENCY        = 0x3C,   // 00111100 00000000  rw
  LIS3DH_REGISTER_TIME_WINDOW         = 0x3D    // 00111101 00000000  rw
};

enum
{
  LIS3DH_STATUS_REG_ZYXDA             = 0x08,   // STATUS_REG: XYZ - sample ready
  LIS3DH_STATUS_REG_ZYXOR             = 0x80,   // STATUS_REG: XYZ - new set of data has overwritten the previous ones
  LIS3DH_CTRL_REG1_XEN                = 0x01,   // CTRL_REG1: X enable
  LIS3DH_CTRL_REG1_YEN                = 0x02,   // CTRL_REG1: Y enable
  LIS3DH_CTRL_REG1_ZEN                = 0x04,   // CTRL_REG1: Z enable
  LIS3DH_CTRL_REG1_XYZEN              = 0x07,   // CTRL_REG1: X+Y+Z enable
  LIS3DH_CTRL_REG1_LPEN               = 0x08,   // CTRL_REG1: Low power enable
  LIS3DH_CTRL_REG1_DATARATE_POWERDOWN = 0x00,   // CTRL_REG1: 0000 xxxx
  LIS3DH_CTRL_REG1_DATARATE_1HZ       = 0x10,   // CTRL_REG1: 0001 xxxx
  LIS3DH_CTRL_REG1_DATARATE_10HZ      = 0x20,   // CTRL_REG1: 0010 xxxx
  LIS3DH_CTRL_REG1_DATARATE_25HZ      = 0x30,   // CTRL_REG1: 0011 xxxx
  LIS3DH_CTRL_REG1_DATARATE_50HZ      = 0x40,   // CTRL_REG1: 0100 xxxx
  LIS3DH_CTRL_REG1_DATARATE_100HZ     = 0x50,   // CTRL_REG1: 0101 xxxx
  LIS3DH_CTRL_REG1_DATARATE_200HZ     = 0x60,   // CTRL_REG1: 0110 xxxx
  LIS3DH_CTRL_REG1_DATARATE_400HZ     = 0x70,   // CTRL_REG1: 0111 xxxx
  LIS3DH_CTRL_REG1_DATARATE_1500HZ    = 0x80,   // CTRL_REG1: 1000 xxxx
  LIS3DH_CTRL_REG1_DATARATE_5000HZ    = 0x90,   // CTRL_REG1: 1001 xxxx
  LIS3DH_CTRL_REG4_BLOCKDATAUPDATE    = 0x80,   // CTRL_REG4: 1xxx xxxx
  LIS3DH_CTRL_REG4_SCALE_2G           = 0x00,   // CTRL_REG4: xx00 xxxx
  LIS3DH_CTRL_REG4_SCALE_4G           = 0x10,   // CTRL_REG4: xx01 xxxx
  LIS3DH_CTRL_REG4_SCALE_8G           = 0x20,   // CTRL_REG4: xx10 xxxx
  LIS3DH_CTRL_REG4_SCALE_16G          = 0x30,   // CTRL_REG4: xx11 xxxx
};
/*******************************************************************************
* IR TEMPERATURE
*******************************************************************************/
#define MLX90614_DEFAULT_ADDRESS 0x5A

// EEPROM
#define TO_MAX              0x00
#define TO_MIN              0x01
#define PWMCTRL             0x02
#define TA_RANGE            0x03
#define EMISSIVITY          0X04
#define CONFIG_REGISTER1    0x05

// RAM Module
#define MLX90614_REGISTER_TA      0x06 // Ambient Temperature
#define MLX90614_REGISTER_TOBJ1	  0x07 // Object Temperature
#define MLX90614_REGISTER_TOBJ2	  0x08
#define MLX90614_REGISTER_TOMAX   0x20
#define MLX90614_REGISTER_TOMIN   0x21
#define MLX90614_REGISTER_PWMCTRL 0x22
#define MLX90614_REGISTER_TARANGE 0x23
#define MLX90614_REGISTER_KE      0x24
#define MLX90614_REGISTER_CONFIG  0x25
#define MLX90614_REGISTER_ADDRESS 0x2E
#define MLX90614_REGISTER_ID0     0x3C
#define MLX90614_REGISTER_ID1     0x3D
#define MLX90614_REGISTER_ID2     0x3E
#define MLX90614_REGISTER_ID3     0x3F
#define MLX90614_REGISTER_SLEEP   0xFF // Not really a register, but close enough

/*******************************************************************************
* MAGNETOMETER
*******************************************************************************/
#define LIS3MDL_ADDR1 0x1E
#define LIS3MDL_ADDR2 0x1C

#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D
#define LIS3MDL_CTRL_REG2 0x21
#define LIS3MDL_CTRL_REG3 0x22
#define LIS3MDL_CTRL_REG1 0x20
#define LIS3MDL_CTRL_REG4 0x23
#define LIS3MDL_CTRL_REG5 0x24

#define WHO_AM_I_REG_ADDR_LIS3MDL 0x0F
#define LIS3MDL_DEVICE_ID 0x3D

/*******************************************************************************
* HUMIDITY
*******************************************************************************/
#define HTU31D_ADDR 0x40		// 0x40 connected to GND, 0x41 connected to VDD

#define HTU31D_READTEMPHUM 0x00
#define HTU31D_READHUM 0x10		// 0b00010000
#define HTU31D_CONVERSION 0x40
#define HTU31D_READSERIAL 0x0A
#define HTU31D_HEATERON 0x04
#define HTU31D_HEATEROFF 0x02
#define HTU31D_RESET 0x1E

/*******************************************************************************
* ALTIMETER
*******************************************************************************/
#define WHO_AM_I_REG_ADDR_ALTIMETER 0x0C
#define ALTIMETER_DEVICE_ID 0xC4
#define ALTIMETER_ADDR 0x60

#define MPL3115A2_ADDR 0x60
#define MPL3115A2_OUT_P_MSB 0X01
#define MPL3115A2_OUT_P_CSB 0x02
#define MPL3115A2_OUT_P_LSB 0x03
#define MPL3115A2_OUT_T_MSB 0x04
#define MPL3115A2_OUT_T_LSB 0x05
#define MPL3115A2_PT_DATA_CFG 0x13
#define MPL3115A2_CTRL_REG1 0x26
#define MPL3115A2_OFF_H 0x2D

/*******************************************************************************
* LCD
*******************************************************************************/
#define LCD_ROW_COUNT 2
#define LCD_COL_COUNT 16