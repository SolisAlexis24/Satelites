#include "LSM9DS1.h"

LSM9DS1::LSM9DS1(i2c_inst_t* i2c_port, mutex_t* i2c_mutex) 
    : i2c_port_(i2c_port),
     i2c_mutex_(i2c_mutex) {
}


bool LSM9DS1::init_accel(GYRO_SCALES gyro_scale, ACCEL_SCALES accel_scale, AG_ODR gyro_odr, AG_ODR accel_odr) {

    uint8_t whoami_ag = read_register(AG_ADDR, WHO_AM_I_AG);

    if (VAL_WHO_AM_I_AG != whoami_ag) {
        printf("LSM9SD1: Error, Sensor LSM9DS1 (AG) no detectado!\n");
        return false;
    }

    // Configurar giroscopio - CTRL_REG1_G (0x10)
    // [ODR_G2][ODR_G1][ODR_G0][FS1][FS0][0][BW1][BW0]
    uint8_t ctrl_reg1_g = (gyro_odr << 5) | (gyro_scale << 3);
    write_register(AG_ADDR, CTRL_REG1_G, ctrl_reg1_g);
    
    // Configurar acelerómetro - CTRL_REG6_XL (0x20)
    // [ODR_XL2][ODR_XL1][ODR_XL0][FS1_XL][FS0_XL][BW_SCAL_ODR][BW_XL1][BW_XL0]
    uint8_t ctrl_reg6_xl = (accel_odr << 5) | (accel_scale << 3);
    write_register(AG_ADDR, CTRL_REG6_XL, ctrl_reg6_xl);

    // Activar salidas, no interrupciones y no 4D
    write_register(AG_ADDR, CTRL_REG4, 0x38);
    // Sin decimacion y activar salidas
    write_register(AG_ADDR, CTRL_REG5_XL, 0x38);
    // Valores por defecto
    write_register(AG_ADDR, CTRL_REG7_XL, 0x00);
    // Valores por defecto
    write_register(AG_ADDR, CTRL_REG8, 0x04);
    // Activar FIFO
    write_register(AG_ADDR, CTRL_REG9, 0x02);

    // Configurar FIFO (modo continuo)
    write_register(AG_ADDR, FIFO_CTRL_REG, 0x00); // Reset FIFO
    write_register(AG_ADDR, FIFO_CTRL_REG, 0xC0); // Modo continuo
    
    calculate_accel_scale_factors(gyro_scale, accel_scale);

    printf("LSM9DS1: Sensor AG nicializado correctamente\n");
    return true;
}


bool LSM9DS1::init_magnetometer(MAG_SCALES scale, MAG_ODR sample_rate) {
    // Verificar conexión leyendo el registro WHO_AM_I
    uint8_t whoami_m = read_register(M_ADDR, WHO_AM_I_M);

    if (VAL_WHO_AM_I_M != whoami_m) {
        printf("LSM9DS1: Error, Magnetómetro LSM9DS1 no detectado!\n");
        return false;
    }

    // Configurar CTRL_REG1_M (0x20)
    // [TEMP_COMP][OM1][OM0][DO2][DO1][DO0][FAST_ODR][ST]
    uint8_t ctrl_reg1 = 0x40 | (sample_rate << 2);  // High performance mode + ODR
    write_register(M_ADDR, CTRL_REG1_M, ctrl_reg1);
    
    // Configurar CTRL_REG2_M (0x21)
    // [0][FS1][FS0][0][REBOOT][SOFT_RST][0][0]
    uint8_t ctrl_reg2 = scale << 5;  // Configurar escala
    write_register(M_ADDR, CTRL_REG2_M, ctrl_reg2);
    
    // Configurar CTRL_REG3_M (0x22) - Modo de conversión continua
    write_register(M_ADDR, CTRL_REG3_M, 0x00);
    
    // Configurar CTRL_REG4_M (0x23) - Alto rendimiento en eje Z
    write_register(M_ADDR, CTRL_REG4_M, 0x08);
    
    // Configurar CTRL_REG5_M (0x24) - Configuración adicional
    write_register(M_ADDR, CTRL_REG5_M, 0x00);
    
    calculate_mag_scale_factor(scale);

    printf("LSM9DS1: Sensor M nicializado correctamente\n");

    return true;
}


void LSM9DS1::set_offsets_mag(float offset_x, float offset_y, float offset_z){
    int16_t x = offset_x/mag_div_scale_factor_;
    int16_t y = offset_y/mag_div_scale_factor_;
    int16_t z = offset_z/mag_div_scale_factor_; // Se mapean los valores
    uint8_t buf[6] = {
        static_cast<uint8_t>(x & 0xFF),   // LSB X
        static_cast<uint8_t>(x >> 8),     // MSB X
        static_cast<uint8_t>(y & 0xFF),   // LSB Y
        static_cast<uint8_t>(y >> 8),     // MSB Y
        static_cast<uint8_t>(z & 0xFF),   // LSB Z
        static_cast<uint8_t>(z >> 8)      // MSB Z
    };
    write_register(M_ADDR, OFFSET_X_REG_L_M ,buf[0]);
    write_register(M_ADDR, OFFSET_X_REG_H_M ,buf[1]);
    write_register(M_ADDR, OFFSET_Y_REG_L_M ,buf[2]);
    write_register(M_ADDR, OFFSET_Y_REG_H_M ,buf[3]);
    write_register(M_ADDR, OFFSET_Z_REG_L_M ,buf[4]);
    write_register(M_ADDR, OFFSET_Z_REG_H_M ,buf[5]);
}

void LSM9DS1::set_offsets_gyro(float offset_x, float offset_y, float offset_z){
    gyro_offset_x_ = offset_x;
    gyro_offset_y_ = offset_y;
    gyro_offset_z_ = offset_z;
}

void LSM9DS1::set_offsets_accel(float offset_x, float offset_y, float offset_z){
    accel_offset_x_ = offset_x;
    accel_offset_y_ = offset_y;
    accel_offset_z_ = offset_z;
}

void LSM9DS1::read_sensor(uint64_t time_ms){
    read_accelerometer(this->last_measurement.accel);
    read_gyroscope(this->last_measurement.gyro);
    read_magnetometer(this->last_measurement.mag);
    this->last_measurement.time_ms = time_ms;
}

uint8_t LSM9DS1::read_register(uint8_t addr, uint8_t reg) {
    uint8_t val;
    if(i2c_write_blocking(i2c_port_, addr, &reg, 1, true) != 1){
        printf("LSM9DS1: Fallo al intentar leer el registro %02x\n", reg);
        return 0;
    }
    if(i2c_read_blocking(i2c_port_, addr, &val, 1, false) != 1){
        printf("LSM9DS1: Fallo al intentar leer el registro %02x\n", reg);
        return 0;        
    }
    return val;
}

void LSM9DS1::write_register(uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    if(i2c_write_blocking(i2c_port_, addr, buf, 2, false) != 2){
        printf("LSM9DS1: Fallo al intentar escribir el registro %02x\n", reg);
    }
}


void LSM9DS1::read_bytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
        mutex_enter_blocking(i2c_mutex_);
        if(i2c_write_blocking(i2c_port_, addr, &reg, 1, true) != 1){
            printf("LSM9DS1: Fallo al intentar leer el registro %02x\n", reg);
            mutex_exit(i2c_mutex_);
            return;   
        }
        mutex_exit(i2c_mutex_);
        mutex_enter_blocking(i2c_mutex_);
        if(i2c_read_blocking(i2c_port_, addr, buf, len, false) != len){
            printf("LSM9DS1: Fallo al intentar leer el registro %02x\n", reg);      
        }
        mutex_exit(i2c_mutex_);
        return;
}


void LSM9DS1::calculate_accel_scale_factors(GYRO_SCALES gyro_scale, ACCEL_SCALES accel_scale) {
    // Factores de conversión para el giroscopio (dps/LSB)
    static const float gyro_scales[] = {
        0.00747f,    // ±245 dps 
        0.01525f,    // ±500 dps
        0.00747f,    // Valor repetido pq 2000dps es 3
        0.061f       // ±2000 dps
    };
    
    // Factores de conversión para el acelerómetro (g/LSB)
    static const float accel_scales[] = {
        0.000061f,  // ±2g  (0.000061 = 2/32768)
        0.000488f,   // ±16g
        0.000122f,  // ±4g
        0.000244f  // ±8g
    };
    
    gyro_div_scale_factor_ = gyro_scales[gyro_scale];
    accel_div_scale_factor_ = accel_scales[accel_scale];
}


void LSM9DS1::calculate_mag_scale_factor(MAG_SCALES scale) {
    // Factores de conversión para el magnetómetro (Gauss/LSB)
    static const float mag_scales[] = {
        0.00012f,   // ±4G  
        0.00024f,   // ±8G
        0.00036f,   // ±12G
        0.00048f    // ±16G
    };
    
    mag_div_scale_factor_ = mag_scales[scale];
}

void LSM9DS1::read_accelerometer(float (&accel)[3]) {
    uint8_t buf[6];
    read_bytes(AG_ADDR, OUT_X_L_XL | LECTURA_CONSECUTIVA, buf, 6);
    
    int16_t x = (buf[1] << 8) | buf[0];
    int16_t y = (buf[3] << 8) | buf[2];
    int16_t z = (buf[5] << 8) | buf[4];
    
    accel[0] = (x * accel_div_scale_factor_) - accel_offset_x_;
    accel[1] = (y * accel_div_scale_factor_) - accel_offset_y_;
    accel[2] = (z * accel_div_scale_factor_) - accel_offset_z_;
}


void LSM9DS1::read_gyroscope(float (&gyro)[3]) {
    uint8_t buf[6];
    read_bytes(AG_ADDR, OUT_X_L_G | LECTURA_CONSECUTIVA, buf, 6);
    
    int16_t x = (buf[1] << 8) | buf[0];
    int16_t y = (buf[3] << 8) | buf[2];
    int16_t z = (buf[5] << 8) | buf[4];

    gyro[0] = (x * gyro_div_scale_factor_) - gyro_offset_x_;
    gyro[1] = (y * gyro_div_scale_factor_) - gyro_offset_y_;
    gyro[2] = (z * gyro_div_scale_factor_) - gyro_offset_z_;
}


void LSM9DS1::read_magnetometer(float (&mag)[3]) {
    uint8_t buf[6];
    read_bytes(M_ADDR, OUT_X_L_M | LECTURA_CONSECUTIVA, buf, 6);
    
    int16_t x = (buf[1] << 8) | buf[0];
    int16_t y = (buf[3] << 8) | buf[2];
    int16_t z = (buf[5] << 8) | buf[4];
    
    mag[0] = x * mag_div_scale_factor_;
    mag[1] = y * mag_div_scale_factor_;
    mag[2] = z * mag_div_scale_factor_;
}