/**
* Calliope mini Extension for INA219 Current Power supply sensor module
*
* Thanks to Wolfgang Ewald https://wolles-elektronikkiste.de/ina219
*
* @author Raik Andritschke
*/

// INA219 I2C Adresse
enum INA219ADDR {
    //% block="0x40"
    X40 = 0x40,
    //% block="0x41"
    X41 = 0x41,
    //% block="0x44"
    X44 = 0x44,
    //% block="0x45"
    X45 = 0x45
};

// Config Register (R/W)
const REG_CONFIG = 0x00;

// SHUNT VOLTAGE REGISTER (R)
const REG_SHUNTVOLTAGE = 0x01;

// BUS VOLTAGE REGISTER (R)
const REG_BUSVOLTAGE = 0x02;

// POWER REGISTER (R)
const REG_POWER = 0x03;

// CURRENT REGISTER (R)
const REG_CURRENT = 0x04;

// CALIBRATION REGISTER (R/W)
const REG_CALIBRATION = 0x05;

enum BusVoltageRange {
    RANGE_16V = 0x00, // set bus voltage range to 16V
    RANGE_32V = 0x01  // set bus voltage range to 32V (default)
}

enum Gain {
    DIV_1_40MV = 0x00,  // shunt prog. gain set to  1, 40 mV range
    DIV_2_80MV = 0x01,  // shunt prog. gain set to /2, 80 mV range
    DIV_4_160MV = 0x02, // shunt prog. gain set to /4, 160 mV range
    DIV_8_320MV = 0x03  // shunt prog. gain set to /8, 320 mV range
}

enum ADCResolution {
    /**
     * Constants for bus_adc_resolution or shunt_adc_resolution
     */
    ADCRES_9BIT_1S = 0x00,   //  9 bit,  1 sample,     84us
    ADCRES_10BIT_1S = 0x01,  // 10 bit,  1 sample,    148us
    ADCRES_11BIT_1S = 0x02,  // 11 bit,  1 sample,    276us
    ADCRES_12BIT_1S = 0x03,  // 12 bit,  1 sample,    532us
    ADCRES_12BIT_2S = 0x09,  // 12 bit,  2 samples,  1.06ms
    ADCRES_12BIT_4S = 0x0A,  // 12 bit,  4 samples,  2.13ms
    ADCRES_12BIT_8S = 0x0B,  // 12 bit,  8 samples,  4.26ms
    ADCRES_12BIT_16S = 0x0C, // 12 bit, 16 samples,  8.51ms
    ADCRES_12BIT_32S = 0x0D, // 12 bit, 32 samples, 17.02ms
    ADCRES_12BIT_64S = 0x0E, // 12 bit, 64 samples, 34.05ms
    ADCRES_12BIT_128S = 0x0F // 12 bit,128 samples, 68.10ms
}

enum Mode {
    /**
     * Constants for mode
     */
    POWERDOWN = 0x00,           // power down
    SVOLT_TRIGGERED = 0x01,     // shunt voltage triggered
    BVOLT_TRIGGERED = 0x02,     // bus voltage triggered
    SANDBVOLT_TRIGGERED = 0x03, // shunt and bus voltage triggered
    ADCOFF = 0x04,              // ADC off
    SVOLT_CONTINUOUS = 0x05,    // shunt voltage continuous
    BVOLT_CONTINUOUS = 0x06,    // bus voltage continuous
    SANDBVOLT_CONTINUOUS = 0x07 // shunt and bus voltage continuous
}

/**
 * Custom blocks
 */
//% weight=20 color=#0fbc11 icon=""
namespace ina219 {

    let I2CADDR = INA219ADDR.X40;

    let current_lsb = 10;
    let cal_value = 4096;
    let power_lsb = 2;

    const INA219_RST = 0x8000;

    //% blockId="init" block="Initialisiere den Sensor INA219 mit I2C Adresse %addr"
    export function init(addr: INA219ADDR): void {
        I2CADDR = addr;
    }

    function readRegister(register: number): number {
        pins.i2cWriteNumber(I2CADDR, register, NumberFormat.UInt8LE, false)
        let buffer = pins.i2cReadBuffer(I2CADDR, 2, false)
        return pins.i2cReadNumber(I2CADDR, NumberFormat.UInt16BE, false);
    }

    function writeRegister(register: number, value: number): void {
        let temp: Buffer = pins.createBuffer(3);
        temp[0] = register;
        temp[1] = value >> 8;
        temp[2] = value & 255;
        pins.i2cWriteBuffer(I2CADDR, temp, false);
    }

    //% blockId="reset" block="Setze den Sensor zurück"
    export function reset(): void {
        writeRegister(REG_CONFIG, INA219_RST);
    }

    //% blockId="setCalibration" block="Kalibriere den Sensor auf %gain"
    export function setCalibration(gain: Gain): void {
        /**
         * bus voltage range << 13
         * gain << 11
         * bus ADC resolution << 7
         * shunt ADC resolution << 3
         * mode
         */
        let config = BusVoltageRange.RANGE_32V << 13
            | gain << 11
            | ADCResolution.ADCRES_12BIT_8S << 7
            | ADCResolution.ADCRES_12BIT_8S << 3
            | Mode.SANDBVOLT_CONTINUOUS;
        writeRegister(REG_CONFIG, config);

        switch (gain) {
            case Gain.DIV_8_320MV: {
                current_lsb = 10;
                cal_value = 4096;
                power_lsb = 2;
                break;
            }
            case Gain.DIV_4_160MV: {
                current_lsb = 20;
                cal_value = 8192;
                power_lsb = 1;
                break;
            }
            case Gain.DIV_2_80MV: {
                current_lsb = 25;
                cal_value = 10240;
                power_lsb = .8;
                break;
            }
            case Gain.DIV_1_40MV: {
                current_lsb = 50;
                cal_value = 20480;
                power_lsb = .4;
                break;
            }
        }
        writeRegister(REG_CALIBRATION, cal_value);
    }

    //% blockId="getShuntVoltage" block="Ermittle Shunt Spannung in mV"
    export function getShuntVoltage(): number {
        let value = readRegister(REG_SHUNTVOLTAGE);
        if (value > 32767) {
            value -= 65535;
        }
        return value * 0.01;
    }

    //% blockId="getBusVoltage" block="Ermittle Bus Spannung in V"
    export function getBusVoltage(): number {
        let busVoltage = readRegister(REG_BUSVOLTAGE);
        return (busVoltage >> 3) * 0.004;
    }

    //% blockId="getCurrentmA" block="Ermittle Strom in mA"
    export function getCurrentmA(): number {
        let value = readRegister(REG_CURRENT);
        if (value > 32767) {
            value -= 65535;
        }
        return value / current_lsb;
    }

    //% blockId="getPowerW" block="Ermittle Leistung in mW"
    export function getPowerW(): number {
        let value = readRegister(REG_POWER);
        if (value > 32767) {
            value -= 65535;
        }
        return value * power_lsb;
    }

}

