#include "I2C_communication.h"

/* Software I2C on PA0/PA1. Software I2C is still open-drain and therefore
 * requires pull-up resistors on both SDA and SCL. */
#define SOFT_I2C_PORT         GPIOA
#define SOFT_I2C_SDA_PIN      DL_GPIO_PIN_0
#define SOFT_I2C_SCL_PIN      DL_GPIO_PIN_1
#define SOFT_I2C_SDA_IOMUX    IOMUX_PINCM1
#define SOFT_I2C_SCL_IOMUX    IOMUX_PINCM2
#define SOFT_I2C_TIMEOUT      (10000U)
#define SOFT_I2C_DELAY_CYCLES ((CPUCLK_FREQ / 200000U) + 1U)

I2cControllerStatus_t gI2cControllerStatus = I2C_STATUS_IDLE;
volatile uint8_t gI2cErrorCode = I2C_ERROR_NONE;
volatile uint32_t gI2cErrorStatus = 0U;

static bool gSoftI2cInitialized = false;

static void SoftI2C_Delay(void)
{
    delay_cycles(SOFT_I2C_DELAY_CYCLES);
}

static void SoftI2C_SdaLow(void)
{
    DL_GPIO_clearPins(SOFT_I2C_PORT, SOFT_I2C_SDA_PIN);
    DL_GPIO_enableOutput(SOFT_I2C_PORT, SOFT_I2C_SDA_PIN);
}

static void SoftI2C_SclLow(void)
{
    DL_GPIO_clearPins(SOFT_I2C_PORT, SOFT_I2C_SCL_PIN);
    DL_GPIO_enableOutput(SOFT_I2C_PORT, SOFT_I2C_SCL_PIN);
}

/* Releasing a line means input/high impedance; the pull-up creates HIGH. */
static void SoftI2C_SdaRelease(void)
{
    DL_GPIO_disableOutput(SOFT_I2C_PORT, SOFT_I2C_SDA_PIN);
}

static void SoftI2C_SclRelease(void)
{
    DL_GPIO_disableOutput(SOFT_I2C_PORT, SOFT_I2C_SCL_PIN);
}

static bool SoftI2C_ReadSda(void)
{
    return (DL_GPIO_readPins(SOFT_I2C_PORT, SOFT_I2C_SDA_PIN) != 0U);
}

static bool SoftI2C_ReadScl(void)
{
    return (DL_GPIO_readPins(SOFT_I2C_PORT, SOFT_I2C_SCL_PIN) != 0U);
}

/* In software mode, status bit 0 is SDA and bit 1 is SCL. */
static void SoftI2C_SaveLineState(void)
{
    gI2cErrorStatus = (SoftI2C_ReadSda() ? 1U : 0U) |
                      (SoftI2C_ReadScl() ? 2U : 0U);
}

static bool SoftI2C_WaitSclHigh(void)
{
    uint32_t timeout = SOFT_I2C_TIMEOUT;

    SoftI2C_SclRelease();
    while (!SoftI2C_ReadScl()) {
        if (--timeout == 0U) {
            gI2cErrorCode = I2C_ERROR_BUSY_TIMEOUT;
            SoftI2C_SaveLineState();
            return false;
        }
    }
    return true;
}

static void SoftI2C_Init(void)
{
    if (gSoftI2cInitialized) {
        return;
    }

    /* Override the hardware-I2C pin mux with GPIO and keep the input buffers
     * enabled. Software I2C must read SDA for ACK/data and read SCL while the
     * lines are released. DL_GPIO_initDigitalOutput() disables INENA, which
     * makes DL_GPIO_readPins() report LOW even when a meter shows 3.3 V. */
    DL_GPIO_initDigitalInput(SOFT_I2C_SDA_IOMUX);
    DL_GPIO_initDigitalInput(SOFT_I2C_SCL_IOMUX);
    DL_GPIO_clearPins(
        SOFT_I2C_PORT, SOFT_I2C_SDA_PIN | SOFT_I2C_SCL_PIN);
    SoftI2C_SdaRelease();
    SoftI2C_SclRelease();
    SoftI2C_Delay();
    gSoftI2cInitialized = true;
}

static bool SoftI2C_Start(void)
{
    SoftI2C_SdaRelease();
    if (!SoftI2C_WaitSclHigh()) {
        return false;
    }
    SoftI2C_Delay();

    if (!SoftI2C_ReadSda()) {
        gI2cErrorCode = I2C_ERROR_IDLE_TIMEOUT;
        SoftI2C_SaveLineState();
        return false;
    }

    SoftI2C_SdaLow();
    SoftI2C_Delay();
    SoftI2C_SclLow();
    SoftI2C_Delay();
    return true;
}

static bool SoftI2C_Stop(void)
{
    SoftI2C_SdaLow();
    SoftI2C_Delay();
    if (!SoftI2C_WaitSclHigh()) {
        return false;
    }
    SoftI2C_Delay();
    SoftI2C_SdaRelease();
    SoftI2C_Delay();
    return SoftI2C_ReadSda();
}

static bool SoftI2C_WriteByte(uint8_t data)
{
    uint8_t bit;
    bool acknowledged;

    for (bit = 0U; bit < 8U; bit++) {
        if ((data & 0x80U) != 0U) {
            SoftI2C_SdaRelease();
        } else {
            SoftI2C_SdaLow();
        }
        SoftI2C_Delay();
        if (!SoftI2C_WaitSclHigh()) {
            return false;
        }
        SoftI2C_Delay();
        SoftI2C_SclLow();
        SoftI2C_Delay();
        data <<= 1U;
    }

    SoftI2C_SdaRelease();
    SoftI2C_Delay();
    if (!SoftI2C_WaitSclHigh()) {
        return false;
    }
    SoftI2C_Delay();
    acknowledged = !SoftI2C_ReadSda();
    SoftI2C_SclLow();
    SoftI2C_Delay();

    if (!acknowledged) {
        gI2cErrorCode = I2C_ERROR_NACK;
        SoftI2C_SaveLineState();
    }
    return acknowledged;
}

static uint8_t SoftI2C_ReadByte(bool acknowledge)
{
    uint8_t bit;
    uint8_t data = 0U;

    SoftI2C_SdaRelease();
    for (bit = 0U; bit < 8U; bit++) {
        data <<= 1U;
        SoftI2C_Delay();
        if (!SoftI2C_WaitSclHigh()) {
            return 0U;
        }
        SoftI2C_Delay();
        if (SoftI2C_ReadSda()) {
            data |= 1U;
        }
        SoftI2C_SclLow();
        SoftI2C_Delay();
    }

    if (acknowledge) {
        SoftI2C_SdaLow();
    } else {
        SoftI2C_SdaRelease();
    }
    SoftI2C_Delay();
    if (!SoftI2C_WaitSclHigh()) {
        return data;
    }
    SoftI2C_Delay();
    SoftI2C_SclLow();
    SoftI2C_Delay();
    SoftI2C_SdaRelease();
    return data;
}

bool I2C_WriteReg(uint8_t addr, uint8_t reg_addr,
                  const uint8_t *reg_data, uint8_t count)
{
    uint8_t i;
    bool ok = false;

    SoftI2C_Init();
    gI2cErrorCode = I2C_ERROR_NONE;
    gI2cErrorStatus = 0U;

    if ((reg_data == NULL) || (count == 0U) || !SoftI2C_Start()) {
        return false;
    }
    if (!SoftI2C_WriteByte((uint8_t)(addr << 1U)) ||
        !SoftI2C_WriteByte(reg_addr)) {
        goto stop;
    }
    for (i = 0U; i < count; i++) {
        if (!SoftI2C_WriteByte(reg_data[i])) {
            goto stop;
        }
    }
    ok = true;

stop:
    if (!SoftI2C_Stop()) {
        ok = false;
    }
    return ok;
}

bool I2C_ReadReg(uint8_t addr, uint8_t reg_addr,
                 uint8_t *reg_data, uint8_t count)
{
    uint8_t i;
    bool ok = false;

    SoftI2C_Init();
    gI2cErrorCode = I2C_ERROR_NONE;
    gI2cErrorStatus = 0U;

    if ((reg_data == NULL) || (count == 0U) || !SoftI2C_Start()) {
        return false;
    }
    if (!SoftI2C_WriteByte((uint8_t)(addr << 1U)) ||
        !SoftI2C_WriteByte(reg_addr)) {
        goto stop;
    }

    if (!SoftI2C_Start() ||
        !SoftI2C_WriteByte((uint8_t)((addr << 1U) | 1U))) {
        goto stop;
    }

    for (i = 0U; i < count; i++) {
        reg_data[i] = SoftI2C_ReadByte(i + 1U < count);
        if (gI2cErrorCode != I2C_ERROR_NONE) {
            goto stop;
        }
    }
    ok = true;

stop:
    if (!SoftI2C_Stop()) {
        ok = false;
    }
    return ok;
}

void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count)
{
    uint8_t i;
    for (i = 0U; i < count; i++) {
        dest[i] = source[i];
    }
}

void I2C_0_INST_IRQHandler(void)
{
    /* Hardware I2C interrupts are unused in software-I2C mode. */
}
