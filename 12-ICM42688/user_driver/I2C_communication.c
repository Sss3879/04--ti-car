/*
 * Copyright (c) 2024, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "I2C_communication.h"

/* Data sent to the Target */
uint8_t gTxPacket[I2C_TX_MAX_PACKET_SIZE] = {0x3E, 0x01, 0x00, 0x40, 0x04,
    0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
/* Counters for TX length and bytes sent */
uint32_t gTxLen, gTxCount;

/* Data received from Target */
uint8_t gRxPacket[I2C_RX_MAX_PACKET_SIZE];
/* Counters for TX length and bytes sent */
uint32_t gRxLen, gRxCount;
/* I2C status */
I2cControllerStatus_t gI2cControllerStatus;

#define I2C_POLL_TIMEOUT (100000U)

static void I2C_RecoverController(void)
{
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    DL_I2C_flushControllerRXFIFO(I2C_0_INST);
    DL_I2C_resetControllerTransfer(I2C_0_INST);
}

static bool I2C_WaitForIdle(void)
{
    uint32_t timeout = I2C_POLL_TIMEOUT;
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0U) {
            I2C_RecoverController();
            return false;
        }
    }
    return true;
}

static bool I2C_WaitForBusRelease(void)
{
    uint32_t timeout = I2C_POLL_TIMEOUT;
    while (DL_I2C_getControllerStatus(I2C_0_INST) &
           DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
        if (--timeout == 0U) {
            I2C_RecoverController();
            return false;
        }
    }
    return I2C_WaitForIdle();
}

static bool I2C_WaitForRxData(void)
{
    uint32_t timeout = I2C_POLL_TIMEOUT;
    while (DL_I2C_isControllerRXFIFOEmpty(I2C_0_INST)) {
        if (--timeout == 0U) {
            I2C_RecoverController();
            return false;
        }
    }
    return true;
}

/* Reference communication sequence:
 * IDLE -> prefill FIFO -> start transfer -> refill FIFO -> BUSY clear -> IDLE. */
static bool I2C_Send(uint8_t addr, const uint8_t *data, uint16_t length)
{
    uint16_t sent;

    if ((data == NULL) || (length == 0U) || !I2C_WaitForIdle()) {
        return false;
    }

    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    sent = DL_I2C_fillControllerTXFIFO(I2C_0_INST, data, length);
    DL_I2C_startControllerTransfer(I2C_0_INST, addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, length);

    while (sent < length) {
        uint32_t timeout = I2C_POLL_TIMEOUT;
        while (DL_I2C_isControllerTXFIFOFull(I2C_0_INST)) {
            if (--timeout == 0U) {
                I2C_RecoverController();
                return false;
            }
        }
        DL_I2C_transmitControllerData(I2C_0_INST, data[sent++]);
    }

    if (!I2C_WaitForBusRelease()) {
        return false;
    }
    delay_cycles(1000);
    return true;
}

/* IDLE -> start RX -> drain FIFO -> BUSY clear -> IDLE. */
static bool I2C_Receive(uint8_t addr, uint8_t *data, uint16_t length)
{
    uint16_t i;

    if ((data == NULL) || (length == 0U) || !I2C_WaitForIdle()) {
        return false;
    }

    DL_I2C_flushControllerRXFIFO(I2C_0_INST);
    DL_I2C_startControllerTransfer(I2C_0_INST, addr,
        DL_I2C_CONTROLLER_DIRECTION_RX, length);

    for (i = 0; i < length; i++) {
        if (!I2C_WaitForRxData()) {
            return false;
        }
        data[i] = DL_I2C_receiveControllerData(I2C_0_INST);
    }

    if (!I2C_WaitForBusRelease()) {
        return false;
    }
    delay_cycles(1000);
    return true;
}

//************Array copy **********************
void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count)
{
    uint8_t copyIndex = 0;
    for (copyIndex = 0; copyIndex < count; copyIndex++) {
        dest[copyIndex] = source[copyIndex];
    }
}

//************I2C write register **********************
bool I2C_WriteReg(uint8_t addr, uint8_t reg_addr,
                  const uint8_t *reg_data, uint8_t count)
{
    uint8_t tx_data[8] = {0};
    uint8_t i;

    if ((reg_data == NULL) || (count == 0U) || (count > 7U)) {
        return false;
    }

    tx_data[0] = reg_addr;
    for (i = 0; i < count; i++) {
        tx_data[i + 1U] = reg_data[i];
    }
    return I2C_Send(addr, tx_data, (uint16_t)count + 1U);
}

//************I2C read register **********************
bool I2C_ReadReg(uint8_t addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
    if ((reg_data == NULL) || (count == 0U)) {
        return false;
    }
    /* The register-select phase transmits exactly one byte. */
    if (!I2C_Send(addr, &reg_addr, 1U)) {
        return false;
    }
    return I2C_Receive(addr, reg_data, count);
}

void I2C_0_INST_IRQHandler(void)
{
    switch (DL_I2C_getPendingInterrupt(I2C_0_INST)) {
        case DL_I2C_IIDX_CONTROLLER_RX_DONE:
            gI2cControllerStatus = I2C_STATUS_RX_COMPLETE;
            break;
        case DL_I2C_IIDX_CONTROLLER_TX_DONE:
            DL_I2C_disableInterrupt(
                I2C_0_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
            gI2cControllerStatus = I2C_STATUS_TX_COMPLETE;
            break;
        case DL_I2C_IIDX_CONTROLLER_RXFIFO_TRIGGER:
            gI2cControllerStatus = I2C_STATUS_RX_INPROGRESS;
            /* Receive all bytes from target */
            while (DL_I2C_isControllerRXFIFOEmpty(I2C_0_INST) != true) {
                if (gRxCount < gRxLen) {
                    gRxPacket[gRxCount++] =
                        DL_I2C_receiveControllerData(I2C_0_INST);
                } else {
                    /* Ignore and remove from FIFO if the buffer is full */
                    DL_I2C_receiveControllerData(I2C_0_INST);
                }
            }
            break;
        case DL_I2C_IIDX_CONTROLLER_TXFIFO_TRIGGER:
            gI2cControllerStatus = I2C_STATUS_TX_INPROGRESS;
            /* Fill TX FIFO with next bytes to send */
            if (gTxCount < gTxLen) {
                gTxCount += DL_I2C_fillControllerTXFIFO(
                    I2C_0_INST, &gTxPacket[gTxCount], gTxLen - gTxCount);
            }
            break;
            /* Not used for this example */
        case DL_I2C_IIDX_CONTROLLER_ARBITRATION_LOST:
        case DL_I2C_IIDX_CONTROLLER_NACK:
            if ((gI2cControllerStatus == I2C_STATUS_RX_STARTED) ||
                (gI2cControllerStatus == I2C_STATUS_TX_STARTED)) {
                /* NACK interrupt if I2C Target is disconnected */
                gI2cControllerStatus = I2C_STATUS_ERROR;
            }
        case DL_I2C_IIDX_CONTROLLER_RXFIFO_FULL:
        case DL_I2C_IIDX_CONTROLLER_TXFIFO_EMPTY:
        case DL_I2C_IIDX_CONTROLLER_START:
        case DL_I2C_IIDX_CONTROLLER_STOP:
        case DL_I2C_IIDX_CONTROLLER_EVENT1_DMA_DONE:
        case DL_I2C_IIDX_CONTROLLER_EVENT2_DMA_DONE:
        default:
            break;
    }
}
