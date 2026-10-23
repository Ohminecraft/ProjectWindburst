/*
 * This file is part of Windburst Project
 */

#pragma once

#include "can_port.h"
#include "../../port/platform.h"
#include "can_timing.h"

// ============================================================================
// The STM32's CAN controller for DroneCAN (section can), with an external 3.3 V transceiver (TJA1051T/3, SN65HVD230 ...):
//   G4 / H5   FDCAN1 / FDCAN2 in classic CAN mode (HAL FDCAN), kernel clock = PCLK1
//   F4        bxCAN CAN1 / CAN2 (HAL CAN; CAN2 needs CAN1's clock as well)
// The controller follows the pins (PinMap_CAN_TD / PinMap_CAN_RD of STM32duino: both pins of one controller). Extended (29-bit)
// frames only - DroneCAN uses nothing else - every one of them into receive FIFO 0. No interrupt: send() puts the frame into a
// free transmit buffer or reports none free, receive() takes what is waiting (the sensor thread polls it). A bus-off is left
// by itself (FDCAN: INIT cleared again by service(); bxCAN: automatic bus-off management) and counted.
// The bit timing: a prescaler that gives 8 .. 25 time quanta a bit, the sample point at 87.5 % (CiA's recommendation).
// NOT confirmed on hardware.
// ============================================================================
#if WB_PLATFORM_STM32 && (defined(FDCAN1) || defined(CAN1))
#include <Arduino.h>
#include <string.h>
#define WB_HAS_STM32_CAN 1

class Stm32CanBus : public CanPort {
public:
    bool start(uint8_t tx_pin, uint8_t rx_pin, uint32_t bitrate) override {
        const PinName tx = (PinName)tx_pin, rx = (PinName)rx_pin;
        void* ctl = pinmap_peripheral(tx, PinMap_CAN_TD);
        if (!ctl || ctl != pinmap_peripheral(rx, PinMap_CAN_RD)) { why_ = "the CAN TX / RX pins are not the pins of one CAN controller"; state_ = FAILED; return false; }
        pin_function(tx, pinmap_function(tx, PinMap_CAN_TD));
        pin_function(rx, pinmap_function(rx, PinMap_CAN_RD));
#if defined(FDCAN1)
        RCC_PeriphCLKInitTypeDef pc = {}; pc.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
#if defined(RCC_FDCANCLKSOURCE_PCLK1)
        pc.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;          // G4: the APB1 clock
#else
        SET_BIT(RCC->PLL1CFGR, RCC_PLL1CFGR_PLL1QEN);                  // H5: PLL1's Q output (its other choices: HSE, PLL2Q)
        pc.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL1Q;
#endif
        HAL_RCCEx_PeriphCLKConfig(&pc);
        __HAL_RCC_FDCAN_CLK_ENABLE();
        cantiming::Timing tm;
        const uint32_t kclk = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
        if (!cantiming::compute(kclk, bitrate, 512, 256, 128, tm)) { why_ = "no CAN bit timing for this bitrate from the FDCAN clock"; state_ = FAILED; return false; }
        h_.Instance = (FDCAN_GlobalTypeDef*)ctl;
#if defined(FDCAN_CLOCK_DIV1)
        h_.Init.ClockDivider = FDCAN_CLOCK_DIV1;
#endif
        h_.Init.FrameFormat = FDCAN_FRAME_CLASSIC; h_.Init.Mode = FDCAN_MODE_NORMAL;
        h_.Init.AutoRetransmission = ENABLE; h_.Init.TransmitPause = DISABLE; h_.Init.ProtocolException = DISABLE;
        const uint32_t presc = tm.presc, s1 = tm.seg1, s2 = tm.seg2;
        h_.Init.NominalPrescaler = presc; h_.Init.NominalSyncJumpWidth = s2; h_.Init.NominalTimeSeg1 = s1; h_.Init.NominalTimeSeg2 = s2;
        h_.Init.DataPrescaler = presc; h_.Init.DataSyncJumpWidth = s2 > 16 ? 16 : s2; h_.Init.DataTimeSeg1 = s1 > 32 ? 32 : s1; h_.Init.DataTimeSeg2 = s2 > 16 ? 16 : s2;
        h_.Init.StdFiltersNbr = 0; h_.Init.ExtFiltersNbr = 0; h_.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
        if (HAL_FDCAN_Init(&h_) != HAL_OK) { why_ = "the FDCAN controller did not initialise"; state_ = FAILED; return false; }
        HAL_FDCAN_ConfigGlobalFilter(&h_, FDCAN_REJECT, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
        if (HAL_FDCAN_Start(&h_) != HAL_OK) { why_ = "the FDCAN controller did not start"; state_ = FAILED; return false; }
#else
        __HAL_RCC_CAN1_CLK_ENABLE();
#if defined(CAN2)
        if ((CAN_TypeDef*)ctl == CAN2) __HAL_RCC_CAN2_CLK_ENABLE();
#endif
        cantiming::Timing tm;
        if (!cantiming::compute(HAL_RCC_GetPCLK1Freq(), bitrate, 1024, 16, 8, tm)) { why_ = "no CAN bit timing for this bitrate from the bus clock"; state_ = FAILED; return false; }
        h_.Instance = (CAN_TypeDef*)ctl;
        const uint32_t presc = tm.presc, s1 = tm.seg1, s2 = tm.seg2;
        h_.Init.Prescaler = presc; h_.Init.Mode = CAN_MODE_NORMAL; h_.Init.SyncJumpWidth = CAN_SJW_1TQ;
        h_.Init.TimeSeg1 = (s1 - 1u) << CAN_BTR_TS1_Pos; h_.Init.TimeSeg2 = (s2 - 1u) << CAN_BTR_TS2_Pos;
        h_.Init.TimeTriggeredMode = DISABLE; h_.Init.AutoBusOff = ENABLE; h_.Init.AutoWakeUp = DISABLE; h_.Init.AutoRetransmission = ENABLE;
        h_.Init.ReceiveFifoLocked = DISABLE; h_.Init.TransmitFifoPriority = ENABLE;
        if (HAL_CAN_Init(&h_) != HAL_OK) { why_ = "the CAN controller did not initialise"; state_ = FAILED; return false; }
        CAN_FilterTypeDef f = {};                                  // everything into FIFO 0 (CAN2's filters start at bank 14)
        f.FilterBank = (CAN_TypeDef*)ctl == CAN1 ? 0 : 14; f.FilterMode = CAN_FILTERMODE_IDMASK; f.FilterScale = CAN_FILTERSCALE_32BIT;
        f.FilterFIFOAssignment = CAN_RX_FIFO0; f.FilterActivation = ENABLE; f.SlaveStartFilterBank = 14;
        if (HAL_CAN_ConfigFilter(&h_, &f) != HAL_OK || HAL_CAN_Start(&h_) != HAL_OK) { why_ = "the CAN controller did not start"; state_ = FAILED; return false; }
#endif
        state_ = RUNNING; why_ = "";
        return true;
    }
    const char* why() const override { return why_; }
    bool send(const dcan::Frame& fr) override {
        if (state_ != RUNNING || fr.len > 8) return false;
#if defined(FDCAN1)
        if (HAL_FDCAN_GetTxFifoFreeLevel(&h_) == 0) return false;
        FDCAN_TxHeaderTypeDef t = {};
        t.Identifier = fr.id & 0x1FFFFFFFu; t.IdType = FDCAN_EXTENDED_ID; t.TxFrameType = FDCAN_DATA_FRAME; t.DataLength = dlc(fr.len);
        t.ErrorStateIndicator = FDCAN_ESI_ACTIVE; t.BitRateSwitch = FDCAN_BRS_OFF; t.FDFormat = FDCAN_CLASSIC_CAN; t.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        uint8_t d[8]; memcpy(d, fr.data, fr.len);
        return HAL_FDCAN_AddMessageToTxFifoQ(&h_, &t, d) == HAL_OK;
#else
        if (HAL_CAN_GetTxMailboxesFreeLevel(&h_) == 0) return false;
        CAN_TxHeaderTypeDef t = {}; t.ExtId = fr.id & 0x1FFFFFFFu; t.IDE = CAN_ID_EXT; t.RTR = CAN_RTR_DATA; t.DLC = fr.len;
        uint8_t d[8]; memcpy(d, fr.data, fr.len); uint32_t mb = 0;
        return HAL_CAN_AddTxMessage(&h_, &t, d, &mb) == HAL_OK;
#endif
    }
    bool receive(dcan::Frame& fr) override {
        if (state_ == FAILED || state_ == OFF) return false;
#if defined(FDCAN1)
        while (HAL_FDCAN_GetRxFifoFillLevel(&h_, FDCAN_RX_FIFO0) > 0) {
            FDCAN_RxHeaderTypeDef r; uint8_t d[64];
            if (HAL_FDCAN_GetRxMessage(&h_, FDCAN_RX_FIFO0, &r, d) != HAL_OK) return false;
            const uint32_t len = r.DataLength > 15u ? r.DataLength >> 16 : r.DataLength;     // (older HALs keep the DLC in bits 16 .. 19)
            if (r.IdType != FDCAN_EXTENDED_ID || r.RxFrameType != FDCAN_DATA_FRAME || len > 8) continue;
            fr.id = r.Identifier; fr.len = (uint8_t)len; memcpy(fr.data, d, len);
            return true;
        }
#else
        while (HAL_CAN_GetRxFifoFillLevel(&h_, CAN_RX_FIFO0) > 0) {
            CAN_RxHeaderTypeDef r; uint8_t d[8];
            if (HAL_CAN_GetRxMessage(&h_, CAN_RX_FIFO0, &r, d) != HAL_OK) return false;
            if (r.IDE != CAN_ID_EXT || r.RTR != CAN_RTR_DATA || r.DLC > 8) continue;
            fr.id = r.ExtId; fr.len = (uint8_t)r.DLC; memcpy(fr.data, d, r.DLC);
            return true;
        }
#endif
        return false;
    }
    void service(uint32_t now_ms) override {
        if (state_ == FAILED || state_ == OFF || (uint32_t)(now_ms - check_ms_) < 20u) return;
        check_ms_ = now_ms;
#if defined(FDCAN1)
        FDCAN_ErrorCountersTypeDef ec; if (HAL_FDCAN_GetErrorCounters(&h_, &ec) == HAL_OK) { tec_ = (uint8_t)ec.TxErrorCnt; rec_ = ec.RxErrorCnt > 255 ? 255 : (uint8_t)ec.RxErrorCnt; }
        FDCAN_ProtocolStatusTypeDef ps;
        if (HAL_FDCAN_GetProtocolStatus(&h_, &ps) == HAL_OK && ps.BusOff) {
            if (state_ != BUS_OFF) { bus_offs_++; state_ = BUS_OFF; }
            CLEAR_BIT(h_.Instance->CCCR, FDCAN_CCCR_INIT);         // leave INIT: the controller waits 128 x 11 recessive bits, then runs again
        } else state_ = RUNNING;
#else
        const uint32_t esr = h_.Instance->ESR;
        tec_ = (uint8_t)((esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos); rec_ = (uint8_t)((esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos);
        if (esr & CAN_ESR_BOFF) { if (state_ != BUS_OFF) { bus_offs_++; state_ = BUS_OFF; } }       // (automatic bus-off management recovers)
        else state_ = RUNNING;
#endif
    }
    uint8_t state() const override { return state_; }
    uint8_t tec() const override { return tec_; }
    uint8_t rec() const override { return rec_; }
    uint32_t bus_offs() const override { return bus_offs_; }
private:
#if defined(FDCAN1)
    static uint32_t dlc(uint8_t n) {
#if defined(FDCAN_DLC_BYTES_8)
        static const uint32_t t[9] = {FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8};
        return t[n > 8 ? 8 : n];
#else
        return (uint32_t)n << 16;
#endif
    }
    FDCAN_HandleTypeDef h_ = {};
#else
    CAN_HandleTypeDef h_ = {};
#endif
    const char* why_ = ""; uint8_t state_ = OFF, tec_ = 0, rec_ = 0; uint32_t bus_offs_ = 0, check_ms_ = 0;
};
#else
#define WB_HAS_STM32_CAN 0
#endif
