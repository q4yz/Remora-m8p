#ifndef BOARD_H
#define BOARD_H

/***********************************************************************
    BTT EBB42 V1.1 / V1.2 and BTT EBB SB2209 CAN V1.0, CAN nodes

    Both STM32G0B1CBT6, 8 MHz crystal, CAN on PB0 / PB1, USB on
    PA11 / PA12: one firmware, target EBB_SB2209 inherits EBB42 and only
    changes the board name. What node/nodeMain.cpp needs to know about
    the board (PORTING.md); the MCU family's part is
    TARGET_<FAMILY>/family.h.
************************************************************************/

#ifdef REMORA_NODE_BOARD
#define BOARD_NAME              REMORA_NODE_BOARD           // EBB_SB2209 target: "BTT EBB SB2209 CAN"
#else
#define BOARD_NAME              "BTT EBB42"
#endif

// CAN: FDCAN2 on PB0 (RX) / PB1 (TX), alternate function 3, 120R termination by jumper.
// The arguments of the family's CAN driver (canBusCreate)
#define BOARD_CAN               FDCAN2, GPIOB, 0, 1, GPIO_AF3_FDCAN2

// pins the node uses itself (CAN_PIN(port, pin) of canProtocol.h): the hub's modules cannot have them
#define BOARD_NODE_RESERVED_PINS \
    CAN_PIN(1, 0), CAN_PIN(1, 1),       /* PB0 / PB1 CAN */ \
    CAN_PIN(0, 11), CAN_PIN(0, 12),     /* PA11 / PA12 USB */

#endif
