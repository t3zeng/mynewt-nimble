/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <assert.h>
#include <string.h>
#include <os/mynewt.h>
#include <nimble/ble.h>
#include <nimble/ble_hci_trans.h>
#include <nimble/hci_common.h>

#include "am_mcu_apollo.h"

#define HCI_CMD_HDR_LEN                              3       /*!< \brief Command packet header length */
#define HCI_ACL_HDR_LEN                              4       /*!< \brief ACL packet header length */
#define HCI_EVT_HDR_LEN                              2       /*!< \brief Event packet header length */

#define HCI_PKT_CMD     0x01
#define HCI_PKT_ACL     0x02
#define HCI_PKT_EVT     0x04

// Tx power level in dBm.
typedef enum
{
  TX_POWER_LEVEL_MINUS_10P0_dBm = 0x4,
  TX_POWER_LEVEL_MINUS_5P0_dBm = 0x5,
  TX_POWER_LEVEL_0P0_dBm = 0x8,
  TX_POWER_LEVEL_PLUS_3P0_dBm = 0xF,
  TX_POWER_LEVEL_INVALID = 0x10,
}txPowerLevel_t;

//*****************************************************************************
//
// Structure for holding outgoing HCI packets.
//
//*****************************************************************************
typedef struct
{
    uint32_t len;
    uint32_t data[MYNEWT_VAL(BLE_HCI_DRV_MAX_TX_PACKET) / sizeof(uint32_t)];
}
hci_drv_write_t;

uint32_t g_read_buf[MYNEWT_VAL(BLE_HCI_DRV_MAX_RX_PACKET) / sizeof(uint32_t)];

#define POOL_ACL_BLOCK_SIZE OS_ALIGN(MYNEWT_VAL(BLE_ACL_BUF_SIZE) +     \
                                     BLE_MBUF_MEMBLOCK_OVERHEAD +       \
                                     BLE_HCI_DATA_HDR_SZ, OS_ALIGNMENT)

/* BLE module handle for Ambiq HAL functions */
void *BLE;

uint8_t g_ble_mac_address[6] = {0};

struct apollo3_ble_hci_api {
#if MYNEWT_VAL(BLE_HOST)
    ble_hci_trans_rx_cmd_fn *evt_cb;
    void *evt_arg;
#endif
    ble_hci_trans_rx_acl_fn *acl_cb;
    void *acl_arg;
};

struct apollo3_ble_hci_rx_data {
    uint8_t type;
    uint8_t hdr[4];
    uint16_t len;
    uint16_t expected_len;
    union {
        uint8_t *buf;
        struct os_mbuf *om;
    };
};

/*
 * If controller-to-host flow control is enabled we need to hold an extra command
 * buffer for HCI_Host_Number_Of_Completed_Packets which can be sent at any time.
 */
#if MYNEWT_VAL(BLE_LL_CFG_FEAT_CTRL_TO_HOST_FLOW_CONTROL)
#define HCI_CMD_COUNT   2
#else
#define HCI_CMD_COUNT   1
#endif

static uint8_t apollo3_ble_hci_pool_cmd_mempool_buf[OS_MEMPOOL_BYTES(
                                                        HCI_CMD_COUNT,
                                                        BLE_HCI_TRANS_CMD_SZ)];
static struct os_mempool apollo3_ble_hci_pool_cmd_mempool;

/* Pools for HCI events (high and low priority) */
static uint8_t apollo3_ble_hci_pool_evt_hi_buf[OS_MEMPOOL_BYTES(
                                            MYNEWT_VAL(BLE_HCI_EVT_HI_BUF_COUNT),
                                            MYNEWT_VAL(BLE_HCI_EVT_BUF_SIZE))];
static struct os_mempool apollo3_ble_hci_pool_evt_hi;
static uint8_t apollo3_ble_hci_pool_evt_lo_buf[OS_MEMPOOL_BYTES(
                                            MYNEWT_VAL(BLE_HCI_EVT_LO_BUF_COUNT),
                                            MYNEWT_VAL(BLE_HCI_EVT_BUF_SIZE))];
static struct os_mempool apollo3_ble_hci_pool_evt_lo;

/* Pool for ACL data */
static uint8_t apollo3_ble_hci_pool_acl_buf[OS_MEMPOOL_BYTES(
                                            MYNEWT_VAL(BLE_ACL_BUF_COUNT),
                                            POOL_ACL_BLOCK_SIZE)];
static struct os_mempool_ext apollo3_ble_hci_pool_acl;
static struct os_mbuf_pool apollo3_ble_hci_pool_acl_mbuf;

/* Interface to host/ll */
static struct apollo3_ble_hci_api apollo3_ble_hci_api;

/* State of RX currently in progress (needs to reassemble frame) */
static struct apollo3_ble_hci_rx_data apollo3_ble_hci_rx_data;

static void
apollo3_ble_hci_trans_rx_process(void)
{
    int rc;
    uint32_t len;
    uint8_t *buf = (uint8_t *)g_read_buf;
    struct apollo3_ble_hci_rx_data *rxd = &apollo3_ble_hci_rx_data;
#if MYNEWT_VAL(BLE_HOST)
    int pool = BLE_HCI_TRANS_BUF_EVT_HI;
#endif

    memset(&apollo3_ble_hci_rx_data, 0, sizeof(apollo3_ble_hci_rx_data));

    /* Read out HCI data from controller */
    am_hal_ble_blocking_hci_read(BLE, g_read_buf, &len);

    /* Populate type with first byte of read buffer */
    rxd->type = buf[0];

#if MYNEWT_VAL(BLE_HOST)
    assert((rxd->type == HCI_PKT_ACL) || (rxd->type == HCI_PKT_EVT));
#endif

    switch (rxd->type) {
#if MYNEWT_VAL(BLE_HOST)
    case HCI_PKT_EVT:
        /* header */
        rxd->len = HCI_EVT_HDR_LEN;
        memcpy(rxd->hdr, &buf[1], rxd->len);

        if (rxd->hdr[0] == BLE_HCI_EVCODE_LE_META) {
            /* For LE Meta event we need 3 bytes to parse header, read one more byte */
            memcpy(&rxd->hdr[rxd->len], &buf[1+rxd->len],1);
            rxd->len++;

            /* Advertising reports shall be allocated from low-prio pool */
            if ((rxd->hdr[2] == BLE_HCI_LE_SUBEV_ADV_RPT) ||
                (rxd->hdr[2] == BLE_HCI_LE_SUBEV_EXT_ADV_RPT)) {
                pool = BLE_HCI_TRANS_BUF_EVT_LO;
            }
        }

        rxd->buf = ble_hci_trans_buf_alloc(pool);
        if (!rxd->buf) {
            /*
            * Only care about valid buffer when shall be allocated from
            * high-prio pool, otherwise NULL is fine and we'll just skip
            * this event.
            */
            if (pool != BLE_HCI_TRANS_BUF_EVT_LO) {
                rxd->buf = ble_hci_trans_buf_alloc(BLE_HCI_TRANS_BUF_EVT_LO);
            }
        }

        rxd->expected_len = HCI_EVT_HDR_LEN + rxd->hdr[1];
        
        /* NOTE: Ambiq Apollo3 controller does not have local supported lmp features implemented
         * The command will always return 0 so we overwrite the buffer here
         */
        if(buf[4] == 0x03 && buf[5] == 0x10 && len == 15) {
            memset(&buf[11], 0x60, sizeof(uint8_t));
        }

        if (rxd->buf) {
            memcpy(rxd->buf, rxd->hdr, rxd->len);
            memcpy(&rxd->buf[rxd->len], &buf[1+rxd->len], rxd->expected_len - rxd->len);
            rxd->len = rxd->expected_len;

            /* evt cb is ble_hs_hci_rx_evt */
            rc = apollo3_ble_hci_api.evt_cb(rxd->buf,
                                            apollo3_ble_hci_api.evt_arg);
            if (rc != 0) {
                ble_hci_trans_buf_free(rxd->buf);
            }
        } 
        break;
#endif
    case HCI_PKT_ACL:
        /* header */
        rxd->len = HCI_ACL_HDR_LEN;
        memcpy(rxd->hdr, &buf[1], rxd->len);

        /* Parse header and allocate proper buffer */
        rxd->om = os_mbuf_get_pkthdr(&apollo3_ble_hci_pool_acl_mbuf,
                                        sizeof(struct ble_mbuf_hdr));
        if (!rxd->om) {
            assert(0);
        }

        os_mbuf_append(rxd->om, rxd->hdr, rxd->len);
        rxd->expected_len = get_le16(&rxd->hdr[2]) + HCI_ACL_HDR_LEN;

        if (rxd->len != rxd->expected_len) {
            os_mbuf_append(rxd->om, &buf[1+rxd->len], rxd->expected_len - rxd->len);
            rxd->len = rxd->expected_len;
        }

        if (rxd->len == rxd->expected_len) {
            rc = apollo3_ble_hci_api.acl_cb(rxd->om, apollo3_ble_hci_api.acl_arg);
            if (rc != 0) {
                os_mbuf_free_chain(rxd->om);
            }
        }
        break;
    default:
        assert(0);
        break;
    }
}

//*****************************************************************************
//
// Interrupt handler that looks for BLECIRQ. This gets set by BLE core when there is something to read
//
//*****************************************************************************
void
apollo3_hci_int(void)
{
    /* Read and clear the interrupt status. */
    uint32_t ui32Status = am_hal_ble_int_status(BLE, true);
    am_hal_ble_int_clear(BLE, ui32Status);

    /* Handle any DMA or Command Complete interrupts. */
    am_hal_ble_int_service(BLE, ui32Status);

    /* If this was a BLEIRQ interrupt, attempt to start a read operation. */
    if (ui32Status & AM_HAL_BLE_INT_BLECIRQ)
    {
        /* Lower WAKE */
        am_hal_ble_wakeup_set(BLE, 0);

        /* Call read function to pull in data from controller */
        apollo3_ble_hci_trans_rx_process();
    }
    else {
        assert(0);
    }
}

//*****************************************************************************
//
// Boot the radio.
//
//*****************************************************************************
uint32_t
apollo3_hci_radio_boot(bool bColdBoot)
{
    uint32_t ui32NumXtalRetries = 0;

    /* Configure and enable the BLE interface. */
    uint32_t ui32Status = AM_HAL_STATUS_FAIL;
    while (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        am_hal_pwrctrl_low_power_init();
        am_hal_ble_initialize(0, &BLE);
        am_hal_ble_power_control(BLE, AM_HAL_BLE_POWER_ACTIVE);

        am_hal_ble_config_t sBleConfig =
        {
            // Configure the HCI interface clock for 6 MHz
            .ui32SpiClkCfg = AM_HAL_BLE_HCI_CLK_DIV8,

            // Set HCI read and write thresholds to 32 bytes each.
            .ui32ReadThreshold = 32,
            .ui32WriteThreshold = 32,

            // The MCU will supply the clock to the BLE core.
            .ui32BleClockConfig = AM_HAL_BLE_CORE_MCU_CLK,

            // Note: These settings only apply to Apollo3 A1/A2 silicon, not B0 silicon.
            // Default settings for expected BLE clock drift (measured in PPM).
            .ui32ClockDrift = 0,
            .ui32SleepClockDrift = 50,

            // Default setting - AGC Enabled
            .bAgcEnabled = true,

            // Default setting - Sleep Algo enabled
            .bSleepEnabled = true,
            
            // Apply the default patches when am_hal_ble_boot() is called.
            .bUseDefaultPatches = true,
        };

        am_hal_ble_config(BLE, &sBleConfig);
        //
        // Delay 1s for 32768Hz clock stability. This isn't required unless this is
        // our first run immediately after a power-up.
        //
        if ( bColdBoot )
        {
            os_time_delay(OS_TICKS_PER_SEC);
        }

        /* Attempt to boot the radio. */
        ui32Status = am_hal_ble_boot(BLE);

        // Check our status.
        if (ui32Status == AM_HAL_STATUS_SUCCESS)
        {
            // If the radio is running, we can exit this loop.
            break;
        }
        else if (ui32Status == AM_HAL_BLE_32K_CLOCK_UNSTABLE)
        {
            // If the radio is running, but the clock looks bad, we can try to
            // restart.
            am_hal_ble_power_control(BLE, AM_HAL_BLE_POWER_OFF);
            am_hal_ble_deinitialize(BLE);

            // We won't restart forever. After we hit the maximum number of
            // retries, we'll just return with failure.
            if (ui32NumXtalRetries++ < MYNEWT_VAL(BLE_HCI_DRV_MAX_XTAL_RETRIES))
            {
                os_time_delay(OS_TICKS_PER_SEC);
            }
            else
            {
                return AM_HAL_STATUS_FAIL;
            }
        }
        else
        {
            am_hal_ble_power_control(BLE, AM_HAL_BLE_POWER_OFF);
            am_hal_ble_deinitialize(BLE);
            //
            // If the radio failed for some reason other than 32K Clock
            // instability, we should just report the failure and return.
            //
            return AM_HAL_STATUS_FAIL;
        }
    }

    // Set the BLE TX Output power to 0dBm.
    am_hal_ble_tx_power_set(BLE, TX_POWER_LEVEL_0P0_dBm);

    // Enable interrupts for the BLE module.
    am_hal_ble_int_clear(BLE, (AM_HAL_BLE_INT_CMDCMP |
                               AM_HAL_BLE_INT_DCMP |
                               AM_HAL_BLE_INT_BLECIRQ));

    am_hal_ble_int_enable(BLE, (AM_HAL_BLE_INT_CMDCMP |
                                AM_HAL_BLE_INT_DCMP |
                                AM_HAL_BLE_INT_BLECIRQ));

    NVIC_EnableIRQ(BLE_IRQn);

    // When it's bColdBoot, it will use Apollo's Device ID to form Bluetooth address.
    if (bColdBoot)
    {
        am_hal_mcuctrl_device_t sDevice;
        am_hal_mcuctrl_info_get(AM_HAL_MCUCTRL_INFO_DEVICEID, &sDevice);

        // Bluetooth address formed by ChipID1 (32 bits) and ChipID0 (8-23 bits).
        memcpy(g_ble_mac_address, &sDevice.ui32ChipID1, sizeof(sDevice.ui32ChipID1));
        // ui32ChipID0 bit 8-31 is test time during chip manufacturing
        g_ble_mac_address[4] = (sDevice.ui32ChipID0 >> 8) & 0xFF;
        g_ble_mac_address[5] = (sDevice.ui32ChipID0 >> 16) & 0xFF;
    }

    return AM_HAL_STATUS_SUCCESS;
}

/* Turn off radio */
void
apollo3_hci_radio_shutdown(void)
{
    NVIC_DisableIRQ(BLE_IRQn);

    am_hal_ble_power_control(BLE, AM_HAL_BLE_POWER_OFF);

    // wait for 1s at max
    uint32_t timeout_ticks = 0;
    while (PWRCTRL->DEVPWREN_b.PWRBLEL)
    {
        if (timeout_ticks >= OS_TICKS_PER_SEC)
        {
            break;
        }
                                                                            
        os_time_delay(1);
        timeout_ticks++;
    }

    am_hal_ble_deinitialize(BLE);
}

/* Wake update helper function */
static void
apollo3_update_wake(void)
{
    AM_CRITICAL_BEGIN;

    // We want to set WAKE if there's something in the write queue, but not if
    // SPISTATUS or IRQ is high.
    if ((BLEIFn(0)->BSTATUS_b.SPISTATUS == 0) && (BLEIF->BSTATUS_b.BLEIRQ == false))
    {
        am_hal_ble_wakeup_set(BLE, 1);

        // If we've set wakeup, but IRQ came up at the same time, we should
        // just lower WAKE again.
        if (BLEIF->BSTATUS_b.BLEIRQ == true)
        {
            am_hal_ble_wakeup_set(BLE, 0);
        }
    }

    AM_CRITICAL_END;
}

//*****************************************************************************
//
// Function used by the BLE stack to send HCI messages to the BLE controller.
// The payload is placed into a queue and the controller is turned on. When it is ready
// an interrupt will fire to handle sending a message
//
//*****************************************************************************
uint8_t
apollo3_hci_write(uint8_t type, uint16_t len, uint8_t *pData)
{
    uint8_t *pui8Wptr;
    hci_drv_write_t psWriteBuffer;

    if (len > (MYNEWT_VAL(BLE_HCI_DRV_MAX_TX_PACKET)-1))  // comparison compensates for the type byte at index 0.
    {
        return 0;
    }

    /* Set all of the fields in the hci write structure. */
    psWriteBuffer.len = len + 1;

    pui8Wptr = (uint8_t *) psWriteBuffer.data;

    *pui8Wptr++ = type;

    for (uint32_t i = 0; i < len; i++)
    {
        pui8Wptr[i] = pData[i];
    }

    /* Wake up the BLE controller. */
    apollo3_update_wake();

    /* Wait on SPI status before writing */
    while ( BLEIFn(0)->BSTATUS_b.SPISTATUS )
    {
        os_time_delay(1);
    }

    am_hal_ble_blocking_hci_write(BLE, AM_HAL_BLE_RAW, psWriteBuffer.data, psWriteBuffer.len);

    return 0;
}

int
ble_hci_trans_reset(void)
{
    apollo3_hci_radio_shutdown();
    apollo3_hci_radio_boot(0);
    
    return 0;
}

static int
ble_hci_trans_acl_tx(struct os_mbuf *om)
{
    struct os_mbuf *x;
    int rc = 0;

    x = om;
    while (x) {
        rc = apollo3_hci_write(HCI_PKT_ACL, x->om_len, x->om_data);
        if (rc < 0) {
            break;
        }
        x = SLIST_NEXT(x, om_next);
    }

    os_mbuf_free_chain(om);

    return (rc < 0) ? BLE_ERR_MEM_CAPACITY : 0;
}

#if MYNEWT_VAL(BLE_HOST)
void
ble_hci_trans_cfg_hs(ble_hci_trans_rx_cmd_fn *evt_cb, void *evt_arg,
                     ble_hci_trans_rx_acl_fn *acl_cb, void *acl_arg)
{
    apollo3_ble_hci_api.evt_cb = evt_cb;
    apollo3_ble_hci_api.evt_arg = evt_arg;
    apollo3_ble_hci_api.acl_cb = acl_cb;
    apollo3_ble_hci_api.acl_arg = acl_arg;
}

int
ble_hci_trans_hs_cmd_tx(uint8_t *cmd)
{
    int len = HCI_CMD_HDR_LEN + cmd[2];
    int rc;

    rc = apollo3_hci_write(HCI_PKT_CMD, len, cmd);

    ble_hci_trans_buf_free(cmd);

    return (rc < 0) ? BLE_ERR_MEM_CAPACITY :  0;
}

int
ble_hci_trans_hs_acl_tx(struct os_mbuf *om)
{
    return ble_hci_trans_acl_tx(om);
}
#endif

uint8_t *
ble_hci_trans_buf_alloc(int type)
{
    uint8_t *buf;

    switch (type) {
    case BLE_HCI_TRANS_BUF_CMD:
        buf = os_memblock_get(&apollo3_ble_hci_pool_cmd_mempool);
        break;
    case BLE_HCI_TRANS_BUF_EVT_HI:
        buf = os_memblock_get(&apollo3_ble_hci_pool_evt_hi);
        if (buf) {
            break;
        }
        /* no break */
    case BLE_HCI_TRANS_BUF_EVT_LO:
        buf = os_memblock_get(&apollo3_ble_hci_pool_evt_lo);
        break;
    default:
        assert(0);
        buf = NULL;
    }

    return buf;
}

void
ble_hci_trans_buf_free(uint8_t *buf)
{
    int rc;

    if (os_memblock_from(&apollo3_ble_hci_pool_cmd_mempool, buf)) {
        rc = os_memblock_put(&apollo3_ble_hci_pool_cmd_mempool, buf);
        assert(rc == 0);
    } else if (os_memblock_from(&apollo3_ble_hci_pool_evt_hi, buf)) {
        rc = os_memblock_put(&apollo3_ble_hci_pool_evt_hi, buf);
        assert(rc == 0);
    } else {
        assert(os_memblock_from(&apollo3_ble_hci_pool_evt_lo, buf));
        rc = os_memblock_put(&apollo3_ble_hci_pool_evt_lo, buf);
        assert(rc == 0);
    }
}

int
ble_hci_trans_set_acl_free_cb(os_mempool_put_fn *cb, void *arg)
{
    apollo3_ble_hci_pool_acl.mpe_put_cb = cb;
    apollo3_ble_hci_pool_acl.mpe_put_arg = arg;

    return 0;
}

void
apollo3_ble_hci_init(void)
{
    int rc;

    SYSINIT_ASSERT_ACTIVE();

    rc = os_mempool_ext_init(&apollo3_ble_hci_pool_acl,
                             MYNEWT_VAL(BLE_ACL_BUF_COUNT), POOL_ACL_BLOCK_SIZE,
                             apollo3_ble_hci_pool_acl_buf,
                             "apollo3_ble_hci_pool_acl");
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = os_mbuf_pool_init(&apollo3_ble_hci_pool_acl_mbuf,
                           &apollo3_ble_hci_pool_acl.mpe_mp, POOL_ACL_BLOCK_SIZE,
                           MYNEWT_VAL(BLE_ACL_BUF_COUNT));
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = os_mempool_init(&apollo3_ble_hci_pool_evt_hi,
                         MYNEWT_VAL(BLE_HCI_EVT_HI_BUF_COUNT),
                         MYNEWT_VAL(BLE_HCI_EVT_BUF_SIZE),
                         apollo3_ble_hci_pool_evt_hi_buf,
                         "apollo3_ble_hci_pool_evt_hi");
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = os_mempool_init(&apollo3_ble_hci_pool_evt_lo,
                         MYNEWT_VAL(BLE_HCI_EVT_LO_BUF_COUNT),
                         MYNEWT_VAL(BLE_HCI_EVT_BUF_SIZE),
                         apollo3_ble_hci_pool_evt_lo_buf,
                         "apollo3_ble_hci_pool_evt_lo");
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = os_mempool_init(&apollo3_ble_hci_pool_cmd_mempool,
                         HCI_CMD_COUNT, BLE_HCI_TRANS_CMD_SZ,
                         apollo3_ble_hci_pool_cmd_mempool_buf,
                         "apollo3_ble_hci_pool_cmd_mempool");
    SYSINIT_PANIC_ASSERT(rc == 0);

    /* Enable interrupt to handle read based on BLECIRQ */
    NVIC_SetVector(BLE_IRQn, (uint32_t)apollo3_hci_int);

    /* Initial coldboot configuration */
    apollo3_hci_radio_boot(1);
}
