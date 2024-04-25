/* Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <string.h>
#include <zephyr/drivers/spi.h>
#include <soc.h>

#define Mhz		1000000
#define khz		1000
#define BUFF_SIZE	5

/* master_spi and slave_spi aliases are defined in
 * overlay files to use different SPI instance if needed.
 */

#define SPIDW_NODE	DT_ALIAS(master_spi)

#define S_SPIDW_NODE	DT_ALIAS(slave_spi)

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

/* delay between greetings (in ms) */
#define SLEEPTIME 500

K_THREAD_STACK_DEFINE(MasterT_stack, STACKSIZE);
static struct k_thread MasterT_data;

K_THREAD_STACK_DEFINE(SlaveT_stack, STACKSIZE);
static struct k_thread SlaveT_data;

void test_spi_transceive(const struct device *dev,
			struct spi_cs_control *cs)
{
	struct spi_config cnfg;
	int length;
	cnfg.frequency = 1 * Mhz;
	cnfg.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(32);
	printk("transceive operation %x\n", cnfg.operation);
	cnfg.slave = 0;
	cnfg.cs = cs;

	uint32_t txdata[BUFF_SIZE] = {0xFFFFFFFF, 0xFFFFFFFE,
					 0xFFFFFFFD, 0xFFFFFFFC, 0xFFFFFFFB};
	uint32_t rxdata[BUFF_SIZE];
	length = BUFF_SIZE * sizeof(txdata[0])	;

	struct spi_buf tx_buffer[1] = {
		{
			.buf = txdata,
			.len = length
		},
	};
	struct spi_buf rx_buffer[1] = {
		{
			.buf = rxdata,
			.len = length
		},
	};
	struct spi_buf_set tx_set = {
		.buffers = &tx_buffer[0],
		.count = 1
	};
	struct spi_buf_set rx_set = {
		.buffers = &rx_buffer[0],
		.count = 1
	};

	int ret = spi_transceive(dev, &cnfg, &tx_set, &rx_set);

	printk("SPI Transceive function called with return : %d\n", ret);
	printk(" tx (i)  : %08x %08x %08x %08x %08x\n",
	       txdata[0], txdata[1], txdata[2], txdata[3], txdata[4]);
	printk(" rx (i)  : %08x %08x %08x %08x %08x\n",
	       rxdata[0], rxdata[1], rxdata[2], rxdata[3], rxdata[4]);
}

void test_spi_receive(const struct device *dev,
				 struct spi_cs_control *cs)
{
	struct spi_config cnfg;

	cnfg.frequency = 1 * Mhz;
	cnfg.operation = SPI_OP_MODE_SLAVE | SPI_WORD_SET(16);
	cnfg.slave = 0;
	cnfg.cs = NULL;

	static uint32_t rxdata[BUFF_SIZE];
	int length = (BUFF_SIZE) * sizeof(rxdata[0]);

	struct spi_buf rx_buf = {
		.buf = rxdata,
		.len = length
	};
	struct spi_buf_set rx_bufset = {
		.buffers = &rx_buf,
		.count = 1
	};

	int ret = spi_transceive(dev, &cnfg, NULL, &rx_bufset);

	printk("test SPI read only returns: %d\n", ret);
	printk("slave read: %08x %08x %08x %08x %08x\n",
		rxdata[0], rxdata[1], rxdata[2], rxdata[3], rxdata[4]);
}

/*
 * Tests SPI write only.
 * Passing only tx buffer, Output needs to be verified with a logic analyzer.
 */
void test_spi_transmit(const struct device *dev,
				 struct spi_cs_control *cs)
{
	struct spi_config cnfg;

	cnfg.frequency = 1 * Mhz;
	cnfg.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(16);
	cnfg.slave = 0;
	cnfg.cs = cs;

	static uint32_t txdata[BUFF_SIZE] = { 0xFFFF1111, 0xFFFF2222,
					0xFFFF3333, 0xFFFF4444, 0xFFFF5555};
	int length = (BUFF_SIZE) * sizeof(txdata[0]);

	struct spi_buf tx_buf = {
		.buf = txdata,
		.len = length
	};

	struct spi_buf_set tx_bufset = {
		.buffers = &tx_buf,
		.count = 1
	};

	int ret = spi_transceive(dev, &cnfg, &tx_bufset, NULL);

	printk("test SPI write only returns: %d\n", ret);
	printk("Master wrote: %08x %08x %08x %08x %08x\n",
		txdata[0], txdata[1], txdata[2], txdata[3], txdata[4]);
}

static void master_spi(void *p1, void *p2, void *p3)
{
	const struct device *const dev = DEVICE_DT_GET(SPIDW_NODE);

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	if (!device_is_ready(dev)) {
		printk("%s: Master device not ready.\n", dev->name);
		return;
	}
	struct spi_cs_control cs_ctrl = (struct spi_cs_control){
		.gpio = GPIO_DT_SPEC_GET(SPIDW_NODE, cs_gpios),
		.delay = 0u,
	};
	while (1) {
		printk("Master Transmit\n");
		test_spi_transmit(dev, &cs_ctrl);
		k_msleep(SLEEPTIME);
	}
}

static void slave_spi(void *p1, void *p2, void *p3)
{
	const struct device *const slave_dev = DEVICE_DT_GET(S_SPIDW_NODE);

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	if (!device_is_ready(slave_dev)) {
		printk("%s: Slave device not ready\n", slave_dev->name);
		return;
	}
	struct spi_cs_control slave_cs_ctrl = (struct spi_cs_control){
		.gpio = GPIO_DT_SPEC_GET(S_SPIDW_NODE, cs_gpios),
		.delay = 0u,
	};
	while (1) {
		printk("slave Receive\n");
		test_spi_receive(slave_dev, &slave_cs_ctrl);
	}
}

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(dma2), arm_dma_pl330, okay)
static void configure_lpspi_for_dma2(void)
{
	//Enable LPSPI EVTRTR channel
	#define LPSPI_DMA_GROUP					0
	#define LPSPI_DMA_RX_PERIPH_REQ			12
	#define LPSPI_DMA_TX_PERIPH_REQ			13

	#define DMA_CTRL_ACK_TYPE_Pos			(16U)
	#define DMA_CTRL_ENA					(1U << 4)

	#define HE_DMA_SEL_LPSPI_Pos           (4)
	#define HE_DMA_SEL_LPSPI_Msk           (0x3U << HE_DMA_SEL_LPSPI_Pos)

	//enable group
	sys_clear_bits(M55HE_CFG_HE_DMA_SEL, HE_DMA_SEL_LPSPI_Msk);

	sys_write32(DMA_CTRL_ENA |
			(0 << DMA_CTRL_ACK_TYPE_Pos),
			EVTRTRLOCAL_DMA_CTRL0 + (LPSPI_DMA_RX_PERIPH_REQ * 4));

	sys_write32(DMA_CTRL_ENA |
			(0 << DMA_CTRL_ACK_TYPE_Pos),
			EVTRTRLOCAL_DMA_CTRL0 + (LPSPI_DMA_TX_PERIPH_REQ * 4));
}
#elif DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(dma0), arm_dma_pl330, okay)
static void configure_spi0_lpspi_for_dma0(void)
{
	uint32_t regdata;

	//Enable SPI0 & LPSPI EVTRTR channel
	#define LPSPI_DMA_RX_PERIPH_REQ        24
	#define LPSPI_DMA_TX_PERIPH_REQ        25
	#define LPSPI_DMA_GROUP                2
	#define SPI0_DMA_RX_PERIPH_REQ         16
	#define SPI0_DMA_TX_PERIPH_REQ         20
	#define SPI0_DMA_GROUP                 2

	#define DMA_CTRL_ACK_TYPE_Pos          (16U)
	#define DMA_CTRL_ENA                   (1U << 4)

	/* Select DMA0 */
	#define HE_DMA_SEL_LPSPI_Pos           (4)
	#define HE_DMA_SEL_LPSPI_Msk           (0x3U << HE_DMA_SEL_LPSPI_Pos)


	//enable group
	regdata = sys_read32(M55HE_CFG_HE_DMA_SEL);
	regdata |= ((LPSPI_DMA_GROUP << HE_DMA_SEL_LPSPI_Pos) &
				HE_DMA_SEL_LPSPI_Msk);
	sys_write32(regdata, M55HE_CFG_HE_DMA_SEL);
	//channel enable
	sys_write32(DMA_CTRL_ENA |
			(0 << DMA_CTRL_ACK_TYPE_Pos)|
			(LPSPI_DMA_GROUP),
			EVTRTR0_DMA_CTRL0 + (LPSPI_DMA_RX_PERIPH_REQ * 4));
	regdata = sys_read32(EVTRTR0_DMA_ACK_TYPE0 + (LPSPI_DMA_GROUP * 4));
	regdata |= (1 << LPSPI_DMA_RX_PERIPH_REQ);
	sys_write32(regdata, EVTRTR0_DMA_ACK_TYPE0 + (LPSPI_DMA_GROUP * 4));

	sys_write32(DMA_CTRL_ENA |
			(0 << DMA_CTRL_ACK_TYPE_Pos)|
			(LPSPI_DMA_GROUP),
			EVTRTR0_DMA_CTRL0 + (LPSPI_DMA_TX_PERIPH_REQ * 4));
	regdata = sys_read32(EVTRTR0_DMA_ACK_TYPE0 + (LPSPI_DMA_GROUP * 4));
	regdata |= (1 << LPSPI_DMA_TX_PERIPH_REQ);
	sys_write32(regdata, EVTRTR0_DMA_ACK_TYPE0 + (LPSPI_DMA_GROUP * 4));

	sys_write32(DMA_CTRL_ENA |
			(0 << DMA_CTRL_ACK_TYPE_Pos)|
			(SPI0_DMA_GROUP),
			EVTRTR0_DMA_CTRL0 + (SPI0_DMA_RX_PERIPH_REQ * 4));
	regdata = sys_read32(EVTRTR0_DMA_ACK_TYPE0 + (SPI0_DMA_GROUP * 4));
	regdata |= (1 << SPI0_DMA_RX_PERIPH_REQ);
	sys_write32(regdata, EVTRTR0_DMA_ACK_TYPE0 + (SPI0_DMA_GROUP * 4));

	sys_write32(DMA_CTRL_ENA |
			(0 << DMA_CTRL_ACK_TYPE_Pos)|
			(SPI0_DMA_GROUP),
			EVTRTR0_DMA_CTRL0 + (SPI0_DMA_TX_PERIPH_REQ * 4));
	regdata = sys_read32(EVTRTR0_DMA_ACK_TYPE0 + (SPI0_DMA_GROUP * 4));
	regdata |= (1 << SPI0_DMA_TX_PERIPH_REQ);
	sys_write32(regdata, EVTRTR0_DMA_ACK_TYPE0 + (SPI0_DMA_GROUP * 4));
}
#endif
void main(void)
{
#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(dma2), arm_dma_pl330, okay)
	configure_lpspi_for_dma2();
#elif DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(dma0), arm_dma_pl330, okay)
	configure_spi0_lpspi_for_dma0();
#endif

	k_tid_t tids = k_thread_create(&SlaveT_data, SlaveT_stack, STACKSIZE,
			&slave_spi, NULL, NULL, NULL,
			PRIORITY, 0, K_NO_WAIT);
	if (tids == NULL) {
		printk("Error creating Slave Thread\n");
	}
	k_tid_t tidm = k_thread_create(&MasterT_data, MasterT_stack, STACKSIZE,
			&master_spi, NULL, NULL, NULL,
			PRIORITY, 0, K_NO_WAIT);
	if (tidm == NULL) {
		printk("Error creating Master Thread\n");
	}

	k_thread_start(&SlaveT_data);
	k_thread_start(&MasterT_data);
}
