/**
  ******************************************************************************
  * @file    stm32f4x7_eth_bsp.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    31-July-2013 
  * @brief   STM32F4x7 Ethernet hardware configuration.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f429_eth.h"
#include "LAN8742A.h"
#include "netconf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ETH_InitTypeDef ETH_InitStructure;
__IO uint32_t  EthStatus = 0;
__IO uint8_t EthLinkStatus = 0;

/* Private function prototypes -----------------------------------------------*/
static void ETH_GPIO_Config(void);
static void ETH_MACDMA_Config(void);

/* Private functions ---------------------------------------------------------*/
/* Bit 2 from Basic Status Register in PHY */
#define GET_PHY_LINK_STATUS()		(ETH_ReadPHYRegister(ETHERNET_PHY_ADDRESS, PHY_BSR) & 0x00000004)

/**
  * @brief  ETH_BSP_Config
  * @param  None
  * @retval None
  */
void Delay_ms(__IO u32 nTime);

void ETH_BSP_Config(void)
{
  /* Configure the GPIO ports for ethernet pins */
  ETH_GPIO_Config();
	
//	GPIO_SetBits(ETH_PWR_PORT,ETH_PWR_PIN);
	
	GPIO_SetBits(ETH_NRST_PORT,ETH_NRST_PIN);
	Delay_ms(10);
	GPIO_ResetBits(ETH_NRST_PORT,ETH_NRST_PIN);
	Delay_ms(10);
	GPIO_SetBits(ETH_NRST_PORT,ETH_NRST_PIN);

  /* Configure the Ethernet MAC/DMA */
  ETH_MACDMA_Config();

  /* Get Ethernet link status*/
  if(GET_PHY_LINK_STATUS())
  {
    EthStatus |= ETH_LINK_FLAG;
  }
}

/**
  * @brief  Configures the Ethernet Interface
  * @param  None
  * @retval None
  */
static void ETH_MACDMA_Config(void)
{
  /* Enable ETHERNET clock  */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_ETH_MAC | RCC_AHB1Periph_ETH_MAC_Tx |
                        RCC_AHB1Periph_ETH_MAC_Rx, ENABLE);
                        
  /* Reset ETHERNET on AHB Bus */
  ETH_DeInit();

  /* Software reset */
  ETH_SoftwareReset();

  /* Wait for software reset */
  while (ETH_GetSoftwareResetStatus() == SET);

  /* ETHERNET Configuration --------------------------------------------------*/
  /* Call ETH_StructInit if you don't like to configure all ETH_InitStructure parameter */
  ETH_StructInit(&ETH_InitStructure);

  /* Fill ETH_InitStructure parametrs */
  /*------------------------   MAC   -----------------------------------*/
	/* 开启网络自适应功能 */
  ETH_InitStructure.ETH_AutoNegotiation = ETH_AutoNegotiation_Enable;
//  ETH_InitStructure.ETH_AutoNegotiation = ETH_AutoNegotiation_Disable; 
//  ETH_InitStructure.ETH_Speed = ETH_Speed_10M;
//  ETH_InitStructure.ETH_Mode = ETH_Mode_FullDuplex;   
  /* 关闭反馈 */
  ETH_InitStructure.ETH_LoopbackMode = ETH_LoopbackMode_Disable;
	/* 关闭重传功能 */
  ETH_InitStructure.ETH_RetryTransmission = ETH_RetryTransmission_Disable;
	/* 关闭自动去除PDA/CRC功能  */
  ETH_InitStructure.ETH_AutomaticPadCRCStrip = ETH_AutomaticPadCRCStrip_Disable;
	/* 关闭接收所有的帧 */
  ETH_InitStructure.ETH_ReceiveAll = ETH_ReceiveAll_Disable;
	/* 允许接收所有广播帧 */
  ETH_InitStructure.ETH_BroadcastFramesReception = ETH_BroadcastFramesReception_Enable;
	/* 关闭混合模式的地址过滤  */
  ETH_InitStructure.ETH_PromiscuousMode = ETH_PromiscuousMode_Disable;
	/* 对于组播地址使用完美地址过滤    */
  ETH_InitStructure.ETH_MulticastFramesFilter = ETH_MulticastFramesFilter_Perfect;
	/* 对单播地址使用完美地址过滤  */
  ETH_InitStructure.ETH_UnicastFramesFilter = ETH_UnicastFramesFilter_Perfect;
#ifdef CHECKSUM_BY_HARDWARE
	/* 开启ipv4和TCP/UDP/ICMP的帧校验和卸载   */
  ETH_InitStructure.ETH_ChecksumOffload = ETH_ChecksumOffload_Enable;
#endif

  /*------------------------   DMA   -----------------------------------*/  

  /* When we use the Checksum offload feature, we need to enable the Store and Forward mode:
  the store and forward guarantee that a whole frame is stored in the FIFO, so the MAC can insert/verify the checksum, 
  if the checksum is OK the DMA can handle the frame otherwise the frame is dropped */
	/*当我们使用帧校验和卸载功能的时候，一定要使能存储转发模式,存储转发模式中要保证整个帧存储在FIFO中,
	这样MAC能插入/识别出帧校验值,当真校验正确的时候DMA就可以处理帧,否则就丢弃掉该帧*/
	
	/* 开启丢弃TCP/IP错误帧 */
  ETH_InitStructure.ETH_DropTCPIPChecksumErrorFrame = ETH_DropTCPIPChecksumErrorFrame_Enable;
	/* 开启接收数据的存储转发模式  */
  ETH_InitStructure.ETH_ReceiveStoreForward = ETH_ReceiveStoreForward_Enable;
	/* 开启发送数据的存储转发模式   */
  ETH_InitStructure.ETH_TransmitStoreForward = ETH_TransmitStoreForward_Enable;

	/* 禁止转发错误帧 */
  ETH_InitStructure.ETH_ForwardErrorFrames = ETH_ForwardErrorFrames_Disable;
	/* 不转发过小的好帧 */
  ETH_InitStructure.ETH_ForwardUndersizedGoodFrames = ETH_ForwardUndersizedGoodFrames_Disable;
	/* 打开处理第二帧功能 */
  ETH_InitStructure.ETH_SecondFrameOperate = ETH_SecondFrameOperate_Enable;
	/* 开启DMA传输的地址对齐功能 */
  ETH_InitStructure.ETH_AddressAlignedBeats = ETH_AddressAlignedBeats_Enable;
	/* 开启固定突发功能 */
  ETH_InitStructure.ETH_FixedBurst = ETH_FixedBurst_Enable;
	/* DMA发送的最大突发长度为32个节拍 */
  ETH_InitStructure.ETH_RxDMABurstLength = ETH_RxDMABurstLength_32Beat;
	/*DMA接收的最大突发长度为32个节拍 */
  ETH_InitStructure.ETH_TxDMABurstLength = ETH_TxDMABurstLength_32Beat;
  ETH_InitStructure.ETH_DMAArbitration = ETH_DMAArbitration_RoundRobin_RxTx_2_1;

  /* Configure Ethernet */
	/* 配置ETH */
  EthStatus = ETH_Init(&ETH_InitStructure, ETHERNET_PHY_ADDRESS);
}

void mcoinit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//??PA

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;//PA8
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//??????

  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//????
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHz


  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	RCC_MCO1Config(RCC_MCO1Source_HSE, RCC_MCO1Div_1);
}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
void ETH_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

/* Enable GPIOs clocks */
  RCC_AHB1PeriphClockCmd(ETH_MDIO_GPIO_CLK            | ETH_MDC_GPIO_CLK          |
                         ETH_RMII_REF_CLK_GPIO_CLK    | ETH_RMII_CRS_DV_GPIO_CLK  |
                         ETH_RMII_RXD0_GPIO_CLK       | ETH_RMII_RXD1_GPIO_CLK    |
                         ETH_RMII_TX_EN_GPIO_CLK      | ETH_RMII_TXD0_GPIO_CLK    |
                         ETH_RMII_TXD1_GPIO_CLK , ENABLE);

  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	mcoinit();

  /* MII/RMII Media interface selection --------------------------------------*/
#ifdef MII_MODE /* Mode MII with STM324xG-EVAL  */
 #ifdef PHY_CLOCK_MCO

  /* Output HSE clock (25MHz) on MCO pin (PA8) to clock the PHY */
  RCC_MCO1Config(RCC_MCO1Source_HSE, RCC_MCO1Div_1);
 #endif /* PHY_CLOCK_MCO */

  SYSCFG_ETH_MediaInterfaceConfig(SYSCFG_ETH_MediaInterface_MII);
#elif defined RMII_MODE  /* Mode RMII with STM324xG-EVAL */

  SYSCFG_ETH_MediaInterfaceConfig(SYSCFG_ETH_MediaInterface_RMII);
#endif



/* Ethernet pins configuration ************************************************/
/*
	ETH_RST --------------------------> PC0
	ETH_MDIO -------------------------> PA2
	ETH_MDC --------------------------> PC1
	ETH_MII_RX_CLK/ETH_RMII_REF_CLK---> PA1
	ETH_MII_RX_DV/ETH_RMII_CRS_DV ----> PA7
	ETH_MII_RXD0/ETH_RMII_RXD0 -------> PC4
	ETH_MII_RXD1/ETH_RMII_RXD1 -------> PC5
	ETH_MII_TX_EN/ETH_RMII_TX_EN -----> PB11
	ETH_MII_TXD0/ETH_RMII_TXD0 -------> PB12
	ETH_MII_TXD1/ETH_RMII_TXD1 -------> PB13
																						*/
  /* Configure ETH_NRST */
	GPIO_InitStructure.GPIO_Pin = ETH_NRST_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(ETH_NRST_PORT, &GPIO_InitStructure);

   /* Configure ETH_MDIO */
  GPIO_InitStructure.GPIO_Pin = ETH_MDIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(ETH_MDIO_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(ETH_MDIO_PORT, ETH_MDIO_SOURCE, ETH_MDIO_AF);
	
	/* Configure ETH_MDC */
  GPIO_InitStructure.GPIO_Pin = ETH_MDC_PIN;
  GPIO_Init(ETH_MDC_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(ETH_MDC_PORT, ETH_MDC_SOURCE, ETH_MDC_AF);
	
	/* Configure ETH_RMII_REF_CLK */
  GPIO_InitStructure.GPIO_Pin = ETH_RMII_REF_CLK_PIN;
  GPIO_Init(ETH_RMII_REF_CLK_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(ETH_RMII_REF_CLK_PORT, ETH_RMII_REF_CLK_SOURCE, ETH_RMII_REF_CLK_AF);
	
	/* Configure ETH_RMII_CRS_DV */
  GPIO_InitStructure.GPIO_Pin = ETH_RMII_CRS_DV_PIN;
  GPIO_Init(ETH_RMII_CRS_DV_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(ETH_RMII_CRS_DV_PORT, ETH_RMII_CRS_DV_SOURCE, ETH_RMII_CRS_DV_AF);
	
	/* Configure ETH_RMII_RXD0 */
  GPIO_InitStructure.GPIO_Pin = ETH_RMII_RXD0_PIN;
  GPIO_Init(ETH_RMII_RXD0_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(ETH_RMII_RXD0_PORT, ETH_RMII_RXD0_SOURCE, ETH_RMII_RXD0_AF);
	
	/* Configure ETH_RMII_RXD1 */
  GPIO_InitStructure.GPIO_Pin = ETH_RMII_RXD1_PIN;
  GPIO_Init(ETH_RMII_RXD1_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(ETH_RMII_RXD1_PORT, ETH_RMII_RXD1_SOURCE, ETH_RMII_RXD1_AF);
	
	/* Configure ETH_RMII_TX_EN */
  GPIO_InitStructure.GPIO_Pin = ETH_RMII_TX_EN_PIN;
  GPIO_Init(ETH_RMII_TX_EN_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(ETH_RMII_TX_EN_PORT, ETH_RMII_TX_EN_SOURCE, ETH_RMII_TX_EN_AF);
	
	/* Configure ETH_RMII_TXD0 */
  GPIO_InitStructure.GPIO_Pin = ETH_RMII_TXD0_PIN;
  GPIO_Init(ETH_RMII_TXD0_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(ETH_RMII_TXD0_PORT, ETH_RMII_TXD0_SOURCE, ETH_RMII_TXD0_AF);
	
	/* Configure ETH_RMII_TXD1 */
  GPIO_InitStructure.GPIO_Pin = ETH_RMII_TXD1_PIN;
  GPIO_Init(ETH_RMII_TXD1_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(ETH_RMII_TXD1_PORT, ETH_RMII_TXD1_SOURCE, ETH_RMII_TXD1_AF);		
}

/* This function is called periodically each second */
/* It checks link status for ethernet controller */
void ETH_CheckLinkStatus(uint16_t PHYAddress) 
{
	static uint8_t status = 0;
	uint32_t t = GET_PHY_LINK_STATUS();
	
	/* If we have link and previous check was not yet */
	if (t && !status) {
		/* Set link up */
		//netif_set_link_up(&gnetif);
		
		status = 1;
	}	
	/* If we don't have link and it was on previous check */
	if (!t && status) {
		EthLinkStatus = 1;
		/* Set link down */
		//netif_set_link_down(&gnetif);
			
		status = 0;
	}
}

/**
  * @brief  Link callback function, this function is called on change of link status.
  * @param  The network interface
  * @retval None
  */


void ETH_EXTERN_GetSpeedAndDuplex(uint32_t PHYAddress, ETH_InitTypeDef* ETH_InitStruct) 
{
	uint32_t RegValue;

/* LAN8720A */
	/* Read status register, register number 31 = 0x1F */
	RegValue = ETH_ReadPHYRegister(ETHERNET_PHY_ADDRESS, 0x1F);
	/* Mask out bits which are not for speed and link indication, bits 4:2 are used */
	RegValue = (RegValue >> 2) & 0x07;

	/* Switch statement */
	switch (RegValue) {
		case 1: /* Base 10, half-duplex */
			ETH_InitStruct->ETH_Speed = ETH_Speed_10M;
			ETH_InitStruct->ETH_Mode = ETH_Mode_HalfDuplex;
			break;
		case 2: /* Base 100, half-duplex */
			ETH_InitStruct->ETH_Speed = ETH_Speed_100M;
			ETH_InitStruct->ETH_Mode = ETH_Mode_HalfDuplex;
			break;
		case 5: /* Base 10, full-duplex */
			ETH_InitStruct->ETH_Speed = ETH_Speed_10M;
			ETH_InitStruct->ETH_Mode = ETH_Mode_FullDuplex;
			break;
		case 6: /* Base 100, full-duplex */
			ETH_InitStruct->ETH_Speed = ETH_Speed_100M;
			ETH_InitStruct->ETH_Mode = ETH_Mode_FullDuplex;
			break;
		default:
			break;
	}
/* LAN8720A */	  
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
