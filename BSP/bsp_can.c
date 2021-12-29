#include "bsp_can.h"
#include "main.h"
#include "CAN_receive.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

BSP_return CAN_BSP_return_data;

/**
 * @brief:        Set up filtering ID and mask for board
 * @param[in]:    None
 * @param[out]:   can_filter_st: The set up settings for the filter transmitted to HAL layer
 * @retval:       None
 */
void BSP_CAN_Filtering_Init(void) {
	CAN_FilterTypeDef can_filter_st;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter_st.FilterIdHigh = 0x0000;
	can_filter_st.FilterIdLow = 0x0000;
	can_filter_st.FilterMaskIdHigh = 0x0000;
	can_filter_st.FilterMaskIdLow = 0x0000;
	can_filter_st.FilterBank = 0;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	can_filter_st.SlaveStartFilterBank = 14;
	can_filter_st.FilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief:        Send Data to the APP layer to do data processing when receiving data from CAN port
 * @param[in]:    hcan: The CAN port that have data transmitted in
 * @param[out]:   rx_data: The data received from HAL layer, to be transmitted to the APP layer
 * @retvel:       None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //Get Data From HAL Layer

	CAN_BSP_return_data.header = rx_header;
	for (int i = 0; i < 8; i++){
		CAN_BSP_return_data.data[i] = rx_data[i];
	}

//	BSP_CAN_Msg_Ptr(rx_header, rx_data);

	if (hcan->Instance == CAN1) {
		APP_Get_CAN1_Data(&rx_header, rx_data); //Pass data to APP Layer
	}
	else if (hcan->Instance == CAN2)
	{
		APP_Get_CAN2_Data(&rx_header, rx_data); //Pass data to APP Layer
	}
}


/**
 * @brief:        Send CenterBoard command Data to HAL layer
 * @param[in]:    hcan: CAN1 connect to all left boards, CAN2 connect to all right boards
 * @param[in]:    CommandID[]: The ID of CenterBoard
 * @param[in]:    *Command: The command for side board motor
 * @retval:        None
 */



BSP_return * BSP_CAN_Msg_Ptr(void)
{
	return &CAN_BSP_return_data;
}
