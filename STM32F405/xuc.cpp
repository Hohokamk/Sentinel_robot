#include "xuc.h"
#include "label.h"
#include "imu.h"
#include "CRC.h"
#include "math.h"
#include "RC.h"

XUC xuc;

void XUC::Init(UART* huart, USART_TypeDef* Instance, uint32_t BaudRate)
{
	huart->Init(Instance, BaudRate).DMARxInit(nullptr).DMATxInit();
	m_uart = huart;
	frame = m_uart->m_uartrx;
	queue_handler = &huart->UartQueueHandler;

	autoaim_controller[0].m_Kp = 0.005f;
	autoaim_controller[0].m_Td = 0.004f;

	autoaim_controller[1].m_Kp = 0.0014f;
	autoaim_controller[1].m_Td = 0.001f;
}

void XUC::Decode()
{
	pd_Rx = xQueueReceive((m_uart->UartQueueHandler), m_frame, NULL);
	if (m_frame[0] == 0xA5)
	{

		yaw_pre = yaw;
		yaw_spd = ((yaw - yaw_pre) / 0.004) * 2 * PI / 60;//0.004根据tasklist的发送频率 计算出来yaw_spd的单位是rpm

		xuc.pitch = u8_to_float(m_frame + 1) * PI / 180.f;
		xuc.yaw = u8_to_float(m_frame + 5);
		xuc.yaw_diff = u8_to_float(m_frame + 9);
		xuc.pitch_diff = u8_to_float(m_frame + 13) * PI / 180.f;
		xuc.distance = u8_to_float(m_frame + 17);
		xuc.fireadvice = m_frame[21] & 0x01;
		xuc.v_y = u8_to_float(m_frame + 25);

	}
}

void XUC::Encode()
{
	own_color = judgement.data.robot_status_t.robot_id <= 7 ? RED : BLUE;
	//TxPacket TxNuc;  // 创建一个数据包实例

	TxNuc.header = 0x5A;
	TxNuc.detect_color = !own_color;
	TxNuc.reset_tracker = 0;
	TxNuc.reserved = 15;
	TxNuc.roll = imu_pantile.GetAngleRoll();
	TxNuc.pitch = imu_pantile.GetAnglePitch();
	TxNuc.yaw = imu_pantile.GetAngleYaw();
	TxNuc.aim_x = aim_x;
	TxNuc.aim_y = aim_y;
	TxNuc.aim_z = aim_z;
	TxNuc.checksum = 0;  // 初始化校验和为0

	// 计算数据包的总大小
	int packet_size = sizeof(TxNuc);

	// 将数据包复制到发送缓冲区
	memcpy(tx_data, &TxNuc, packet_size);

	// 计算并附加 CRC16 校验和
	appendCRC16CheckSum(tx_data, packet_size);

	// 发送数据
	m_uart->UARTTransmit(tx_data, packet_size);
}

uint16_t XUC::getCRC16CheckSum(const uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC)
{
	uint8_t ch_data;

	if (pchMessage == nullptr) return 0xFFFF;
	while (dwLength--) {
		ch_data = *pchMessage++;
		(wCRC) =
			((uint16_t)(wCRC) >> 8) ^ CRC_TAB[((uint16_t)(wCRC) ^ (uint16_t)(ch_data)) & 0x00ff];
	}

	return wCRC;
}

uint32_t XUC::verifyCRC16CheckSum(const uint8_t* pchMessage, uint32_t dwLength)
{
	uint16_t w_expected = 0;

	if ((pchMessage == nullptr) || (dwLength <= 2)) return false;

	w_expected = getCRC16CheckSum(pchMessage, dwLength - 2, CRC16_INIT);
	return (
		(w_expected & 0xff) == pchMessage[dwLength - 2] &&
		((w_expected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

void XUC::appendCRC16CheckSum(uint8_t* pchMessage, uint32_t dwLength)
{
	uint16_t w_crc = 0;

	if ((pchMessage == nullptr) || (dwLength <= 2)) return;

	w_crc = getCRC16CheckSum(reinterpret_cast<uint8_t*>(pchMessage), dwLength - 2, CRC16_INIT);

	pchMessage[dwLength - 2] = (uint8_t)(w_crc & 0x00ff);
	pchMessage[dwLength - 1] = (uint8_t)((w_crc >> 8) & 0x00ff);
}
