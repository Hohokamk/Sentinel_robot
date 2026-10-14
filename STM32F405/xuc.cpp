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

void XUC::Decode()//解码接收数据
{
	//检查收到的数据包是否正确，四个过滤
	if (!m_uart) return;
	pd_Rx = xQueueReceive((m_uart->UartQueueHandler), m_frame, NULL);
	if (pd_Rx != pdTRUE) return;

	++rx_total;
	if (m_frame[0] != 0xA5) return;// 没数据就退出，别等
	if (!VerifyCRC16CheckSum(m_frame, XUC_FRAME_LEN)) { ++crc_err; return; }
	//开始解码

		yaw = FR4(m_frame + 5) * PI / 180.f;                      // ① 先更新当前值
		yaw_spd = ((yaw - yaw_pre) / 0.005) * 2 * PI / 60;        // ② 再求差
		yaw_pre = yaw;                                            // ③ 最后保存
		pitch = FR4(m_frame + 1) * PI / 180.f;

		yaw_diff = FR4(m_frame + 9) * PI / 180.f;
		pitch_diff = FR4(m_frame + 13) * PI / 180.f;
		distance = FR4(m_frame + 17);
		fireadvice = m_frame[21] & 0x01;
		v_y = FR4(m_frame + 25);

		// 导航下行（帧尾追加，与自瞄同帧）
		speed_x = FR4(m_frame + 30);
		speed_y = FR4(m_frame + 34);

	++rx_count;
}

void XUC::Encode()//编码要发送的数据
{
	own_color = judgement.data.robot_status_t.robot_id <= 7 ? RED : BLUE;

	TxNuc.header = 0x5A;
	TxNuc.detect_color = !own_color;
	TxNuc.reset_tracker = 0;          // TODO: 进 AUTO 的边沿置 1 发一帧
	TxNuc.reserved = 15;

	// ===== 云台 IMU 块（p_*）=====
	TxNuc.p_roll = imu_pantile.GetAngleRoll();
	TxNuc.p_pitch = imu_pantile.GetAnglePitch();
	TxNuc.p_yaw = imu_pantile.GetAngleYaw();
	TxNuc.p_gx = imu_pantile.GetGyroRoll();
	TxNuc.p_gy = imu_pantile.GetGyroPitch();
	TxNuc.p_gz = imu_pantile.GetGyroYaw();
	TxNuc.p_time = imu_pantile.GetTimestamp();
	TxNuc.p_id = imu_pantile.id;
	TxNuc.p_valid = imu_pantile.IsValid() ? 1 : 0;

	// ===== 底盘 IMU 块（c_*，无角速度）=====
	TxNuc.c_roll = imu_chassis.GetAngleRoll();
	TxNuc.c_pitch = imu_chassis.GetAnglePitch();
	TxNuc.c_yaw = imu_chassis.GetAngleYaw();
	TxNuc.c_time = imu_chassis.GetTimestamp();
	TxNuc.c_id = imu_chassis.id;
	TxNuc.c_valid = imu_chassis.IsValid() ? 1 : 0;

	// ===== 回传计数 =====
	TxNuc.rx_total = rx_total;
	TxNuc.rx_count = rx_count;

	TxNuc.checksum = 0;                       // 可留可不留，CRC 会覆写它
	// 计算数据包的总大小
	int packet_size = sizeof(TxNuc);          // = 64
	// 将数据包复制到发送缓冲区,缓冲区大小为96
	memcpy(tx_data, &TxNuc, packet_size);
	// 计算并附加 CRC16 校验和
	AppendCRC16CheckSum(tx_data, packet_size);   // 覆盖 [0..61]，CRC 落 [62..63]
	// 发送数据
	m_uart->DMATransmit(tx_data, packet_size);//AI说这个比下面那个好
	//m_uart->UARTTransmit(tx_data, packet_size);
}

//没有了，ai说在CRC.h里有了，下面是原来的代码，注释掉了