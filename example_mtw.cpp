#define WIN32_LEAN_AND_MEAN
#define _WIN32_WINNT 0x0601

#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

#include <string>
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <set>
#include <list>
#include <utility>
#include <cmath>
#include <cassert>
#include <vector>
#include <array>
#include <chrono>

#include <xsensdeviceapi.h>
#include "conio.h"
#include "xsmutex.h"
#include <xstypes/xstime.h>
#include <glm/glm.hpp>


/*! \brief Stream insertion operator overload for XsPortInfo */
std::ostream& operator << (std::ostream& out, XsPortInfo const& p)
{
	out << "Port: " << std::setw(2) << std::right << p.portNumber() << " (" << p.portName().toStdString() << ") @ "
		<< std::setw(7) << p.baudrate() << " Bd"
		<< ", " << "ID: " << p.deviceId().toString().toStdString();
	return out;
}

static void debugPrintPose(size_t idx, const XsQuaternion& q, const glm::vec3& p) {
	std::cout.setf(std::ios::fixed);
	std::cout << std::setprecision(4)
		<< "[MTW " << idx << "] q=(" << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << ") "
		<< "p=(" << p.x << "," << p.y << "," << p.z << ")\n";
}

static XsVector quatRotate(const XsQuaternion& q, const XsVector& v)
{
	// quaternion (w,x,y,z)
	const double w = q.w(), x = q.x(), y = q.y(), z = q.z();
	const double vx = v[0], vy = v[1], vz = v[2];

	// q * v  (v 는 순허수 quaternion [0,v])
	const double ix = w * vx + y * vz - z * vy;
	const double iy = w * vy + z * vx - x * vz;
	const double iz = w * vz + x * vy - y * vx;
	const double iw = -x * vx - y * vy - z * vz;

	// (q * v) * q*   (q* = [w,-x,-y,-z])
	XsVector out(3);
	out[0] = ix * w + iw * (-x) + iy * (-z) - iz * (-y);
	out[1] = iy * w + iw * (-y) + iz * (-x) - ix * (-z);
	out[2] = iz * w + iw * (-z) + ix * (-y) - iy * (-x);
	return out;
}

void sendQuaternionData(SOCKET sock, const std::vector<XsQuaternion>& data)
{
	std::ostringstream oss;
	for (const auto& q : data) {
		oss << q.w() << ',' << q.x() << ',' << q.y() << ',' << q.z() << ';';
	}
	oss << '\n';
	const std::string out = oss.str();
	if (send(sock, out.c_str(), (int)out.size(), 0) == SOCKET_ERROR) {
		std::cerr << "Send failed: " << WSAGetLastError() << std::endl;
	}
}

bool trySend(SOCKET s, const char* buf, int len)
{
	int n = send(s, buf, len, 0);
	if (n == SOCKET_ERROR)
	{
		int e = WSAGetLastError();
		if (e == WSAEWOULDBLOCK)       // 커널 버퍼가 가득 차 있음 → 프레임 드롭
			return false;
		//std::cerr << "send error: " << e << '\n';
	}
	return true;
}

void sendPoseData(
	SOCKET sock,
	const std::vector<XsQuaternion>& qVec,
	const std::vector<glm::vec3>& pVec)
{
	constexpr size_t SENSOR_COUNT = 7;
	constexpr size_t FPS = 7; // qw,qx,qy,qz,px,py,pz
	std::array<float, SENSOR_COUNT* FPS> frame{};      // 49 floats = 196B

	// 기본 패딩: 단위쿼터니언
	for (size_t s = 0; s < SENSOR_COUNT; ++s) frame[s * FPS + 0] = 1.0f;

	const size_t N = std::min(qVec.size(), (size_t)SENSOR_COUNT);
	for (size_t i = 0; i < N; ++i) {
		frame[i * FPS + 0] = (float)qVec[i].w();
		frame[i * FPS + 1] = (float)qVec[i].x();
		frame[i * FPS + 2] = (float)qVec[i].y();
		frame[i * FPS + 3] = (float)qVec[i].z();
		frame[i * FPS + 4] = pVec[i].x; frame[i * FPS + 5] = pVec[i].y; frame[i * FPS + 6] = pVec[i].z;
	}

	const int bytes = (int)(frame.size() * sizeof(float));
	if (!trySend(sock, reinterpret_cast<const char*>(frame.data()), bytes)) {
		return; // 버퍼 꽉 차면 이번 프레임 통째로 스킵 (부분전송 금지)
	}
}



/*! \brief Stream insertion operator overload for XsDevice */
std::ostream& operator << (std::ostream& out, XsDevice const& d)
{
	out << "ID: " << d.deviceId().toString().toStdString() << " (" << d.productCode().toStdString() << ")";
	return out;
}

/*! \brief Given a list of update rates and a desired update rate, returns the closest update rate to the desired one */
int findClosestUpdateRate(const XsIntArray& supportedUpdateRates, const int desiredUpdateRate)
{
	if (supportedUpdateRates.empty())
	{
		return 0;
	}

	if (supportedUpdateRates.size() == 1)
	{
		return supportedUpdateRates[0];
	}

	int uRateDist = -1;
	int closestUpdateRate = -1;
	for (XsIntArray::const_iterator itUpRate = supportedUpdateRates.begin(); itUpRate != supportedUpdateRates.end(); ++itUpRate)
	{
		const int currDist = std::abs(*itUpRate - desiredUpdateRate);

		if ((uRateDist == -1) || (currDist < uRateDist))
		{
			uRateDist = currDist;
			closestUpdateRate = *itUpRate;
		}
	}
	return closestUpdateRate;
}

/*! \brief Convert a quaternion to Euler angles (roll, pitch, yaw) in radians
 *  좌표계에 따라 변환 방식이 다를 수 있으므로, 실제 적용 시 테스트 후 보정하십시오.
 */
void quaternionToEuler(const XsQuaternion& qs, double& roll, double& pitch, double& yaw)
{
	// 쿼터니언 성분 추출
	double w = qs.w();
	double x = qs.x();
	double y = qs.y();
	double z = qs.z();

	// Roll (x-axis rotation)
	roll = std::atan2(2.0 * (w * x + y * z), 1 - 2.0 * (x * x + y * y));

	// Pitch (y-axis rotation)
	pitch = std::asin(2.0 * (w * y - z * x));

	// Yaw (z-axis rotation)
	yaw = std::atan2(2.0 * (w * z + x * y), 1 - 2.0 * (y * y + z * z));
}

//----------------------------------------------------------------------
// Callback handler for wireless master
//----------------------------------------------------------------------

class WirelessMasterCallback : public XsCallback
{
public:
	typedef std::set<XsDevice*> XsDeviceSet;

	XsDeviceSet getWirelessMTWs() const
	{
		XsMutexLocker lock(m_mutex);
		return m_connectedMTWs;
	}

protected:
	virtual void onConnectivityChanged(XsDevice* dev, XsConnectivityState newState)
	{
		XsMutexLocker lock(m_mutex);
		switch (newState)
		{
		case XCS_Disconnected:		/*!< Device has disconnected, only limited informational functionality is available. */
			std::cout << "\nEVENT: MTW Disconnected -> " << *dev << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		case XCS_Rejected:			/*!< Device has been rejected and is disconnected, only limited informational functionality is available. */
			std::cout << "\nEVENT: MTW Rejected -> " << *dev << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		case XCS_PluggedIn:			/*!< Device is connected through a cable. */
			std::cout << "\nEVENT: MTW PluggedIn -> " << *dev << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		case XCS_Wireless:			/*!< Device is connected wirelessly. */
			std::cout << "\nEVENT: MTW Connected -> " << *dev << std::endl;
			m_connectedMTWs.insert(dev);
			break;
		case XCS_File:				/*!< Device is reading from a file. */
			std::cout << "\nEVENT: MTW File -> " << *dev << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		case XCS_Unknown:			/*!< Device is in an unknown state. */
			std::cout << "\nEVENT: MTW Unknown -> " << *dev << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		default:
			std::cout << "\nEVENT: MTW Error -> " << *dev << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		}
	}
private:
	mutable XsMutex m_mutex;
	XsDeviceSet m_connectedMTWs;
};

//----------------------------------------------------------------------
// Callback handler for MTw
// Handles onDataAvailable callbacks for MTW devices
//----------------------------------------------------------------------

class MtwCallback : public XsCallback
{
public:
	MtwCallback(int mtwIndex, XsDevice* device)
		: m_mtwIndex(mtwIndex)
		, m_device(device)
	{
	}

	bool dataAvailable() const
	{
		XsMutexLocker lock(m_mutex);
		return !m_packetBuffer.empty();
	}

	XsDataPacket const* getOldestPacket() const
	{
		XsMutexLocker lock(m_mutex);
		XsDataPacket const* packet = &m_packetBuffer.front();
		return packet;
	}

	void deleteOldestPacket()
	{
		XsMutexLocker lock(m_mutex);
		m_packetBuffer.pop_front();
	}

	XsDataPacket const* getLatestPacket() const {
		XsMutexLocker lock(m_mutex);
		return &m_packetBuffer.back();
	}

	void clearPackets() {
		XsMutexLocker lock(m_mutex);
		m_packetBuffer.clear();
	}

	int getMtwIndex() const
	{
		return m_mtwIndex;
	}

	XsDevice const& device() const
	{
		assert(m_device != 0);
		return *m_device;
	}

protected:
	virtual void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet)
	{
		XsMutexLocker lock(m_mutex);
		// NOTE: Processing of packets should not be done in this thread.

		m_packetBuffer.push_back(*packet);
		if (m_packetBuffer.size() > 300)
		{
			deleteOldestPacket();
		}
	}

private:
	mutable XsMutex m_mutex;
	std::list<XsDataPacket> m_packetBuffer;
	int m_mtwIndex;
	XsDevice* m_device;
};

//----------------------------------------------------------------------
// Main
//----------------------------------------------------------------------

int main(int argc, char* argv[])
{
	(void)argc;
	(void)argv;
	const int desiredUpdateRate = 75;	// Use 75 Hz update rate for MTWs
	const int desiredRadioChannel = 19;	// Use radio channel 19 for wireless master.

	WirelessMasterCallback wirelessMasterCallback; // Callback for wireless master
	std::vector<MtwCallback*> mtwCallbacks; // Callbacks for MTW devices

	std::cout << "Constructing XsControl..." << std::endl;
	XsControl* control = XsControl::construct();
	if (control == 0)
	{
		std::cout << "Failed to construct XsControl instance." << std::endl;
	}

	try
	{
		std::cout << "Scanning ports..." << std::endl;
		XsPortInfoArray detectedDevices = XsScanner::scanPorts();

		std::cout << "Finding wireless master..." << std::endl;
		XsPortInfoArray::const_iterator wirelessMasterPort = detectedDevices.begin();
		while (wirelessMasterPort != detectedDevices.end() && !wirelessMasterPort->deviceId().isWirelessMaster())
		{
			++wirelessMasterPort;
		}
		if (wirelessMasterPort == detectedDevices.end())
		{
			throw std::runtime_error("No wireless masters found");
		}
		std::cout << "Wireless master found @ " << *wirelessMasterPort << std::endl;

		std::cout << "Opening port..." << std::endl;
		if (!control->openPort(wirelessMasterPort->portName().toStdString(), wirelessMasterPort->baudrate()))
		{
			std::ostringstream error;
			error << "Failed to open port " << *wirelessMasterPort;
			throw std::runtime_error(error.str());
		}

		std::cout << "Getting XsDevice instance for wireless master..." << std::endl;
		XsDevicePtr wirelessMasterDevice = control->device(wirelessMasterPort->deviceId());
		if (wirelessMasterDevice == 0)
		{
			std::ostringstream error;
			error << "Failed to construct XsDevice instance: " << *wirelessMasterPort;
			throw std::runtime_error(error.str());
		}

		std::cout << "XsDevice instance created @ " << *wirelessMasterDevice << std::endl;

		std::cout << "Setting config mode..." << std::endl;
		if (!wirelessMasterDevice->gotoConfig())
		{
			std::ostringstream error;
			error << "Failed to goto config mode: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

		std::cout << "Attaching callback handler..." << std::endl;
		wirelessMasterDevice->addCallbackHandler(&wirelessMasterCallback);

		std::cout << "Getting the list of the supported update rates..." << std::endl;
		const XsIntArray supportedUpdateRates = wirelessMasterDevice->supportedUpdateRates();

		std::cout << "Supported update rates: ";
		for (XsIntArray::const_iterator itUpRate = supportedUpdateRates.begin(); itUpRate != supportedUpdateRates.end(); ++itUpRate)
		{
			std::cout << *itUpRate << " ";
		}
		std::cout << std::endl;

		const int newUpdateRate = findClosestUpdateRate(supportedUpdateRates, desiredUpdateRate);

		std::cout << "Setting update rate to " << newUpdateRate << " Hz..." << std::endl;
		if (!wirelessMasterDevice->setUpdateRate(newUpdateRate))
		{
			std::ostringstream error;
			error << "Failed to set update rate: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

		std::cout << "Disabling radio channel if previously enabled..." << std::endl;
		if (wirelessMasterDevice->isRadioEnabled())
		{
			if (!wirelessMasterDevice->disableRadio())
			{
				std::ostringstream error;
				error << "Failed to disable radio channel: " << *wirelessMasterDevice;
				throw std::runtime_error(error.str());
			}
		}

		std::cout << "Setting radio channel to " << desiredRadioChannel << " and enabling radio..." << std::endl;
		if (!wirelessMasterDevice->enableRadio(desiredRadioChannel))
		{
			std::ostringstream error;
			error << "Failed to set radio channel: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

		std::cout << "Waiting for MTW to wirelessly connect...\n" << std::endl;

		WSADATA wsa;
		if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
			std::cerr << "WSAStartup failed\n";
			return -1;
		}

		// 2) 서버(파이썬) 소켓 생성 & 연결
		SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (sock == INVALID_SOCKET) {
			std::cerr << "Socket 생성 실패: " << WSAGetLastError() << std::endl;
			WSACleanup();
			return -1;
		}
		{
			u_long nb = 1;
			ioctlsocket(sock, FIONBIO, &nb);
			// (선택) 버퍼 키우기 → 블로킹 확률 더 줄이고 싶으면 사용
			// int sz = 256*1024;
			// setsockopt(sock, SOL_SOCKET, SO_SNDBUF, (char*)&sz, sizeof(sz));
		}

		// 3) Nagle 끄기
		BOOL flag = TRUE;
		setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (char*)&flag, sizeof(flag));

		// 4) 서버 주소
		sockaddr_in servAddr{};
		servAddr.sin_family = AF_INET;
		servAddr.sin_port = htons(65431);
		InetPtonA(AF_INET, "127.0.0.1", &servAddr.sin_addr);

		// 5) 연결
		if (connect(sock, (sockaddr*)&servAddr, sizeof(servAddr)) == SOCKET_ERROR) {
			int e = WSAGetLastError();
			if (e != WSAEWOULDBLOCK && e != WSAEINPROGRESS) {   // 즉시 실패 케이스
				std::cerr << "Connect 실패: " << e << '\n';
				closesocket(sock); WSACleanup();
				return -1;
			}
			// 논-블로킹 connect는 이후 select()/WSAPoll() 로 완료 확인해도 되지만
			// 예제에선 로컬 loopback이므로 잠시 sleep 후 사용.
			Sleep(100);   // 0.1초 대기
		}
		std::cout << "→ Python 서버와 연결됨 (127.0.0.1:65431)\n";

		bool waitForConnections = true;
		size_t connectedMTWCount = wirelessMasterCallback.getWirelessMTWs().size();
		do
		{
			XsTime::msleep(0);

			while (true)
			{
				size_t nextCount = wirelessMasterCallback.getWirelessMTWs().size();
				if (nextCount != connectedMTWCount)
				{
					std::cout << "Number of connected MTWs: " << nextCount << ". Press 'Y' to start measurement." << std::endl;
					connectedMTWCount = nextCount;
				}
				else
				{
					break;
				}
			}
			if (_kbhit())
			{
				waitForConnections = (toupper((char)_getch()) != 'Y');
			}
		} while (waitForConnections);

		std::cout << "Starting measurement..." << std::endl;
		if (!wirelessMasterDevice->gotoMeasurement())
		{
			std::ostringstream error;
			error << "Failed to goto measurement mode: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

		std::cout << "Getting XsDevice instances for all MTWs..." << std::endl;
		XsDeviceIdArray allDeviceIds = control->deviceIds();
		XsDeviceIdArray mtwDeviceIds;
		for (XsDeviceIdArray::const_iterator i = allDeviceIds.begin(); i != allDeviceIds.end(); ++i)
		{
			if (i->isMtw())
			{
				mtwDeviceIds.push_back(*i);
			}
		}
		XsDevicePtrArray mtwDevices;
		for (XsDeviceIdArray::const_iterator i = mtwDeviceIds.begin(); i != mtwDeviceIds.end(); ++i)
		{
			XsDevicePtr mtwDevice = control->device(*i);
			if (mtwDevice != 0)
			{
				mtwDevices.push_back(mtwDevice);
			}
			else
			{
				throw std::runtime_error("Failed to create an MTW XsDevice instance");
			}
		}

		std::cout << "Attaching callback handlers to MTWs..." << std::endl;
		mtwCallbacks.resize(mtwDevices.size());
		for (int i = 0; i < (int)mtwDevices.size(); ++i)
		{
			mtwCallbacks[i] = new MtwCallback(i, mtwDevices[i]);
			mtwDevices[i]->addCallbackHandler(mtwCallbacks[i]);
		}

		std::cout << "\nMain loop. Press any key to quit\n" << std::endl;
		std::cout << "Waiting for data available..." << std::endl;

		// ───── (1) 준비부 ───────────
		std::vector<XsQuaternion> quaternionData(mtwCallbacks.size()); // Room to store quaternion data for each MTW
		std::vector<XsVector> accelerationData(mtwCallbacks.size());
		unsigned int printCounter = 0;
		
		const double dtFixed = 1.0 / newUpdateRate;            // 75 Hz = 0.0133 s ★★★
		const glm::vec3 g(0.0f, 0.0f, 9.81f);                  // 중력벡터 (world) ★★★

		// 센서마다 속도·위치 초기화 ★★★
		std::vector<glm::vec3> vel(mtwCallbacks.size(), glm::vec3(0.0f));
		std::vector<glm::vec3> pos(mtwCallbacks.size(), glm::vec3(0.0f));
		constexpr size_t POS_SENSOR = 0;

		while (!_kbhit()) {
			XsTime::msleep(0);

			bool newData = false;

			// 1) 최신 패킷 수집
			for (size_t i = 0; i < mtwCallbacks.size(); ++i) {
				if (!mtwCallbacks[i]->dataAvailable()) continue;
				newData = true;
				const XsDataPacket* packet = mtwCallbacks[i]->getLatestPacket();

				quaternionData[i] = packet->orientationQuaternion();
				accelerationData[i] = packet->freeAcceleration();

				mtwCallbacks[i]->clearPackets();
			}

			if (newData) {
				// POS_SENSOR 인덱스의 센서에 대해서만 위치 계산
				if (POS_SENSOR < accelerationData.size()) {
					// XsVector를 glm::vec3로 올바르게 변환
					glm::vec3 accW(
						static_cast<float>(accelerationData[POS_SENSOR][0]),
						static_cast<float>(accelerationData[POS_SENSOR][1]),
						static_cast<float>(accelerationData[POS_SENSOR][2])
					);

					// ZUPT (Zero Velocity Update)
					if (glm::length(accW) < 0.2f) {
						vel[POS_SENSOR] = glm::vec3(0.0f);
					}
					else {
						// 가속도 적분으로 속도 업데이트
						vel[POS_SENSOR] += accW * static_cast<float>(dtFixed);
					}

					// 속도 적분으로 위치 업데이트
					pos[POS_SENSOR] += vel[POS_SENSOR] * static_cast<float>(dtFixed);
				}

				// 나머지 센서들의 위치는 0으로 설정
				for (size_t i = 0; i < pos.size(); ++i) {
					if (i != POS_SENSOR) {
						pos[i] = glm::vec3(0.0f);
					}
				}

				// 5) 송신 (논블로킹, 버퍼 꽉 차면 프레임 드롭하고 계속)
				sendPoseData(sock, quaternionData, pos);
			}
		}
			
		// ───── 루프 종료 후 정리(여기서만 종료) ─────────────────────────────
		std::cout << "Stopping measurement..." << std::endl;

		std::cout << "Setting config mode..." << std::endl;
		if (!wirelessMasterDevice->gotoConfig()) {
			std::ostringstream error;
			error << "Failed to goto config mode: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

		std::cout << "Disabling radio... " << std::endl;
		if (!wirelessMasterDevice->disableRadio()) {
			std::ostringstream error;
			error << "Failed to disable radio: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}
		closesocket(sock);
		WSACleanup();
		}

	catch (std::exception const& ex)
		{
			std::cout << ex.what() << std::endl;
			std::cout << "****ABORT****" << std::endl;
		}
	catch (...)
		{
			std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
			std::cout << "****ABORT****" << std::endl;
		}

	std::cout << "Closing XsControl..." << std::endl;
	control->close();

	std::cout << "Deleting mtw callbacks..." << std::endl;
	for (std::vector<MtwCallback*>::iterator i = mtwCallbacks.begin(); i != mtwCallbacks.end(); ++i)
	{
		delete (*i);
	}

	std::cout << "Successful exit." << std::endl;
	std::cout << "Press [ENTER] to continue." << std::endl; std::cin.get();

	return 0;
}
